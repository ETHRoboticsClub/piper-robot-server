import asyncio
import websockets
import json
import ssl
import os
import time
import numpy as np
import torch
import math
import sys
import logging
from typing import Dict, Optional, Tuple
from scipy.spatial.transform import Rotation as R  # Add this for quaternion operations

import pybullet as p
import pybullet_data

from lerobot.common.robot_devices.robots.manipulator import ManipulatorRobot
from lerobot.common.robot_devices.robots.configs import So100RobotConfig
from lerobot.common.robot_devices.motors.configs import FeetechMotorsBusConfig
from lerobot.common.robot_devices.utils import RobotDeviceNotConnectedError

# --- Logging Setup ---
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# --- SSL Configuration ---
CERTFILE = 'cert.pem'
KEYFILE = 'key.pem'

# --- Constants ---
VR_TO_ROBOT_SCALE = 1.0  # Scale factor for VR movements (adjust as needed)
SEND_INTERVAL = 0.05     # Send robot commands every 50ms (20Hz)
POSITION_SMOOTHING = 0.1 # Smoothing factor for position updates (0=no smoothing, 1=full smoothing)

# IK Configuration
USE_ORIENTATION_CONTROL = False  # Enable/disable orientation control in IK

# PyBullet Visualization Markers:
# - Red sphere: Left arm current end effector position
# - Blue sphere: Right arm current end effector position  
# - Green sphere: Left arm goal end effector position (from VR controller)
# - Yellow sphere: Right arm goal end effector position (from VR controller)
# Note: IK uses position-only control for 4-DOF arm movement

# Joint configuration
common_motors = {
    "shoulder_pan": [1, "sts3215"],
    "shoulder_lift": [2, "sts3215"], 
    "elbow_flex": [3, "sts3215"],
    "wrist_flex": [4, "sts3215"],
    "wrist_roll": [5, "sts3215"],
    "gripper": [6, "sts3215"],
}

JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
NUM_JOINTS = len(JOINT_NAMES)
NUM_IK_JOINTS = 4  # Use only first 4 joints: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex
WRIST_ROLL_INDEX = 4
GRIPPER_INDEX = 5

# --- Robot Configuration (Both Physical Arms as Followers) ---
robot_config = So100RobotConfig(
    follower_arms={
        "left": FeetechMotorsBusConfig(port="/dev/ttySO100follower", motors=common_motors.copy()),
        "right": FeetechMotorsBusConfig(port="/dev/ttySO100leader", motors=common_motors.copy())
    },
    cameras={}
)

# --- Global State ---
class VRTeleopState:
    def __init__(self):
        # Robot (single instance managing both arms as followers)
        self.robot = None
        self.robot_connected = False
        
        # VR Controller states
        self.left_controller_data = None
        self.right_controller_data = None
        self.left_grip_active = False
        self.right_grip_active = False
        self.left_origin_position = None  # VR position when grip was first pressed
        self.right_origin_position = None
        
        # VR Controller rotation tracking
        self.left_origin_rotation = None  # VR rotation quaternion when grip was first pressed
        self.right_origin_rotation = None
        self.left_z_axis_rotation = 0.0   # Relative rotation around forward axis in degrees
        self.right_z_axis_rotation = 0.0
        self.left_origin_wrist_angle = 0.0  # Robot's wrist_roll angle when grip was pressed
        self.right_origin_wrist_angle = 0.0
        
        # Current goal states for visualization
        self.left_goal_position = None
        self.right_goal_position = None
        
        # Robot arm states (current joint angles) - now both are followers
        self.left_arm_joint_angles = np.zeros(NUM_JOINTS)   # Left follower arm
        self.right_arm_joint_angles = np.zeros(NUM_JOINTS)  # Right follower arm
        self.left_arm_origin_position = None  # Robot EF position when grip was pressed
        self.right_arm_origin_position = None
        
        # PyBullet
        self.physics_client = None
        self.robot_id = None
        self.p_joint_indices = [None] * NUM_JOINTS
        self.end_effector_link_index = -1
        self.viz_markers = {}
        
        # Joint limits (will be read from URDF)
        self.joint_limits_min_deg = np.full(NUM_JOINTS, -180.0)
        self.joint_limits_max_deg = np.full(NUM_JOINTS, 180.0)
        
        # Control timing
        self.last_send_time = 0
        
        # Logging timing - new additions for throttled logging
        self.last_log_time = 0
        self.log_interval = 1.0  # Log once per second
        
        # WebSocket
        self.websocket_clients = set()

# Global state instance
state = VRTeleopState()

# --- SSL Setup ---
def setup_ssl():
    if not os.path.exists(CERTFILE) or not os.path.exists(KEYFILE):
        logger.error(f"Certificate ('{CERTFILE}') or Key ('{KEYFILE}') not found for WebSocket server.")
        logger.error("Please ensure these files exist. You might need to generate them first.")
        return None
    
    ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    try:
        ssl_context.load_cert_chain(certfile=CERTFILE, keyfile=KEYFILE)
        logger.info("SSL certificate and key loaded successfully.")
        return ssl_context
    except ssl.SSLError as e:
        logger.error(f"Error loading SSL cert/key: {e}")
        return None

# --- PyBullet Setup ---
def setup_pybullet():
    try:
        state.physics_client = p.connect(p.GUI)
    except p.error as e:
        logger.warning(f"Could not connect to PyBullet GUI: {e}")
        try:
            state.physics_client = p.connect(p.DIRECT)
        except p.error:
            logger.error("Failed to connect to PyBullet")
            return False
    
    if state.physics_client < 0:
        return False
    
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.loadURDF("plane.urdf")
    
    # Load robot URDF
    urdf_path = "URDF/SO_5DOF_ARM100_8j/urdf/so100.urdf"
    if not os.path.exists(urdf_path):
        logger.error(f"URDF file not found: {urdf_path}")
        return False
    
    try:
        state.robot_id = p.loadURDF(urdf_path, [0, 0, 0], [0, 0, 0, 1], useFixedBase=1)
    except p.error as e:
        logger.error(f"Failed to load URDF: {e}")
        return False
    
    # Map joint names to PyBullet indices
    num_joints = p.getNumJoints(state.robot_id)
    urdf_to_internal_map = {
        "Rotation": "shoulder_pan",
        "Pitch": "shoulder_lift", 
        "Elbow": "elbow_flex",
        "Wrist_Pitch": "wrist_flex",
        "Wrist_Roll": "wrist_roll",
        "Jaw": "gripper",
    }
    
    p_name_to_index = {}
    for i in range(num_joints):
        info = p.getJointInfo(state.robot_id, i)
        joint_name = info[1].decode('UTF-8')
        p_name_to_index[joint_name] = i
        if info[2] != p.JOINT_FIXED:
            p.setJointMotorControl2(state.robot_id, i, p.VELOCITY_CONTROL, force=0)
    
    # Map to our joint indices
    for urdf_name, internal_name in urdf_to_internal_map.items():
        if internal_name in JOINT_NAMES and urdf_name in p_name_to_index:
            target_idx = JOINT_NAMES.index(internal_name)
            state.p_joint_indices[target_idx] = p_name_to_index[urdf_name]
    
    # Find end effector link
    for i in range(num_joints):
        info = p.getJointInfo(state.robot_id, i)
        link_name = info[12].decode('UTF-8')  # Link name, not joint name
        if link_name == "Fixed_Jaw_tip":
            state.end_effector_link_index = i
            logger.info(f"Using Fixed_Jaw_tip link (index {i}) as IK target")
            break
    
    if state.end_effector_link_index == -1:
        logger.error("Could not find Fixed_Jaw_tip link")
        return False
    
    # Read joint limits
    for i in range(NUM_JOINTS):
        pb_index = state.p_joint_indices[i]
        if pb_index is not None:
            joint_info = p.getJointInfo(state.robot_id, pb_index)
            lower, upper = joint_info[8], joint_info[9]
            if lower < upper:
                state.joint_limits_min_deg[i] = math.degrees(lower)
                state.joint_limits_max_deg[i] = math.degrees(upper)
    
    # Create visualization markers with coordinate frames
    target_shape = p.createVisualShape(p.GEOM_SPHERE, radius=0.02, rgbaColor=[1, 0, 0, 0.8])
    state.viz_markers['left_target'] = p.createMultiBody(baseVisualShapeIndex=target_shape, basePosition=[0, 0, 1])
    
    actual_shape = p.createVisualShape(p.GEOM_SPHERE, radius=0.02, rgbaColor=[0, 0, 1, 0.8])
    state.viz_markers['right_target'] = p.createMultiBody(baseVisualShapeIndex=actual_shape, basePosition=[0, 0, 1])
    
    # Add goal position markers (green for left goal, yellow for right goal)
    left_goal_shape = p.createVisualShape(p.GEOM_SPHERE, radius=0.025, rgbaColor=[0, 1, 0, 0.9])
    state.viz_markers['left_goal'] = p.createMultiBody(baseVisualShapeIndex=left_goal_shape, basePosition=[0, 0, 1])

    right_goal_shape = p.createVisualShape(p.GEOM_SPHERE, radius=0.025, rgbaColor=[1, 1, 0, 0.9])
    state.viz_markers['right_goal'] = p.createMultiBody(baseVisualShapeIndex=right_goal_shape, basePosition=[0, 0, 1])
    
    # Add coordinate frame visualizations
    state.viz_markers['left_target_frame'] = []
    state.viz_markers['right_target_frame'] = []
    state.viz_markers['left_goal_frame'] = []
    state.viz_markers['right_goal_frame'] = []
    
    # Create axis lines for each marker (X=red, Y=green, Z=blue)
    axis_length = 0.05
    axis_colors = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]  # RGB for XYZ
    
    for marker_name in ['left_target_frame', 'right_target_frame', 'left_goal_frame', 'right_goal_frame']:
        frame_lines = []
        for i in range(3):  # X, Y, Z axes
            line_id = p.addUserDebugLine([0, 0, -1], [0, 0, -1], lineColorRGB=axis_colors[i], lineWidth=3)
            frame_lines.append(line_id)
        state.viz_markers[marker_name] = frame_lines
    
    logger.info("PyBullet setup complete")
    return True

# --- Coordinate Transformations ---
def vr_to_robot_coordinates(vr_pos: Dict[str, float]) -> np.ndarray:
    """Convert VR controller position to robot coordinate system."""
    # VR coordinate system: X=right, Y=up, Z=back (towards user)
    # Robot coordinate system: X=forward, Y=left, Z=up
    # Fixed mapping based on user feedback:
    return np.array([
        -vr_pos['x'] * VR_TO_ROBOT_SCALE,   # VR +Z (back) -> Robot +X (forward)
        vr_pos['z'] * VR_TO_ROBOT_SCALE,  # VR +X (right) -> Robot -Y (right) 
        vr_pos['y'] * VR_TO_ROBOT_SCALE    # VR +Y (up) -> Robot +Z (up)
    ])

def compute_relative_position(current_vr_pos: Dict[str, float], origin_vr_pos: Dict[str, float]) -> np.ndarray:
    """Compute relative position from VR origin to current position."""
    delta_vr = {
        'x': current_vr_pos['x'] - origin_vr_pos['x'],
        'y': current_vr_pos['y'] - origin_vr_pos['y'], 
        'z': current_vr_pos['z'] - origin_vr_pos['z']
    }
    return vr_to_robot_coordinates(delta_vr)

# --- Robot Control ---
def compute_ik(target_position: np.ndarray, target_orientation_quat: Optional[np.ndarray], current_angles: np.ndarray) -> np.ndarray:
    """Compute inverse kinematics for position-only control using 4 joints."""
    if state.physics_client is None or state.robot_id is None:
        return current_angles[:NUM_IK_JOINTS]
    
    # Use current angles to set PyBullet state
    current_angles_rad = np.deg2rad(current_angles)
    for i in range(NUM_JOINTS):
        if state.p_joint_indices[i] is not None:
            p.resetJointState(state.robot_id, state.p_joint_indices[i], current_angles_rad[i])
    
    try:
        # Use only first 4 joints for IK (exclude wrist_roll and gripper)
        ik_lower = np.deg2rad(state.joint_limits_min_deg[:NUM_IK_JOINTS])
        ik_upper = np.deg2rad(state.joint_limits_max_deg[:NUM_IK_JOINTS])
        ik_ranges = ik_upper - ik_lower
        
        # Rest poses for natural 4-DOF arm configuration
        rest_poses = [0.0] * NUM_IK_JOINTS
        rest_poses[0] = 0.0        # shoulder_pan neutral
        rest_poses[1] = math.pi/2  # shoulder_lift pointing forward
        rest_poses[2] = math.pi/4  # elbow_flex slightly bent
        rest_poses[3] = 0.0        # wrist_flex neutral
        
        # Joint damping for the 4 IK joints
        # joint_damping = [
        #     0.1,  # shoulder_pan: moderate
        #     0.1,  # shoulder_lift: moderate  
        #     0.1,  # elbow_flex: moderate
        #     0.1,  # wrist_flex: moderate
        # ]
        
        # Position-only IK (no orientation control)
        ik_solution = p.calculateInverseKinematics(
            bodyUniqueId=state.robot_id,
            endEffectorLinkIndex=state.end_effector_link_index,
            targetPosition=target_position.tolist(),
            lowerLimits=ik_lower.tolist(),
            upperLimits=ik_upper.tolist(),
            jointRanges=ik_ranges.tolist(),
            restPoses=rest_poses,
            # jointDamping=joint_damping,
            solver=0,                                # 0 = DLS (Damped Least Squares)
            maxNumIterations=100,
            residualThreshold=1e-4
        )
        
        return np.rad2deg(ik_solution[:NUM_IK_JOINTS])
        
    except Exception as e:
        logger.warning(f"IK failed: {e}")
        return current_angles[:NUM_IK_JOINTS]

def get_current_ef_position(joint_angles: np.ndarray) -> np.ndarray:
    """Get end effector position from joint angles using forward kinematics."""
    if state.physics_client is None or state.robot_id is None:
        return np.array([0.2, 0.0, 0.15])  # Default position
    
    # Set joint positions
    joint_angles_rad = np.deg2rad(joint_angles)
    for i in range(NUM_JOINTS):
        if state.p_joint_indices[i] is not None:
            p.resetJointState(state.robot_id, state.p_joint_indices[i], joint_angles_rad[i])
    
    # Get end effector position
    link_state = p.getLinkState(state.robot_id, state.end_effector_link_index)
    return np.array(link_state[0])

def should_log_now() -> bool:
    """Check if enough time has passed to log again."""
    current_time = time.time()
    if current_time - state.last_log_time >= state.log_interval:
        state.last_log_time = current_time
        return True
    return False

def send_robot_commands():
    """Send computed joint angles to robot arms."""
    if not state.robot_connected:
        return
    
    current_time = time.time()
    if current_time - state.last_send_time < SEND_INTERVAL:
        return
    
    # Only log if grips are active and enough time has passed
    if (state.left_grip_active or state.right_grip_active) and should_log_now():
        active_arms = []
        if state.left_grip_active:
            active_arms.append("LEFT")
        if state.right_grip_active:
            active_arms.append("RIGHT")
        logger.info(f"🤖 Active control: {', '.join(active_arms)} | Left joints: {state.left_arm_joint_angles.round(1)} | Right joints: {state.right_arm_joint_angles.round(1)}")
    
    try:
        # Try concatenated tensor format (12 joints total: 6 left + 6 right)
        concatenated_angles = np.concatenate([state.left_arm_joint_angles, state.right_arm_joint_angles])
        action_tensor = torch.from_numpy(concatenated_angles).float()
        
        state.robot.send_action(action_tensor)
        state.last_send_time = current_time
        
    except Exception as e:
        logger.error(f"❌ Error sending robot commands: {e}")

# --- VR Controller Processing ---
def process_controller_data(data: Dict):
    """Process incoming VR controller dual-packet data."""
    
    # Handle new dual controller format
    if 'leftController' in data and 'rightController' in data:
        left_data = data['leftController']
        right_data = data['rightController']
        
        # Process left controller
        if left_data.get('position') and left_data.get('gripActive', False):
            process_single_controller('left', left_data)
        elif not left_data.get('gripActive', False) and state.left_grip_active:
            # Handle grip release for left
            handle_grip_release('left')
            
        # Process right controller  
        if right_data.get('position') and right_data.get('gripActive', False):
            process_single_controller('right', right_data)
        elif not right_data.get('gripActive', False) and state.right_grip_active:
            # Handle grip release for right
            handle_grip_release('right')
            
        return
    
    # Handle legacy single controller format (for backward compatibility)
    hand = data.get('hand')
    if data.get('gripReleased'):
        handle_grip_release(hand)
        return
        
    # Process single controller data
    if hand and data.get('position'):
        process_single_controller(hand, data)

def euler_to_quaternion(euler_deg: Dict[str, float]) -> np.ndarray:
    """Convert Euler angles in degrees to quaternion [x, y, z, w]."""
    # Convert degrees to radians
    euler_rad = [math.radians(euler_deg['x']), math.radians(euler_deg['y']), math.radians(euler_deg['z'])]
    
    # Create rotation from Euler angles (XYZ order)
    rotation = R.from_euler('xyz', euler_rad)
    
    # Return quaternion as [x, y, z, w]
    return rotation.as_quat()

def calculate_z_axis_rotation(current_quat: np.ndarray, initial_quat: np.ndarray) -> float:
    """Calculate rotation around the local Z-axis (forward direction) in degrees."""
    try:
        # Create rotation objects
        current_rot = R.from_quat(current_quat)
        initial_rot = R.from_quat(initial_quat)
        
        # Calculate relative rotation (from initial to current)
        relative_rot = current_rot * initial_rot.inv()
        
        # Get the controller's current forward direction (local Z-axis in world space)
        forward_direction = current_rot.apply([0, 0, 1])
        
        # Convert relative rotation to axis-angle representation
        angle_rad = relative_rot.magnitude()
        
        # Handle case where there's no rotation
        if angle_rad < 0.0001:
            return 0.0
        
        # Get the rotation axis
        rotation_axis = relative_rot.as_rotvec() / angle_rad if angle_rad > 0 else np.array([0, 0, 1])
        
        # Project the rotation axis onto the forward direction
        projected_component = np.dot(rotation_axis, forward_direction)
        
        # The rotation around the forward axis is the angle times the projection
        forward_rotation_rad = angle_rad * projected_component
        
        # Convert to degrees
        degrees = math.degrees(forward_rotation_rad)
        
        # Normalize to -180 to +180 range to avoid sudden jumps
        while degrees > 180:
            degrees -= 360
        while degrees < -180:
            degrees += 360
        
        return degrees
        
    except Exception as e:
        logger.warning(f"Error calculating Z-axis rotation: {e}")
        return 0.0

def process_single_controller(hand: str, data: Dict):
    """Process data for a single controller."""
    position = data.get('position', {})
    rotation = data.get('rotation', {})
    grip_active = data.get('gripActive', False)
    
    if hand == 'left' and grip_active:
        state.left_controller_data = data
        
        # Handle grip activation for left arm
        if not state.left_grip_active:
            # Grip just activated - set origin
            state.left_grip_active = True
            state.left_origin_position = position.copy()
            state.left_origin_rotation = euler_to_quaternion(rotation) if rotation else None
            state.left_arm_origin_position = get_current_ef_position(state.left_arm_joint_angles)
            state.left_z_axis_rotation = 0.0
            # Store the current wrist_roll angle as baseline
            state.left_origin_wrist_angle = state.left_arm_joint_angles[WRIST_ROLL_INDEX]
            logger.info(f"🔒 LEFT grip activated - controlling left arm (wrist baseline: {state.left_origin_wrist_angle:.1f}°)")
        
        # Compute target position for left arm
        if state.left_origin_position and state.left_arm_origin_position is not None:
            relative_delta = compute_relative_position(position, state.left_origin_position)
            target_position = state.left_arm_origin_position + relative_delta
            
            # Calculate Z-axis rotation for wrist_roll control
            if state.left_origin_rotation is not None and rotation:
                current_quat = euler_to_quaternion(rotation)
                state.left_z_axis_rotation = calculate_z_axis_rotation(current_quat, state.left_origin_rotation)
                
                # Apply relative Z-axis rotation to the original wrist_roll position
                # This preserves the robot's current wrist position at grip activation
                wrist_roll_angle = state.left_origin_wrist_angle + state.left_z_axis_rotation
                state.left_arm_joint_angles[WRIST_ROLL_INDEX] = np.clip(
                    wrist_roll_angle,
                    state.joint_limits_min_deg[WRIST_ROLL_INDEX],
                    state.joint_limits_max_deg[WRIST_ROLL_INDEX]
                )
            
            # Store goal position for visualization
            state.left_goal_position = target_position.copy()
            
            # Update goal position visualization
            if state.physics_client is not None and 'left_goal' in state.viz_markers:
                p.resetBasePositionAndOrientation(
                    state.viz_markers['left_goal'], 
                    target_position.tolist(), 
                    [0, 0, 0, 1]
                )
            
            # Smooth the position update
            current_ef_pos = get_current_ef_position(state.left_arm_joint_angles)
            smoothed_target = current_ef_pos + POSITION_SMOOTHING * (target_position - current_ef_pos)
            
            # Compute IK for position-only control using 4 joints
            ik_solution = compute_ik(smoothed_target, None, state.left_arm_joint_angles)
            
            # Update first 4 joints with IK solution (wrist_roll controlled separately, gripper unchanged)
            state.left_arm_joint_angles[:NUM_IK_JOINTS] = ik_solution
            
            state.left_arm_joint_angles = np.clip(
                state.left_arm_joint_angles,
                state.joint_limits_min_deg,
                state.joint_limits_max_deg
            )
    
    elif hand == 'right' and grip_active:
        state.right_controller_data = data
        
        # Handle grip activation for right arm
        if not state.right_grip_active:
            # Grip just activated - set origin
            state.right_grip_active = True
            state.right_origin_position = position.copy()
            state.right_origin_rotation = euler_to_quaternion(rotation) if rotation else None
            state.right_arm_origin_position = get_current_ef_position(state.right_arm_joint_angles)
            state.right_z_axis_rotation = 0.0
            # Store the current wrist_roll angle as baseline
            state.right_origin_wrist_angle = state.right_arm_joint_angles[WRIST_ROLL_INDEX]
            logger.info(f"🔒 RIGHT grip activated - controlling right arm (wrist baseline: {state.right_origin_wrist_angle:.1f}°)")
        
        # Compute target position for right arm
        if state.right_origin_position and state.right_arm_origin_position is not None:
            relative_delta = compute_relative_position(position, state.right_origin_position)
            target_position = state.right_arm_origin_position + relative_delta
            
            # Calculate Z-axis rotation for wrist_roll control
            if state.right_origin_rotation is not None and rotation:
                current_quat = euler_to_quaternion(rotation)
                state.right_z_axis_rotation = calculate_z_axis_rotation(current_quat, state.right_origin_rotation)
                
                # Apply relative Z-axis rotation to the original wrist_roll position
                # This preserves the robot's current wrist position at grip activation
                wrist_roll_angle = state.right_origin_wrist_angle + state.right_z_axis_rotation
                state.right_arm_joint_angles[WRIST_ROLL_INDEX] = np.clip(
                    wrist_roll_angle,
                    state.joint_limits_min_deg[WRIST_ROLL_INDEX],
                    state.joint_limits_max_deg[WRIST_ROLL_INDEX]
                )
            
            # Store goal position for visualization
            state.right_goal_position = target_position.copy()
            
            # Update goal position visualization
            if state.physics_client is not None and 'right_goal' in state.viz_markers:
                p.resetBasePositionAndOrientation(
                    state.viz_markers['right_goal'], 
                    target_position.tolist(), 
                    [0, 0, 0, 1]
                )
            
            # Smooth the position update
            current_ef_pos = get_current_ef_position(state.right_arm_joint_angles)
            smoothed_target = current_ef_pos + POSITION_SMOOTHING * (target_position - current_ef_pos)
            
            # Compute IK for position-only control using 4 joints
            ik_solution = compute_ik(smoothed_target, None, state.right_arm_joint_angles)
            
            # Update first 4 joints with IK solution (wrist_roll controlled separately, gripper unchanged)
            state.right_arm_joint_angles[:NUM_IK_JOINTS] = ik_solution
            
            state.right_arm_joint_angles = np.clip(
                state.right_arm_joint_angles,
                state.joint_limits_min_deg,
                state.joint_limits_max_deg
            )

def handle_grip_release(hand: str):
    """Handle grip release for a controller."""
    if hand == 'left':
        if state.left_grip_active:
            state.left_grip_active = False
            state.left_origin_position = None
            state.left_origin_rotation = None
            state.left_z_axis_rotation = 0.0
            state.left_origin_wrist_angle = 0.0
            state.left_arm_origin_position = None
            state.left_goal_position = None
            
            # Hide goal marker
            if state.physics_client is not None and 'left_goal' in state.viz_markers:
                p.resetBasePositionAndOrientation(
                    state.viz_markers['left_goal'], 
                    [0, 0, -1], 
                    [0, 0, 0, 1]
                )
            
            logger.info("🔓 LEFT grip released - control stopped")
    
    elif hand == 'right':
        if state.right_grip_active:
            state.right_grip_active = False
            state.right_origin_position = None
            state.right_origin_rotation = None
            state.right_z_axis_rotation = 0.0
            state.right_origin_wrist_angle = 0.0
            state.right_arm_origin_position = None
            state.right_goal_position = None
            
            # Hide goal marker
            if state.physics_client is not None and 'right_goal' in state.viz_markers:
                p.resetBasePositionAndOrientation(
                    state.viz_markers['right_goal'], 
                    [0, 0, -1], 
                    [0, 0, 0, 1]
                )
            
            logger.info("🔓 RIGHT grip released - control stopped")

# --- WebSocket Handler ---
async def websocket_handler(websocket, path=None):
    """Handle WebSocket connections from VR controllers."""
    client_address = websocket.remote_address
    logger.info(f"VR client connected: {client_address}")
    state.websocket_clients.add(websocket)
    
    try:
        async for message in websocket:
            try:
                data = json.loads(message)
                process_controller_data(data)
            except json.JSONDecodeError:
                logger.warning(f"Received non-JSON message: {message}")
            except Exception as e:
                logger.error(f"Error processing VR data: {e}")
    
    except websockets.exceptions.ConnectionClosedOK:
        logger.info(f"VR client {client_address} disconnected normally")
    except websockets.exceptions.ConnectionClosedError as e:
        logger.warning(f"VR client {client_address} disconnected with error: {e}")
    except Exception as e:
        logger.error(f"Unexpected error with VR client {client_address}: {e}")
    finally:
        state.websocket_clients.discard(websocket)
        # Handle grip releases when client disconnects
        handle_grip_release('left')
        handle_grip_release('right')
        logger.info(f"VR client {client_address} cleanup complete")

# --- Main Control Loop ---
async def control_loop():
    """Main control loop for robot commands and visualization."""
    while True:
        try:
            # Send robot commands
            send_robot_commands()
            
            # Update PyBullet visualization
            if state.physics_client is not None and state.robot_id is not None:
                # Update robot visualization with current joint angles
                for i in range(NUM_JOINTS):
                    if state.p_joint_indices[i] is not None:
                        angle_rad = math.radians(state.left_arm_joint_angles[i])
                        p.resetJointState(state.robot_id, state.p_joint_indices[i], angle_rad)
                
                # Update visualization markers:
                # - Red/Blue spheres: Current end effector positions
                # - Green/Yellow spheres: Goal end effector positions from VR controllers (updated in VR processing)
                if state.left_grip_active and state.left_origin_position:
                    follower_ef_pos = get_current_ef_position(state.left_arm_joint_angles)
                    p.resetBasePositionAndOrientation(
                        state.viz_markers['left_target'], 
                        follower_ef_pos, 
                        [0, 0, 0, 1]
                    )
                    # Update coordinate frames
                    update_coordinate_frame(state.viz_markers['left_target_frame'], follower_ef_pos)
                    
                    if state.left_goal_position is not None:
                        update_coordinate_frame(state.viz_markers['left_goal_frame'], 
                                              state.left_goal_position)
                
                if state.right_grip_active and state.right_origin_position:
                    leader_ef_pos = get_current_ef_position(state.right_arm_joint_angles)
                    p.resetBasePositionAndOrientation(
                        state.viz_markers['right_target'],
                        leader_ef_pos,
                        [0, 0, 0, 1]
                    )
                    # Update coordinate frames
                    update_coordinate_frame(state.viz_markers['right_target_frame'], leader_ef_pos)
                    
                    if state.right_goal_position is not None:
                        update_coordinate_frame(state.viz_markers['right_goal_frame'], 
                                              state.right_goal_position)
                
                p.stepSimulation()
            
            await asyncio.sleep(SEND_INTERVAL)
            
        except Exception as e:
            logger.error(f"Error in control loop: {e}")
            await asyncio.sleep(0.1)

# --- Robot Setup ---
def setup_robot():
    """Initialize and connect to robot."""
    logger.info("🔌 Starting robot setup...")
    
    # Try to connect robot
    try:
        logger.info("🔌 Attempting to connect robot...")
        state.robot = ManipulatorRobot(robot_config)
        state.robot.connect()
        state.robot_connected = True
        logger.info("✅ Robot connected successfully")
    except Exception as e:
        logger.warning(f"❌ Failed to connect to robot: {e}")
        state.robot_connected = False
    
    if not state.robot_connected:
        logger.error("❌ Failed to connect to robot")
        return False
    
    # Read initial joint states from robot
    try:
        observation = state.robot.capture_observation()
        logger.info("📊 Reading initial state from robot")
        
        if observation and "observation.state" in observation:
            initial_state = observation["observation.state"].cpu().numpy()
            logger.info(f"📊 Raw initial joint state shape: {initial_state.shape}, values: {initial_state}")
            
            # Handle concatenated state from dual follower arms
            if len(initial_state) == NUM_JOINTS * 2:  # 12 joints total (6 per arm)
                # Split concatenated state into left and right arm states
                state.left_arm_joint_angles = initial_state[:NUM_JOINTS].copy()
                state.right_arm_joint_angles = initial_state[NUM_JOINTS:].copy()
                logger.info(f"📊 Split state - Left arm: {state.left_arm_joint_angles}")
                logger.info(f"📊 Split state - Right arm: {state.right_arm_joint_angles}")
            elif len(initial_state) == NUM_JOINTS:  # 6 joints (single arm or fallback)
                # Use same state for both arms
                state.left_arm_joint_angles = initial_state.copy()
                state.right_arm_joint_angles = initial_state.copy()
                logger.info(f"📊 Single arm state used for both arms: {initial_state}")
            else:
                logger.warning(f"⚠️ Unexpected state length {len(initial_state)}, using defaults")
                state.left_arm_joint_angles = np.zeros(NUM_JOINTS)
                state.right_arm_joint_angles = np.zeros(NUM_JOINTS)
        else:
            logger.warning("⚠️ Could not read initial robot state, using defaults")
            state.left_arm_joint_angles = np.zeros(NUM_JOINTS)
            state.right_arm_joint_angles = np.zeros(NUM_JOINTS)
        
        logger.info(f"🎉 Robot setup complete! Connected robot: {state.robot_connected}")
        return True
        
    except Exception as e:
        logger.error(f"❌ Error during robot setup: {e}")
        import traceback
        traceback.print_exc()
        return False

# --- Shutdown ---
def shutdown():
    """Clean shutdown of all systems."""
    logger.info("Shutting down...")
    
    # Move robot back to initial positions before disconnecting
    if state.robot_connected and state.robot:
        try:
            logger.info("🏠 Moving robot arms back to initial positions...")
            
            # Use the specific initial positions you provided
            initial_left_arm = np.array([1.7578125, 185.09766, 175.3418, 72.1582, 3.6914062, 0.3358522])
            initial_right_arm = np.array([-9.316406, 193.27148, 181.05469, 72.77344, -4.0429688, 0.8780488])
            
            # Create action tensor with initial joint angles for both arms
            initial_action = np.concatenate([initial_left_arm, initial_right_arm])
            action_tensor = torch.from_numpy(initial_action).float()
            
            # Send robot to initial position
            state.robot.send_action(action_tensor)
            logger.info(f"🏠 Commanded return to initial positions:")
            logger.info(f"   Left arm: {initial_left_arm.round(1)}")
            logger.info(f"   Right arm: {initial_right_arm.round(1)}")
            
            # Give the robot time to move to initial position
            logger.info("⏳ Waiting 1 second for robot to reach initial position...")
            time.sleep(1.0)
            
        except Exception as e:
            logger.error(f"❌ Error moving robot to initial position: {e}")
    
    # Disconnect robot
    if state.robot_connected and state.robot:
        try:
            # Disable torque on follower arms
            if hasattr(state.robot, 'follower_arms'):
                for arm_name, arm in state.robot.follower_arms.items():
                    try:
                        arm.write("Torque_Enable", 0)
                        logger.info(f"Disabled torque on follower arm: {arm_name}")
                    except Exception as e:
                        logger.warning(f"Could not disable torque on follower arm {arm_name}: {e}")
            
            state.robot.disconnect()
            logger.info("Robot disconnected")
        except Exception as e:
            logger.error(f"Error disconnecting robot: {e}")
    
    # Disconnect PyBullet
    if state.physics_client is not None and p.isConnected(state.physics_client):
        p.disconnect(state.physics_client)
        logger.info("PyBullet disconnected")

# --- Main ---
async def main():
    """Main entry point."""
    logger.info("Starting VR Robot Teleoperation System...")
    
    # Setup SSL
    ssl_context = setup_ssl()
    if ssl_context is None:
        logger.error("SSL setup failed")
        return
    
    # Setup PyBullet
    if not setup_pybullet():
        logger.error("PyBullet setup failed")
        return
    
    # Setup Robot
    if not setup_robot():
        logger.error("Robot setup failed, continuing with visualization only")
    
    # Start WebSocket server
    host = "0.0.0.0"  # Listen on all interfaces
    port = 8442
    logger.info(f"Starting WebSocket server on wss://{host}:{port}")
    
    try:
        # Start WebSocket server and control loop concurrently
        server = await websockets.serve(websocket_handler, host, port, ssl=ssl_context)
        control_task = asyncio.create_task(control_loop())
        
        logger.info("VR Robot Teleoperation System ready!")
        logger.info("Connect your VR headset and use grip buttons to control the robot arms:")
        logger.info("  - Left controller grip: Controls left follower arm (4-DOF position-only)")
        logger.info("  - Right controller grip: Controls right follower arm (4-DOF position-only)")
        logger.info("  - IK controls: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex")
        logger.info("  - Target: Fixed_Jaw_tip for precise end-effector positioning")
        
        # Run forever
        await asyncio.gather(
            server.wait_closed(),
            control_task
        )
        
    except KeyboardInterrupt:
        logger.info("Ctrl+C detected, shutting down...")
    except Exception as e:
        logger.error(f"Main loop error: {e}")
    finally:
        shutdown()

def update_coordinate_frame(frame_lines: list, position: np.ndarray, orientation_quat: Optional[np.ndarray] = None):
    """Update coordinate frame visualization lines for position-only display."""
    if state.physics_client is None or not frame_lines:
        return
    
    axis_length = 0.05
    
    # For position-only control, just show a simple coordinate frame aligned with world axes
    # Default to identity rotation (world-aligned frame)
    if orientation_quat is None:
        orientation_quat = [0, 0, 0, 1]  # Identity quaternion
    
    # Convert quaternion to rotation matrix
    r = R.from_quat(orientation_quat)
    rotation_matrix = r.as_matrix()
    
    # Update each axis line (X=red, Y=green, Z=blue)
    for i in range(3):
        axis_vector = rotation_matrix[:, i] * axis_length
        end_point = position + axis_vector
        
        p.addUserDebugLine(
            position.tolist(), 
            end_point.tolist(), 
            lineColorRGB=[[1, 0, 0], [0, 1, 0], [0, 0, 1]][i], 
            lineWidth=3,
            replaceItemUniqueId=frame_lines[i]
        )

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nShutdown complete.") 