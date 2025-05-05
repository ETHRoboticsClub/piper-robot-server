import time
import numpy as np
import torch
import math
from pynput import keyboard
import pybullet as p
import pybullet_data
import json
import os
import sys

# TODO: Import or copy IK/FK functions
# from forward_kinematics import forward_kinematics, get_transform # Assuming FK returns T matrices now
# from inverse_kinematics import iterative_ik, load_joint_limits # Assuming limits loading is separate

from lerobot.common.robot_devices.robots.manipulator import ManipulatorRobot
from lerobot.common.robot_devices.robots.configs import So100RobotConfig

# --- Constants ---
# Define control steps (adjust sensitivity as needed)
POS_STEP = 0.01  # meters
ANGLE_STEP = 5.0 # degrees
GRIPPER_STEP = 10.0 # degrees (or adjust based on gripper range)

# Joint names order expected by So100RobotConfig/ManipulatorRobot
# Check lerobot/common/robot_devices/robots/configs.py if unsure
JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
NUM_JOINTS = len(JOINT_NAMES)
NUM_IK_JOINTS = 4 # Number of joints controlled by IK

# --- Robot Configuration ---
# You might need to adjust the port names if they are different on your system.
robot_config = So100RobotConfig(
    # Example: Override default ports if needed
    # leader_arms={"main": FeetechMotorsBusConfig(port="/dev/ttyYOUR_LEADER_PORT", ...)},
    # follower_arms={"main": FeetechMotorsBusConfig(port="/dev/ttyYOUR_FOLLOWER_PORT", ...)},
    # Disable cameras if not needed
    cameras={}
)

# Instantiate the robot object
robot = ManipulatorRobot(robot_config)

# --- Global State Variables ---
# Target state
target_ef_position = np.array([0.2, 0.0, 0.15]) # Initial target position
target_pitch_deg = 0.0                          # Initial target pitch
target_wrist_roll_deg = 0.0                     # Initial target wrist roll
target_gripper_deg = 0.0                        # Initial target gripper angle (0=closed?, adjust based on range)

# Current actual state (will be updated from robot observation)
current_joint_angles_deg = np.zeros(NUM_JOINTS)
current_ef_position = np.array([0.0, 0.0, 0.0])
current_ef_rpy_deg = np.array([0.0, 0.0, 0.0])

# Keyboard control flags
control_active = True
delta_pos = np.zeros(3) # [dx, dy, dz]
delta_pitch = 0.0
delta_wrist_roll = 0.0
delta_gripper = 0.0

# PyBullet visualization IDs
physicsClient = None
robotId = None
p_joint_indices = {} # Map URDF joint name to pybullet index
viz_markers = {}
debug_line_ids = {}

# Joint limits (loaded from inverse_kinematics or defined here)
joint_limits_deg = None

# TODO: Add IK/FK functions here or import them
# TODO: Add PyBullet setup function
# TODO: Add Keyboard listener functions (on_press, on_release)
# TODO: Add Main control loop
# TODO: Add Cleanup logic (disconnect, torque off)

# --- Forward Kinematics Functions (Copied from forward_kinematics.py) ---
def R_x(angle_deg):
    angle = np.deg2rad(angle_deg)
    return np.array([
        [1, 0, 0],
        [0, np.cos(angle), -np.sin(angle)],
        [0, np.sin(angle), np.cos(angle)]
    ])

def R_y(angle_deg):
    angle = np.deg2rad(angle_deg)
    return np.array([
        [np.cos(angle), 0, np.sin(angle)],
        [0, 1, 0],
        [-np.sin(angle), 0, np.cos(angle)]
    ])

def R_z(theta):
    # Note: Takes radians, unlike R_x/R_y
    return np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta), np.cos(theta), 0],
        [0, 0, 1]
    ])

def T(R, px, py, pz):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [px, py, pz]
    return T

def forward_kinematics(theta1, theta2, theta3, theta4, return_intermediate_frames=False):
    # Link lengths and heights (Using the hardcoded ones from forward_kinematics.py)
    l0, l1, l2, l3, l4 = 0.03, 0.11, 0.134, 0.07, 0.1
    h0, h1, h2 = 0.052, 0.03, -0.005

    theta1_rad = np.deg2rad(theta1)
    theta2_rad = np.deg2rad(theta2)
    theta3_rad = np.deg2rad(theta3)
    theta4_rad = np.deg2rad(theta4)

    # --- Calculate Individual Transforms (Based on the version in forward_kinematics.py) ---
    # 0 to 1
    R_base_to_joint_static = R_z(1.5708) @ R_y(0) @ R_x(1.5708) # Using rad inputs directly
    R_joint_rotation = R_y(theta1) # R_y takes degrees
    correction_angle_deg_01 = -20.0
    R_correction_01 = R_y(correction_angle_deg_01)
    R01 = R_base_to_joint_static @ R_joint_rotation @ R_correction_01
    p01 = np.array([0, -0.0452, 0.0181])
    T01 = T(R01, p01[0], p01[1], p01[2])

    # 1 to 2
    R_frame1_to_joint2_static = R_z(-1.5708) @ R_y(0) @ R_x(3.1416) # Using rad inputs
    R_joint2_rotation = R_z(theta2_rad) # R_z takes radians
    correction_angle_deg_12 = -15.0
    R_correction_12 = R_z(np.deg2rad(correction_angle_deg_12)) # Correct around Z
    R12 = R_frame1_to_joint2_static @ R_joint2_rotation @ R_correction_12
    p12 = np.array([0.000125, 0.1086, 0])
    T12 = T(R12, p12[0], p12[1], p12[2])

    # 2 to 3
    R_frame2_to_joint3_static = R_z(-2.2391) @ R_y(0) @ R_x(0) # Using rad inputs
    R_joint3_rotation = R_z(theta3_rad) # R_z takes radians
    R23 = R_frame2_to_joint3_static @ R_joint3_rotation
    p23 = np.array([-0.11238, 0.0282, 0])
    T23 = T(R23, p23[0], p23[1], p23[2])

    # 3 to 4
    R_frame3_to_joint4_static = R_z(0) @ R_y(1.5708) @ R_x(0.90254) # Using rad inputs
    R_joint4_rotation = R_x(theta4) # R_x takes degrees
    R34 = R_frame3_to_joint4_static @ R_joint4_rotation
    p34 = np.array([-0.1102, 0.005375, 0])
    T34 = T(R34, p34[0], p34[1], p34[2])

    # 4 to 5 (End Effector)
    R45 = np.eye(3)
    T45 = T(R45, -0.04, 0, -0.12) # Offset -4cm X, -12cm Z relative to Frame 4

    # Cumulative Transforms
    T02 = T01 @ T12
    T03 = T02 @ T23
    T04 = T03 @ T34
    T05 = T04 @ T45

    # Extract Position and RPY
    position = T05[:3, 3]
    R = T05[:3, :3]
    roll = np.arctan2(R[2, 1], R[2, 2])
    pitch = np.arctan2(-R[2, 0], np.sqrt(R[2, 1]**2 + R[2, 2]**2))
    yaw = np.arctan2(R[1, 0], R[0, 0])
    rpy = np.rad2deg(np.array([roll, pitch, yaw]))

    if return_intermediate_frames:
        return position, rpy, T01, T02, T03, T04
    else:
        return position, rpy

# --- Inverse Kinematics Functions (Copied from inverse_kinematics.py) ---
LIMITS_FILE = "limits.json"

def load_joint_limits():
    global joint_limits_deg
    if os.path.exists(LIMITS_FILE):
        try:
            with open(LIMITS_FILE, 'r') as f:
                limits_data = json.load(f)
                if len(limits_data) >= NUM_IK_JOINTS:
                    min_limits = [entry['min_degrees'] for entry in limits_data[:NUM_IK_JOINTS]]
                    max_limits = [entry['max_degrees'] for entry in limits_data[:NUM_IK_JOINTS]]
                    joint_limits_deg = (np.array(min_limits), np.array(max_limits))
                    print(f"Loaded joint limits (degrees {NUM_IK_JOINTS} joints) from {LIMITS_FILE}:")
                    print(f"  Min: {joint_limits_deg[0]}")
                    print(f"  Max: {joint_limits_deg[1]}")
                else:
                    print(f"Warning: {LIMITS_FILE} does not contain enough entries for {NUM_IK_JOINTS} joints.", file=sys.stderr)
                    joint_limits_deg = None # Ensure it's None if loading fails
        except (IOError, json.JSONDecodeError, KeyError) as e:
            print(f"Warning: Could not load or parse {LIMITS_FILE}. Error: {e}", file=sys.stderr)
            joint_limits_deg = None # Ensure it's None if loading fails
    else:
        print(f"Warning: Limits file {LIMITS_FILE} not found. IK will operate without joint limits.", file=sys.stderr)
        joint_limits_deg = None # Ensure it's None if loading fails

def numeric_jacobian(fk_func, angles, eps=1e-6):
    J = np.zeros((4, len(angles))) # 4 (x,y,z,pitch) x N joints
    for i in range(len(angles)):
        angles_fwd = angles.copy()
        angles_bwd = angles.copy()
        angles_fwd[i] += eps
        angles_bwd[i] -= eps

        pos_fwd, rpy_fwd = fk_func(*angles_fwd)
        pos_bwd, rpy_bwd = fk_func(*angles_bwd)

        J[0:3, i] = (pos_fwd - pos_bwd) / (2*eps) # Position derivative

        # Pitch derivative (handle wrap around)
        pitch_diff = (rpy_fwd[1] - rpy_bwd[1])
        pitch_diff = (pitch_diff + 180) % 360 - 180
        J[3, i] = pitch_diff / (2*eps)
    return J

def iterative_ik(target_pos, target_pitch, initial_guess=np.zeros(NUM_IK_JOINTS),
                 max_iter=100, tol=1e-3, alpha=0.5):
    global joint_limits_deg # Use the globally loaded limits
    angles = np.array(initial_guess[:NUM_IK_JOINTS], dtype=float) # Ensure we use only NUM_IK_JOINTS

    min_angles = None
    max_angles = None
    if joint_limits_deg is not None:
        min_angles, max_angles = joint_limits_deg
        if len(min_angles) != NUM_IK_JOINTS or len(max_angles) != NUM_IK_JOINTS:
             print(f"Warning: Loaded limits dimensions ({len(min_angles)}/{len(max_angles)}) != NUM_IK_JOINTS ({NUM_IK_JOINTS}). Ignoring limits.", file=sys.stderr)
             min_angles, max_angles = None, None # Ignore inconsistent limits

    for iter_count in range(max_iter):
        current_pos, current_rpy = forward_kinematics(*angles)

        pos_error = target_pos - current_pos
        pitch_error = target_pitch - current_rpy[1]
        pitch_error = (pitch_error + 180) % 360 - 180 # Handle wrap around
        error = np.hstack((pos_error, pitch_error))

        if np.linalg.norm(error) < tol:
            break

        J = numeric_jacobian(forward_kinematics, angles)
        lambda_damping = 0.03
        try:
            J_pinv = J.T @ np.linalg.inv(J @ J.T + lambda_damping**2 * np.eye(J.shape[0]))
        except np.linalg.LinAlgError:
            J_pinv = np.linalg.pinv(J)

        delta_angles = alpha * (J_pinv @ error)
        angles += delta_angles

        if min_angles is not None and max_angles is not None:
            angles = np.clip(angles, min_angles, max_angles)

    # No explicit 'convergence failed' print to reduce noise
    return angles

# --- PyBullet Setup (Adapted from keyboard_teleop.py) ---
def setup_pybullet():
    global physicsClient, robotId, p_joint_indices, viz_markers, debug_line_ids
    try:
        physicsClient = p.connect(p.GUI)
    except p.error as e:
        print(f"Could not connect to PyBullet GUI, attempting DIRECT connection: {e}")
        try:
            physicsClient = p.connect(p.DIRECT) # Fallback for headless
        except p.error as direct_e:
            print(f"Failed to connect to PyBullet DIRECT mode: {direct_e}")
            print("PyBullet visualization will be disabled.")
            physicsClient = -1 # Indicate connection failure
            return False # Signal failure

    if physicsClient < 0: return False # Early exit if connection failed

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0,0,-9.81)
    p.loadURDF("plane.urdf")

    # Load the robot URDF (Ensure path is correct relative to workspace root)
    urdf_path = "URDF/SO_5DOF_ARM100_05d.SLDASM/urdf/SO_5DOF_ARM100_05d.SLDASM.urdf"
    if not os.path.exists(urdf_path):
        print(f"ERROR: URDF file not found at {urdf_path}", file=sys.stderr)
        p.disconnect(physicsClient)
        physicsClient = -1
        return False # Signal failure

    robot_start_pos = [0, 0, 0]
    robot_start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    robotId = p.loadURDF(urdf_path, robot_start_pos, robot_start_orientation, useFixedBase=1)

    # Map JOINT_NAMES to pybullet joint indices
    num_p_joints = p.getNumJoints(robotId)
    p_joint_indices = {}
    # Need to map URDF joint names (from pybullet) to our internal JOINT_NAMES order
    # This requires knowing the URDF joint names. Let's assume they match the keys
    # used in the old script's `leader_motors` dict for now, but this might need adjustment.
    # URDF names likely: "Shoulder_Rotation", "Shoulder_Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll", "Gripper"
    # Our JOINT_NAMES: ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
    # --> We need a mapping! <--
    urdf_to_internal_name_map = {
        "Shoulder_Rotation": "shoulder_pan",
        "Shoulder_Pitch": "shoulder_lift",
        "Elbow": "elbow_flex",
        "Wrist_Pitch": "wrist_flex",
        "Wrist_Roll": "wrist_roll",
        "Gripper": "gripper",
        # Add other potential mappings if URDF names differ significantly
    }

    p_name_to_index = {}
    for i in range(num_p_joints):
        info = p.getJointInfo(robotId, i)
        joint_name_bytes = info[1]
        joint_name_str = joint_name_bytes.decode('UTF-8')
        p_name_to_index[joint_name_str] = i
        # Disable default velocity control
        p.setJointMotorControl2(robotId, i, p.VELOCITY_CONTROL, force=0)

    # Create the final mapping based on our JOINT_NAMES order
    p_joint_indices_list = [None] * NUM_JOINTS
    missing_joints = []
    for urdf_name, internal_name in urdf_to_internal_name_map.items():
        if urdf_name in p_name_to_index:
            try:
                target_idx = JOINT_NAMES.index(internal_name)
                p_joint_indices_list[target_idx] = p_name_to_index[urdf_name]
            except ValueError:
                print(f"Warning: Internal name '{internal_name}' from map not in JOINT_NAMES list.", file=sys.stderr)
        else:
            missing_joints.append(urdf_name)

    if None in p_joint_indices_list:
        print("ERROR: Could not map all required joints to PyBullet indices:", file=sys.stderr)
        for i, name in enumerate(JOINT_NAMES):
             if p_joint_indices_list[i] is None:
                 print(f"  - '{name}' not found or mapped in URDF.", file=sys.stderr)
        p.disconnect(physicsClient)
        physicsClient = -1
        return False # Signal failure

    if missing_joints:
         print(f"Warning: Some URDF joints in the map were not found in the loaded URDF: {missing_joints}", file=sys.stderr)

    p_joint_indices = p_joint_indices_list # Store the ordered list of indices

    print(f"PyBullet Joint Indices (matched to JOINT_NAMES order {JOINT_NAMES}): {p_joint_indices}")

    # Create Visual Markers
    markers = {}
    target_shape = p.createVisualShape(p.GEOM_SPHERE, radius=0.02, rgbaColor=[1, 0, 0, 0.8])
    markers['target'] = p.createMultiBody(baseVisualShapeIndex=target_shape, basePosition=[0,0,1])
    actual_shape = p.createVisualShape(p.GEOM_SPHERE, radius=0.02, rgbaColor=[0, 0, 1, 0.8])
    markers['actual_ef'] = p.createMultiBody(baseVisualShapeIndex=actual_shape, basePosition=[0,0,1])
    viz_markers = markers # Store globally

    # Initialize debug line IDs
    debug_line_ids = {k: -1 for k in ['x_target','y_target','z_target', 'x_actual','y_actual','z_actual']}

    print(f"PyBullet setup complete. Robot ID: {robotId}")
    return True # Signal success

def update_pybullet_visuals(current_angles_deg, target_pos, actual_pos):
    """Updates the robot pose and markers in PyBullet."""
    global robotId, p_joint_indices, viz_markers, debug_line_ids
    if physicsClient is None or physicsClient < 0 or robotId is None:
        return # PyBullet not active or failed

    # Update robot joint positions
    current_angles_rad = np.deg2rad(current_angles_deg)
    for i in range(NUM_JOINTS):
        if p_joint_indices[i] is not None: # Check if joint was successfully mapped
            p.resetJointState(robotId, p_joint_indices[i], targetValue=current_angles_rad[i])

    # Update marker positions
    if 'target' in viz_markers:
        p.resetBasePositionAndOrientation(viz_markers['target'], target_pos, [0,0,0,1])
    if 'actual_ef' in viz_markers:
        p.resetBasePositionAndOrientation(viz_markers['actual_ef'], actual_pos, [0,0,0,1])

    # Update debug lines (coordinate frames)
    # Target Frame (Red)
    origin = target_pos
    x_end = [origin[0] + 0.05, origin[1], origin[2]]
    y_end = [origin[0], origin[1] + 0.05, origin[2]]
    z_end = [origin[0], origin[1], origin[2] + 0.05]
    if debug_line_ids['x_target'] >= 0: p.addUserDebugLine(origin, x_end, [1,0,0], 1.5, replaceItemUniqueId=debug_line_ids['x_target'])
    else: debug_line_ids['x_target'] = p.addUserDebugLine(origin, x_end, [1,0,0], 1.5)
    if debug_line_ids['y_target'] >= 0: p.addUserDebugLine(origin, y_end, [0,1,0], 1.5, replaceItemUniqueId=debug_line_ids['y_target'])
    else: debug_line_ids['y_target'] = p.addUserDebugLine(origin, y_end, [0,1,0], 1.5)
    if debug_line_ids['z_target'] >= 0: p.addUserDebugLine(origin, z_end, [0,0,1], 1.5, replaceItemUniqueId=debug_line_ids['z_target'])
    else: debug_line_ids['z_target'] = p.addUserDebugLine(origin, z_end, [0,0,1], 1.5)

    # Actual EF Frame (Blue) - Use FK based on current angles for this?
    # For simplicity, let's just show position for now. Frame drawing requires orientation.
    origin = actual_pos
    x_end = [origin[0] + 0.05, origin[1], origin[2]]
    y_end = [origin[0], origin[1] + 0.05, origin[2]]
    z_end = [origin[0], origin[1], origin[2] + 0.05]
    if debug_line_ids['x_actual'] >= 0: p.addUserDebugLine(origin, x_end, [1,0,1], 1.5, replaceItemUniqueId=debug_line_ids['x_actual']) # Magenta X
    else: debug_line_ids['x_actual'] = p.addUserDebugLine(origin, x_end, [1,0,1], 1.5)
    if debug_line_ids['y_actual'] >= 0: p.addUserDebugLine(origin, y_end, [0,1,1], 1.5, replaceItemUniqueId=debug_line_ids['y_actual']) # Cyan Y
    else: debug_line_ids['y_actual'] = p.addUserDebugLine(origin, y_end, [0,1,1], 1.5)
    if debug_line_ids['z_actual'] >= 0: p.addUserDebugLine(origin, z_end, [1,1,0], 1.5, replaceItemUniqueId=debug_line_ids['z_actual']) # Yellow Z
    else: debug_line_ids['z_actual'] = p.addUserDebugLine(origin, z_end, [1,1,0], 1.5)

    p.stepSimulation() # Keep simulation advancing


# --- Keyboard Control (Adapted from keyboard_teleop.py) ---
listener = None # Global listener object

def on_press(key):
    global control_active, delta_pos, delta_pitch, delta_wrist_roll, delta_gripper
    try:
        # Position Control (World Frame)
        if key.char == 'w': delta_pos[0] = POS_STEP   # Forward (+X)
        elif key.char == 's': delta_pos[0] = -POS_STEP  # Backward (-X)
        elif key.char == 'a': delta_pos[1] = POS_STEP   # Left (+Y)
        elif key.char == 'd': delta_pos[1] = -POS_STEP  # Right (-Y)
        elif key.char == 'q': delta_pos[2] = POS_STEP   # Up (+Z)
        elif key.char == 'e': delta_pos[2] = -POS_STEP  # Down (-Z)

        # Gripper Control
        elif key.char == '+': delta_gripper = GRIPPER_STEP # Open/Increase Angle
        elif key.char == '-': delta_gripper = -GRIPPER_STEP # Close/Decrease Angle

    except AttributeError:
        # Special keys (Arrows for Pitch/Roll)
        if key == keyboard.Key.up: delta_pitch = ANGLE_STEP        # Pitch Up
        elif key == keyboard.Key.down: delta_pitch = -ANGLE_STEP   # Pitch Down
        elif key == keyboard.Key.left: delta_wrist_roll = -ANGLE_STEP # Wrist Roll CCW
        elif key == keyboard.Key.right: delta_wrist_roll = ANGLE_STEP # Wrist Roll CW
        # Exit control loop
        elif key == keyboard.Key.esc:
             print("ESC pressed. Stopping control loop.")
             control_active = False
             return False # Stop the listener

def on_release(key):
    global control_active, delta_pos, delta_pitch, delta_wrist_roll, delta_gripper
    try:
        # Reset delta on key release
        if key.char in ('w', 's'): delta_pos[0] = 0
        elif key.char in ('a', 'd'): delta_pos[1] = 0
        elif key.char in ('q', 'e'): delta_pos[2] = 0
        elif key.char in ('+', '-'): delta_gripper = 0
    except AttributeError:
        if key in (keyboard.Key.up, keyboard.Key.down): delta_pitch = 0
        elif key in (keyboard.Key.left, keyboard.Key.right): delta_wrist_roll = 0

def start_keyboard_listener():
    global listener
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()
    print("\nKeyboard Teleoperation Enabled:")
    print("  W/S: Move Target X (+/-)")
    print("  A/D: Move Target Y (+/-)")
    print("  Q/E: Move Target Z (+/-)")
    print("  Up/Down: Adjust Target Pitch (+/-)")
    print("  Left/Right: Adjust Wrist Roll (+/-)")
    print("  +/-: Adjust Gripper Angle (+/-)")
    print("  ESC: Exit")

# TODO: Add Main control loop
# TODO: Add Cleanup logic (disconnect, torque off) - Already added in initial structure

if __name__ == "__main__":
    pybullet_enabled = False
    try:
        print("Connecting to the robot...")
        robot.connect()
        print("Robot connected successfully.")

        # Load joint limits for IK
        load_joint_limits()

        # Initialize PyBullet
        print("Initializing PyBullet...")
        pybullet_enabled = setup_pybullet()
        if not pybullet_enabled:
            print("Continuing without PyBullet visualization.")

        # Get initial robot state
        print("Reading initial robot state...")
        observation = robot.capture_observation()
        # Ensure the key matches how ManipulatorRobot structures observations
        # It's often 'observation.state' for the concatenated tensor
        if "observation.state" in observation:
            initial_joint_angles_deg_tensor = observation["observation.state"]
            # Ensure it's a CPU numpy array for calculations
            current_joint_angles_deg = initial_joint_angles_deg_tensor.cpu().numpy()
            print(f"Initial joint angles (degrees): {current_joint_angles_deg}")
            # Use the first NUM_IK_JOINTS angles for FK
            current_ef_position, current_ef_rpy_deg = forward_kinematics(*current_joint_angles_deg[:NUM_IK_JOINTS])
            print(f"Initial FK EF Position: {current_ef_position}")
            print(f"Initial FK EF RPY (deg): {current_ef_rpy_deg}")
            # Set initial target position to current FK position
            target_ef_position = current_ef_position.copy()
            target_pitch_deg = current_ef_rpy_deg[1]
            target_wrist_roll_deg = current_joint_angles_deg[4] # Index 4 is wrist_roll
            target_gripper_deg = current_joint_angles_deg[5]    # Index 5 is gripper
        else:
            print("Warning: 'observation.state' not found in initial observation. Using default state values.", file=sys.stderr)
            # Keep the default initial target values
            current_joint_angles_deg = np.zeros(NUM_JOINTS) # Fallback
            current_ef_position, current_ef_rpy_deg = forward_kinematics(*current_joint_angles_deg[:NUM_IK_JOINTS]) # FK from zeros

        # Update PyBullet with initial state
        if pybullet_enabled:
            update_pybullet_visuals(current_joint_angles_deg, target_ef_position, current_ef_position)
            # Set initial marker positions accurately now that we have FK result
            p.resetBasePositionAndOrientation(viz_markers['target'], target_ef_position, [0,0,0,1])
            p.resetBasePositionAndOrientation(viz_markers['actual_ef'], current_ef_position, [0,0,0,1])

        # Start keyboard listener
        start_keyboard_listener()

        # --- Main Control Loop ---
        print("\nEntering main control loop...")
        last_send_time = time.time()
        send_interval = 0.05 # Send commands roughly every 50ms (20Hz)

        while control_active:
            loop_start_time = time.time()

            # 1. Update Target State based on Keyboard Input
            target_ef_position += delta_pos * (loop_start_time - last_send_time) # Scale movement by time
            target_pitch_deg += delta_pitch * (loop_start_time - last_send_time)
            target_wrist_roll_deg += delta_wrist_roll * (loop_start_time - last_send_time)
            target_gripper_deg += delta_gripper * (loop_start_time - last_send_time)
            # TODO: Clamp target angles (wrist roll, gripper) to reasonable limits if known

            # 2. Get Current Robot State (Optional, for updating visualization)
            # Avoid reading too frequently if it slows things down
            # observation = robot.capture_observation()
            # if "observation.state" in observation:
            #     current_joint_angles_deg = observation["observation.state"].cpu().numpy()
            #     # Optional: Recalculate current EF pos/rpy for viz marker
            #     current_ef_position, current_ef_rpy_deg = forward_kinematics(*current_joint_angles_deg[:NUM_IK_JOINTS])

            # 3. Calculate IK for the first NUM_IK_JOINTS
            # Use current angles as initial guess for smoothness
            ik_solution_angles = iterative_ik(
                target_pos=target_ef_position,
                target_pitch=target_pitch_deg,
                initial_guess=current_joint_angles_deg[:NUM_IK_JOINTS]
            )

            # 4. Construct Full Action Command (Angles in Degrees)
            # Combine IK solution with direct keyboard control for wrist/gripper
            action_angles_deg = np.zeros(NUM_JOINTS)
            action_angles_deg[:NUM_IK_JOINTS] = ik_solution_angles
            action_angles_deg[4] = target_wrist_roll_deg # Wrist roll (index 4)
            action_angles_deg[5] = target_gripper_deg   # Gripper (index 5)

            # Update current angles based on command for smoother IK/Viz next iteration
            # (This assumes the command is instantly achieved, which isn't true,
            # but is better than using stale observation data if reads are slow)
            current_joint_angles_deg = action_angles_deg.copy()
            # Update current EF pos based on the *commanded* angles for viz
            current_ef_position, current_ef_rpy_deg = forward_kinematics(*current_joint_angles_deg[:NUM_IK_JOINTS])


            # 5. Send Action to Robot (if enough time has passed)
            current_time = time.time()
            if current_time - last_send_time >= send_interval:
                action_tensor = torch.from_numpy(action_angles_deg).float()
                try:
                    # print(f"Sending action: {action_tensor.numpy()}") # Debug print
                    robot.send_action(action_tensor)
                    last_send_time = current_time
                except Exception as send_err:
                    print(f"Error sending action: {send_err}", file=sys.stderr)
                    # Maybe break or add error handling

            # 6. Update PyBullet Visualization
            if pybullet_enabled:
                update_pybullet_visuals(current_joint_angles_deg, target_ef_position, current_ef_position)

            # 7. Loop Rate Control
            elapsed_time = time.time() - loop_start_time
            sleep_time = max(0, send_interval - elapsed_time) # Maintain roughly send_interval Hz loop rate
            time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\nCtrl+C detected. Exiting.")
        control_active = False # Ensure loop terminates
    except Exception as e:
        print(f"\nAn unhandled error occurred in the main loop: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # --- Cleanup ---
        print("\nInitiating shutdown sequence...")
        # Stop keyboard listener if running
        if listener and listener.is_alive():
            print("Stopping keyboard listener...")
            listener.stop()
            # listener.join() # Optional: wait for listener thread

        # Disable torque and disconnect robot
        try:
            if robot.is_connected:
                print("Disabling torque on follower motors...")
                # Iterate through follower arms and attempt to disable torque directly
                for arm_name, follower_arm_instance in robot.follower_arms.items():
                    try:
                        # Directly attempt to write 0 to disable torque
                        follower_arm_instance.write("Torque_Enable", 0)
                        print(f"Torque disabled for arm: {arm_name}")
                    except Exception as torque_err:
                        print(f"Could not disable torque for arm {arm_name}: {torque_err}")
        except AttributeError:
            print("Robot object state prevents torque disable check (e.g., connection failed).")
        except Exception as final_err:
            print(f"An error occurred during torque disable: {final_err}")

        # Disconnect robot
        if robot and hasattr(robot, 'disconnect') and callable(getattr(robot, 'disconnect')):
             print("Disconnecting robot...")
             robot.disconnect()
             print("Robot disconnected.")
        else:
            print("Robot object not available or disconnect method missing.")
            
        # Disconnect PyBullet
        if physicsClient is not None and p.isConnected(physicsClient):
             print("Disconnecting PyBullet...")
             p.disconnect(physicsClient)
             print("PyBullet disconnected.")

        print("Shutdown complete.") 