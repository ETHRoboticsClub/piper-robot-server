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
from lerobot.common.robot_devices.motors.configs import FeetechMotorsBusConfig
from lerobot.common.robot_devices.utils import RobotDeviceNotConnectedError 


# --- Constants ---
# Define control steps (adjust sensitivity as needed)
POS_STEP = 0.01  # meters
ANGLE_STEP = 5.0 # degrees
GRIPPER_STEP = 10.0 # degrees (or adjust based on gripper range)
common_motors = {
    # name: (index, model) - Standard SO100
    "shoulder_pan": [1, "sts3215"],
    "shoulder_lift": [2, "sts3215"],
    "elbow_flex": [3, "sts3215"],
    "wrist_flex": [4, "sts3215"],
    "wrist_roll": [5, "sts3215"],
    "gripper": [6, "sts3215"],
}
# Joint names order expected by So100RobotConfig/ManipulatorRobot
# Check lerobot/common/robot_devices/robots/configs.py if unsure
JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
NUM_JOINTS = len(JOINT_NAMES)
NUM_IK_JOINTS = 4 # Number of joints controlled by IK (shoulder_pan, shoulder_lift, elbow_flex, wrist_flex)
WRIST_ROLL_INDEX = 4
GRIPPER_INDEX = 5

# --- Robot Configuration ---
# You might need to adjust the port names if they are different on your system.
robot_config = So100RobotConfig(
    # Example: Override default ports if needed
    # leader_arms={"main": FeetechMotorsBusConfig(port="/dev/ttyYOUR_LEADER_PORT", ...)},
    follower_arms={"main": FeetechMotorsBusConfig(port="/dev/ttySO100follower", motors=common_motors.copy())},
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
end_effector_link_index = -1 # PyBullet index for the link used as IK target

# --- URDF Joint Limits (Read from file) --- #
# Global variables to store URDF limits read by PyBullet
URDF_LIMITS_MIN_RAD = np.full(NUM_JOINTS, -math.pi) # Default fallback
URDF_LIMITS_MAX_RAD = np.full(NUM_JOINTS, math.pi)
URDF_LIMITS_RANGE_RAD = np.full(NUM_JOINTS, 2 * math.pi)
URDF_LIMITS_MIN_DEG = np.rad2deg(URDF_LIMITS_MIN_RAD)
URDF_LIMITS_MAX_DEG = np.rad2deg(URDF_LIMITS_MAX_RAD)

print("--- Using URDF Limits (from file) ---")
# Note: Limits will be printed after reading in setup_pybullet

# --- PyBullet Setup (Adapted from keyboard_teleop.py) ---
def setup_pybullet():
    global physicsClient, robotId, p_joint_indices, viz_markers, debug_line_ids, end_effector_link_index
    # --- Add URDF Limit globals --- #
    global URDF_LIMITS_MIN_RAD, URDF_LIMITS_MAX_RAD, URDF_LIMITS_RANGE_RAD
    global URDF_LIMITS_MIN_DEG, URDF_LIMITS_MAX_DEG

    try:
        physicsClient = p.connect(p.GUI)
    except p.error as e:
        print(f"Could not connect to PyBullet GUI, attempting DIRECT connection: {e}")
        try:
            physicsClient = p.connect(p.DIRECT) # Fallback for headless
        except p.error as direct_e:
            print(f"Failed to connect to PyBullet DIRECT mode: {direct_e}")
            print("PyBullet visualization will be disabled.")
            physicsClient = -1
            return False

    if physicsClient < 0: return False

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0,0,-9.81)
    p.loadURDF("plane.urdf")

    # --- Load the NEW robot URDF --- #
    urdf_path = "URDF/SO_5DOF_ARM100_8j/urdf/so100.urdf"
    print(f"Loading URDF: {urdf_path}")
    if not os.path.exists(urdf_path):
        print(f"ERROR: URDF file not found at {urdf_path}", file=sys.stderr)
        if p.isConnected(physicsClient): p.disconnect(physicsClient)
        physicsClient = -1
        return False

    robot_start_pos = [0, 0, 0]
    robot_start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    try:
        robotId = p.loadURDF(urdf_path, robot_start_pos, robot_start_orientation, useFixedBase=1)
    except p.error as load_err:
         print(f"ERROR: Failed to load URDF: {load_err}", file=sys.stderr)
         if p.isConnected(physicsClient): p.disconnect(physicsClient)
         physicsClient = -1
         return False

    # Map JOINT_NAMES to pybullet joint indices
    # NOTE: Joint names in this new URDF might be different!
    # We need to inspect the URDF or adjust the mapping if startup fails.
    num_p_joints = p.getNumJoints(robotId)
    p_name_to_index = {}
    print("\nAvailable PyBullet Joints in URDF:")
    for i in range(num_p_joints):
        info = p.getJointInfo(robotId, i)
        joint_name_bytes = info[1]
        joint_name_str = joint_name_bytes.decode('UTF-8')
        link_name_str = info[12].decode('UTF-8')
        joint_type = info[2]
        print(f"  Index: {i}, Name: '{joint_name_str}', Type: {joint_type}, Link: '{link_name_str}'")
        p_name_to_index[joint_name_str] = i
        if joint_type != p.JOINT_FIXED:
            p.setJointMotorControl2(robotId, i, p.VELOCITY_CONTROL, force=0)

    # --- Attempt to map our expected JOINT_NAMES to PyBullet indices --- #
    # This map likely needs adjustment based on the new URDF's joint names
    # Check the printout above to see the actual names
    urdf_to_internal_name_map = {
        # --- Updated based on URDF printout --- #
        "Rotation": "shoulder_pan",
        "Pitch": "shoulder_lift",
        "Elbow": "elbow_flex",
        "Wrist_Pitch": "wrist_flex",
        "Wrist_Roll": "wrist_roll",
        "Jaw": "gripper",
    }
    print("\nAttempting to map internal names to PyBullet indices using map:")
    print(urdf_to_internal_name_map)

    p_joint_indices_list = [None] * NUM_JOINTS
    missing_joints = []
    mapped_internal_names = set()
    for urdf_name, internal_name in urdf_to_internal_name_map.items():
        if internal_name not in JOINT_NAMES:
             print(f"Warning: Internal name '{internal_name}' in map is not in expected JOINT_NAMES list. Skipping.", file=sys.stderr)
             continue
        if urdf_name in p_name_to_index:
            target_idx = JOINT_NAMES.index(internal_name)
            if p_joint_indices_list[target_idx] is not None:
                 print(f"Warning: Internal name '{internal_name}' already mapped. Check map for duplicates.", file=sys.stderr)
            else:
                 p_joint_indices_list[target_idx] = p_name_to_index[urdf_name]
                 mapped_internal_names.add(internal_name)
                 print(f"  Mapped: '{internal_name}' -> URDF '{urdf_name}' (Index {p_name_to_index[urdf_name]})")
        else:
            missing_joints.append(urdf_name)
            print(f"  Failed map: URDF joint '{urdf_name}' (for '{internal_name}') not found in loaded URDF.")

    unmapped_internal = [name for name in JOINT_NAMES if name not in mapped_internal_names]
    if unmapped_internal:
        print(f"ERROR: Could not map required internal joints: {unmapped_internal}", file=sys.stderr)
        # Don't disconnect yet, allow limit reading attempt
        # return False # Signal failure

    if missing_joints:
         print(f"Warning: Some URDF joints specified in the map were not found in the loaded URDF: {missing_joints}", file=sys.stderr)

    p_joint_indices = p_joint_indices_list
    print(f"\nFinal PyBullet Indices (matched to {JOINT_NAMES}): {p_joint_indices}")

    # --- Find End Effector Link Index --- #
    # This likely needs adjustment based on the new URDF's link names
    # Check the printout above. Common names: 'gripper_link', 'tool_link', 'ee_link'
    target_link_name = "Fixed_Jaw" # Updated - Associated with Wrist_Roll joint
    end_effector_link_index = -1
    print(f"\nSearching for End Effector Link: '{target_link_name}'")
    found_link_index = None
    for i in range(num_p_joints):
        info = p.getJointInfo(robotId, i)
        link_name_str = info[12].decode('UTF-8')
        if link_name_str == target_link_name:
            # PyBullet uses the link index matching the joint index CONROLLING that link
            found_link_index = i
            break

    if found_link_index is not None:
        end_effector_link_index = found_link_index
        print(f"Found End Effector Link '{target_link_name}' controlled by Joint Index: {end_effector_link_index}")
    else:
        print(f"ERROR: Could not find end-effector link '{target_link_name}' in URDF.", file=sys.stderr)
        # Don't disconnect yet
        # return False # Signal failure

    # --- Extract and Store URDF Limits for ALL Joints --- #
    temp_min_rad = list(URDF_LIMITS_MIN_RAD) # Start with defaults
    temp_max_rad = list(URDF_LIMITS_MAX_RAD)
    temp_range_rad = list(URDF_LIMITS_RANGE_RAD)
    temp_min_deg = list(URDF_LIMITS_MIN_DEG)
    temp_max_deg = list(URDF_LIMITS_MAX_DEG)

    print("\nReading URDF limits for all mapped joints:")
    limits_read_count = 0
    for i in range(NUM_JOINTS):
        pb_index = p_joint_indices[i]
        joint_name = JOINT_NAMES[i]
        if pb_index is not None:
            try:
                joint_info = p.getJointInfo(robotId, pb_index)
                lower = joint_info[8] # Lower limit in radians
                upper = joint_info[9] # Upper limit in radians

                # Use limits only if valid (lower < upper)
                if lower < upper:
                    temp_min_rad[i] = lower
                    temp_max_rad[i] = upper
                    temp_range_rad[i] = upper - lower
                    temp_min_deg[i] = math.degrees(lower)
                    temp_max_deg[i] = math.degrees(upper)
                    print(f"  {joint_name:<15} (Idx:{pb_index}): Min={temp_min_deg[i]:8.2f} deg, Max={temp_max_deg[i]:8.2f} deg")
                    limits_read_count += 1
                else:
                    print(f"  {joint_name:<15} (Idx:{pb_index}): Invalid/No limits in URDF (lower={lower:.2f}, upper={upper:.2f}). Using defaults [-180, 180].")
            except p.error as info_err:
                 print(f"Error getting joint info for {joint_name} (Idx:{pb_index}): {info_err}", file=sys.stderr)
        else:
             print(f"  {joint_name:<15}: Not mapped to PyBullet index. Using default limits.")
    
    # Store globally
    URDF_LIMITS_MIN_RAD = np.array(temp_min_rad)
    URDF_LIMITS_MAX_RAD = np.array(temp_max_rad)
    URDF_LIMITS_RANGE_RAD = np.array(temp_range_rad)
    URDF_LIMITS_MIN_DEG = np.array(temp_min_deg)
    URDF_LIMITS_MAX_DEG = np.array(temp_max_deg)
    
    if limits_read_count < NUM_JOINTS:
         print("Warning: Failed to read valid URDF limits for one or more joints. Default limits were used.")

    # Final check for fatal errors
    if None in p_joint_indices or end_effector_link_index == -1:
        print("ERROR: Failed to map all required joints or find end effector link. Cannot proceed.", file=sys.stderr)
        if p.isConnected(physicsClient): p.disconnect(physicsClient)
        physicsClient = -1
        return False

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

        # Initialize PyBullet (this now reads URDF limits for all joints)
        print("Initializing PyBullet...")
        pybullet_enabled = setup_pybullet()
        if not pybullet_enabled:
            print("ERROR: PyBullet initialization failed. Exiting.", file=sys.stderr)
            # Attempt cleanup even if setup failed partially
            if physicsClient is not None and p.isConnected(physicsClient):
                 p.disconnect(physicsClient)
            sys.exit(1)
        
        print("PyBullet setup successful.")

        # Get initial robot state (use this directly)
        print("Reading initial robot state...")
        observation = robot.capture_observation()
        if "observation.state" in observation:
            initial_joint_angles_deg_tensor = observation["observation.state"]
            current_joint_angles_deg = initial_joint_angles_deg_tensor.cpu().numpy()
            print(f"Initial state read (degrees): {current_joint_angles_deg}")
            # DO NOT CLAMP initial state here
        else:
            print("Warning: Could not read initial state. Using zeros.", file=sys.stderr)
            current_joint_angles_deg = np.zeros(NUM_JOINTS)

        # --- Use PyBullet FK based on ACTUAL INITIAL STATE to get initial EF state --- #
        if pybullet_enabled and end_effector_link_index != -1:
            current_angles_rad = np.deg2rad(current_joint_angles_deg)
            # Reset PyBullet state to match actual initial state
            for i in range(NUM_JOINTS):
                if p_joint_indices[i] is not None:
                     p.resetJointState(robotId, p_joint_indices[i], targetValue=current_angles_rad[i])

            # FK calculation
            link_state = p.getLinkState(robotId, end_effector_link_index)
            current_ef_position = np.array(link_state[0])
            current_ef_orientation_quat = link_state[1]
            current_ef_rpy_rad = p.getEulerFromQuaternion(current_ef_orientation_quat)
            current_ef_rpy_deg = np.rad2deg(current_ef_rpy_rad)
            print(f"Initial FK EF Position: {current_ef_position}")
            print(f"Initial FK EF RPY (deg): {current_ef_rpy_deg}")
            # Set initial target state based on initial FK
            target_ef_position = current_ef_position.copy()
            target_pitch_deg = current_ef_rpy_deg[1]
            target_wrist_roll_deg = current_joint_angles_deg[WRIST_ROLL_INDEX] # From initial state
            target_gripper_deg = current_joint_angles_deg[GRIPPER_INDEX]     # From initial state
        else:
             print("Warning: PyBullet disabled or EF link not found. Cannot get initial FK.", file=sys.stderr)
             # Use initial angles directly for initial targets
             target_ef_position = np.array([0.2, 0.0, 0.15]) # Default target pos if FK fails
             target_pitch_deg = 0.0
             target_wrist_roll_deg = current_joint_angles_deg[WRIST_ROLL_INDEX] # From initial state
             target_gripper_deg = current_joint_angles_deg[GRIPPER_INDEX]     # From initial state

        # Update PyBullet with initial state
        if pybullet_enabled:
            update_pybullet_visuals(current_joint_angles_deg, target_ef_position, current_ef_position)
            if end_effector_link_index != -1:
                p.resetBasePositionAndOrientation(viz_markers['target'], target_ef_position, [0,0,0,1])
                p.resetBasePositionAndOrientation(viz_markers['actual_ef'], current_ef_position, [0,0,0,1])


        # Start keyboard listener
        start_keyboard_listener()

        # --- Main Control Loop (starts from actual initial state) --- #
        print("\nEntering main control loop...")
        last_send_time = time.time()
        send_interval = 0.05 # Send commands roughly every 50ms (20Hz)

        while control_active:
            loop_start_time = time.time()

            # 1. Update Target State based on Keyboard Input
            target_ef_position += delta_pos
            target_pitch_deg += delta_pitch
            target_wrist_roll_deg += delta_wrist_roll
            target_gripper_deg += delta_gripper

            # 2. Get Current Robot State (for visualization) using PyBullet FK
            ik_solution_angles_deg = current_joint_angles_deg[:NUM_IK_JOINTS] # Default to current if IK fails
            if pybullet_enabled and end_effector_link_index != -1:
                # Update PyBullet sim joints based on last command state
                current_angles_rad = np.deg2rad(current_joint_angles_deg)
                for i in range(NUM_JOINTS):
                    if p_joint_indices[i] is not None:
                         p.resetJointState(robotId, p_joint_indices[i], targetValue=current_angles_rad[i])

                # Get current EF position from PyBullet FK for visualization
                link_state = p.getLinkState(robotId, end_effector_link_index)
                current_ef_position_viz = np.array(link_state[0])

                # 3. Calculate IK using PyBullet for the first NUM_IK_JOINTS
                target_pitch_rad = np.deg2rad(target_pitch_deg)
                target_orientation_quat = p.getQuaternionFromEuler([0, target_pitch_rad, 0])

                # Use URDF limits (read during setup) for IK
                # Slice the global arrays for the IK joints
                ik_lower_limits = URDF_LIMITS_MIN_RAD[:NUM_IK_JOINTS].tolist()
                ik_upper_limits = URDF_LIMITS_MAX_RAD[:NUM_IK_JOINTS].tolist()
                ik_joint_ranges = URDF_LIMITS_RANGE_RAD[:NUM_IK_JOINTS].tolist()

                ik_joint_indices = p_joint_indices[:NUM_IK_JOINTS]

                try:
                    # IK result (ik_solution_rad) is in PyBullet's coordinate system
                    ik_solution_rad = p.calculateInverseKinematics(
                        bodyUniqueId=robotId,
                        endEffectorLinkIndex=end_effector_link_index,
                        targetPosition=target_ef_position.tolist(),
                        targetOrientation=target_orientation_quat,
                        lowerLimits=ik_lower_limits,     # Use URDF limits
                        upperLimits=ik_upper_limits,     # Use URDF limits
                        jointRanges=ik_joint_ranges,     # Use URDF limits
                        restPoses=[0.0] * len(ik_joint_indices),
                        maxNumIterations=100,
                        residualThreshold=1e-4
                    )
                    ik_solution_angles_deg = np.rad2deg(ik_solution_rad[:NUM_IK_JOINTS])
                except Exception as ik_err:
                     print(f"Warning: PyBullet IK failed: {ik_err}", file=sys.stderr)
                     # Keep previous angles if IK fails
                     ik_solution_angles_deg = current_joint_angles_deg[:NUM_IK_JOINTS]
            else:
                 current_ef_position_viz = target_ef_position
                 print("Warning: PyBullet disabled or EF link not found. IK cannot be calculated.", file=sys.stderr)
                 # Keep previous angles if no PyBullet
                 ik_solution_angles_deg = current_joint_angles_deg[:NUM_IK_JOINTS]

            # 4. Construct Full Action Command (in Lerobot/PyBullet Degrees - assumed aligned now)
            action_angles_deg = np.zeros(NUM_JOINTS)
            action_angles_deg[:NUM_IK_JOINTS] = ik_solution_angles_deg
            action_angles_deg[WRIST_ROLL_INDEX] = target_wrist_roll_deg
            action_angles_deg[GRIPPER_INDEX] = target_gripper_deg

            # >>> Final Clamp: Use simple np.clip with URDF degree limits <<<
            action_angles_deg = np.clip(action_angles_deg,
                                        URDF_LIMITS_MIN_DEG,
                                        URDF_LIMITS_MAX_DEG)

            # Update current angles based on the *clamped* command
            current_joint_angles_deg = action_angles_deg.copy()

            # 5. Send Action to Robot
            current_time = time.time()
            if current_time - last_send_time >= send_interval:
                action_tensor = torch.from_numpy(action_angles_deg).float()
                try:
                    robot.send_action(action_tensor)
                    last_send_time = current_time
                except Exception as send_err:
                    print(f"Error sending action: {send_err}", file=sys.stderr)

            # 6. Update PyBullet Visualization (use non-offset angles)
            if pybullet_enabled:
                update_pybullet_visuals(current_joint_angles_deg, target_ef_position, current_ef_position_viz)

            # 7. Loop Rate Control
            elapsed_time = time.time() - loop_start_time
            sleep_time = max(0, send_interval - elapsed_time)
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
        # Check if robot object exists and has the is_connected attribute/method
        robot_is_connected = False
        try:
            # Use hasattr to be safe, although lerobot API should guarantee is_connected
            if robot and hasattr(robot, 'is_connected') and robot.is_connected:
                robot_is_connected = True
        except Exception as check_err:
             print(f"Could not check robot connection status: {check_err}")
             robot_is_connected = False # Assume not connected if check fails
        
        if robot_is_connected:
            try:
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
                print("Robot object state prevents torque disable check (e.g., missing follower_arms).", file=sys.stderr)
            except Exception as final_err:
                print(f"An error occurred during torque disable: {final_err}", file=sys.stderr)

            # Disconnect robot only if it was connected
            try:
                 print("Disconnecting robot...")
                 robot.disconnect()
                 print("Robot disconnected.")
            except RobotDeviceNotConnectedError:
                 print("Robot was already disconnected.") # Handle expected error
            except Exception as disc_err:
                 print(f"An error occurred during robot disconnect: {disc_err}", file=sys.stderr)
        else:
            print("Robot not connected or connection status unknown, skipping torque disable and disconnect.")
            
        # Disconnect PyBullet
        if physicsClient is not None and p.isConnected(physicsClient):
             print("Disconnecting PyBullet...")
             p.disconnect(physicsClient)
             print("PyBullet disconnected.")

        print("Shutdown complete.") 