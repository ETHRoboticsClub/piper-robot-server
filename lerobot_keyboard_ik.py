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

# --- URDF Joint Limits for IK (Using Wide Fallback) --- #
# We use wide limits for IK because empirical limits might wrap around
# and URDF limits were reported as invalid.
urdf_limits_min_rad = [-math.pi] * NUM_IK_JOINTS
urdf_limits_max_rad = [ math.pi] * NUM_IK_JOINTS
urdf_limits_range_rad = [2 * math.pi] * NUM_IK_JOINTS

# --- Empirical Degree Limits (from investigate.py output) --- #
# Define based on the min/max reported degrees (ASSUMING NO WRAPPING based on user investigation)
EMPIRICAL_MIN_DEG = np.array([ 90, -10,  -10, -105, -130,  0])
EMPIRICAL_MAX_DEG = np.array([ 270, 180, 170, 70,   80, 100])

# Calculate Radian versions for PyBullet IK (first NUM_IK_JOINTS)
EMPIRICAL_MIN_RAD = np.deg2rad(EMPIRICAL_MIN_DEG[:NUM_IK_JOINTS])
EMPIRICAL_MAX_RAD = np.deg2rad(EMPIRICAL_MAX_DEG[:NUM_IK_JOINTS])
EMPIRICAL_RANGE_RAD = EMPIRICAL_MAX_RAD - EMPIRICAL_MIN_RAD

print("--- Using Empirical Degree Limits (Assumed Non-Wrapping) ---")
for i, name in enumerate(JOINT_NAMES):
    print(f"  {name}: Min={EMPIRICAL_MIN_DEG[i]:.2f}, Max={EMPIRICAL_MAX_DEG[i]:.2f}")
print("---------------------------------------------------------")

# --- PyBullet Setup (Adapted from keyboard_teleop.py) ---
def setup_pybullet():
    global physicsClient, robotId, p_joint_indices, viz_markers, debug_line_ids, end_effector_link_index
    # No specific limit setup needed here anymore, done globally

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

    # Find the end-effector link index for IK calculations
    # We target the frame BEFORE the wrist roll joint. From URDF, Wrist_Roll connects Wrist_Pitch_Roll -> Fixed_Gripper
    # So, we use the link "Fixed_Gripper" as the end point for the 4-DOF IK.
    target_link_name = "Fixed_Gripper"
    end_effector_link_index = -1
    for i in range(num_p_joints):
        info = p.getJointInfo(robotId, i)
        link_name_str = info[12].decode('UTF-8')
        if link_name_str == target_link_name:
            # IMPORTANT: PyBullet's getLinkState uses the LINK index, which matches the JOINT index controlling it.
            end_effector_link_index = i
            break

    if end_effector_link_index == -1:
        print(f"ERROR: Could not find end-effector link '{target_link_name}' in URDF.", file=sys.stderr)
        p.disconnect(physicsClient)
        physicsClient = -1
        return False # Signal failure
    else:
        print(f"Using link '{target_link_name}' (PyBullet Index: {end_effector_link_index}) as IK/FK target.")

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

    # Using empirical limits (calculated globally) for PyBullet IK.
    print("Using empirical radian limits for PyBullet IK.")

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

        # Initialize PyBullet (this now reads URDF limits)
        print("Initializing PyBullet...")
        pybullet_enabled = setup_pybullet()
        if not pybullet_enabled:
            print("Continuing without PyBullet visualization.")

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
            current_angles_rad = np.deg2rad(current_joint_angles_deg) # Use actual initial angles
            # Reset PyBullet state to match actual initial state
            for i in range(NUM_JOINTS):
                if p_joint_indices[i] is not None:
                     p.resetJointState(robotId, p_joint_indices[i], targetValue=current_angles_rad[i])

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
            ik_solution_angles_deg = np.zeros(NUM_IK_JOINTS)
            if pybullet_enabled and end_effector_link_index != -1:
                # Update PyBullet sim joints based on last command
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

                # Use EMPIRICAL limits (converted to radians) for IK
                ik_lower_limits = EMPIRICAL_MIN_RAD.tolist()
                ik_upper_limits = EMPIRICAL_MAX_RAD.tolist()
                ik_joint_ranges = EMPIRICAL_RANGE_RAD.tolist()

                ik_joint_indices = p_joint_indices[:NUM_IK_JOINTS]

                try:
                    ik_solution_rad = p.calculateInverseKinematics(
                        bodyUniqueId=robotId,
                        endEffectorLinkIndex=end_effector_link_index,
                        targetPosition=target_ef_position.tolist(),
                        targetOrientation=target_orientation_quat,
                        lowerLimits=ik_lower_limits,     # Use Empirical limits
                        upperLimits=ik_upper_limits,     # Use Empirical limits
                        jointRanges=ik_joint_ranges,     # Use Empirical limits
                        restPoses=[0.0] * len(ik_joint_indices),
                        maxNumIterations=100,
                        residualThreshold=1e-4
                    )
                    ik_solution_angles_deg = np.rad2deg(ik_solution_rad[:NUM_IK_JOINTS])
                except Exception as ik_err:
                     print(f"Warning: PyBullet IK failed: {ik_err}", file=sys.stderr)
            else:
                 current_ef_position_viz = target_ef_position
                 print("Warning: PyBullet disabled or EF link not found. IK cannot be calculated.", file=sys.stderr)

            # 4. Construct Full Action Command
            action_angles_deg = np.zeros(NUM_JOINTS)
            action_angles_deg[:NUM_IK_JOINTS] = ik_solution_angles_deg
            action_angles_deg[WRIST_ROLL_INDEX] = target_wrist_roll_deg # Use target directly
            action_angles_deg[GRIPPER_INDEX] = target_gripper_deg   # Use target directly

            # >>> Final Clamp: Use simple np.clip with empirical degree limits <<<
            action_angles_deg = np.clip(action_angles_deg,
                                        EMPIRICAL_MIN_DEG,
                                        EMPIRICAL_MAX_DEG)

            # Update current angles based on the *clamped* command
            current_joint_angles_deg = action_angles_deg.copy()

            # 5. Send Action to Robot
            current_time = time.time() # Need time import back
            if current_time - last_send_time >= send_interval:
                action_tensor = torch.from_numpy(action_angles_deg).float()
                try:
                    robot.send_action(action_tensor)
                    last_send_time = current_time
                except Exception as send_err:
                    print(f"Error sending action: {send_err}", file=sys.stderr)

            # 6. Update PyBullet Visualization (use _viz position)
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