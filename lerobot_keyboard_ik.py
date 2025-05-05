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
NUM_IK_JOINTS = 4 # Number of joints controlled by IK (shoulder_pan, shoulder_lift, elbow_flex, wrist_flex)
WRIST_ROLL_INDEX = 4
GRIPPER_INDEX = 5

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
end_effector_link_index = -1 # PyBullet index for the link used as IK target

# Joint limits (loaded from inverse_kinematics or defined here)
# REMOVED - Assuming lerobot handles limits or they are enforced by PyBullet IK implicitly

# --- Kinematics Functions Removed ---
# Using PyBullet's internal kinematics based on the loaded URDF


# --- PyBullet Setup (Adapted from keyboard_teleop.py) ---
def setup_pybullet():
    global physicsClient, robotId, p_joint_indices, viz_markers, debug_line_ids, end_effector_link_index
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

        # Load joint limits for IK - REMOVED, assuming lerobot handles this.
        # load_joint_limits()

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

            # --- Use PyBullet FK to get initial EF state ---
            if pybullet_enabled and end_effector_link_index != -1:
                 # Set PyBullet joints to initial state to calculate FK accurately
                current_angles_rad = np.deg2rad(current_joint_angles_deg)
                for i in range(NUM_JOINTS):
                    if p_joint_indices[i] is not None:
                         p.resetJointState(robotId, p_joint_indices[i], targetValue=current_angles_rad[i])

                link_state = p.getLinkState(robotId, end_effector_link_index)
                current_ef_position = np.array(link_state[0]) # World position [x,y,z]
                current_ef_orientation_quat = link_state[1] # World orientation [x,y,z,w]
                current_ef_rpy_rad = p.getEulerFromQuaternion(current_ef_orientation_quat)
                current_ef_rpy_deg = np.rad2deg(current_ef_rpy_rad)
                print(f"Initial PyBullet FK EF Position: {current_ef_position}")
                print(f"Initial PyBullet FK EF RPY (deg): {current_ef_rpy_deg}")
                # Set initial target state to current FK state
                target_ef_position = current_ef_position.copy()
                target_pitch_deg = current_ef_rpy_deg[1] # Use pitch from FK
                # Keep direct angle control for wrist roll and gripper based on initial read
                target_wrist_roll_deg = current_joint_angles_deg[WRIST_ROLL_INDEX]
                target_gripper_deg = current_joint_angles_deg[GRIPPER_INDEX]
            else:
                 print("Warning: Cannot get initial FK state (PyBullet disabled or end effector not found). Using defaults.", file=sys.stderr)
                 # Keep the default initial target values from global scope if FK fails
                 # FK from zeros is no longer relevant as the function is removed.
                 current_joint_angles_deg = np.zeros(NUM_JOINTS) # Fallback

        else:
            print("Warning: 'observation.state' not found in initial observation. Using default state values.", file=sys.stderr)
            # Keep the default initial target values
            current_joint_angles_deg = np.zeros(NUM_JOINTS) # Fallback
            # Cannot calculate FK from zeros anymore. Default targets remain.

        # Update PyBullet with initial state (handles case where FK failed)
        if pybullet_enabled:
            update_pybullet_visuals(current_joint_angles_deg, target_ef_position, current_ef_position) # current_ef_position might be default here
            if end_effector_link_index != -1: # Place markers accurately if FK worked
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
            # Apply the step directly, as delta values are set to +/- STEP or 0 by keyboard callbacks
            target_ef_position += delta_pos
            target_pitch_deg += delta_pitch
            target_wrist_roll_deg += delta_wrist_roll
            target_gripper_deg += delta_gripper
            # TODO: Clamp target angles (wrist roll, gripper) to reasonable limits if known
            # Example clamping (adjust ranges as needed):
            # target_wrist_roll_deg = np.clip(target_wrist_roll_deg, -180, 180)
            # target_gripper_deg = np.clip(target_gripper_deg, 0, 100) # Assuming 0-100 range for gripper

            # 2. Get Current Robot State (for visualization) using PyBullet FK
            ik_solution_angles_deg = np.zeros(NUM_IK_JOINTS) # Initialize
            if pybullet_enabled and end_effector_link_index != -1:
                # Update PyBullet sim joints based on last command (approximation of reality)
                current_angles_rad = np.deg2rad(current_joint_angles_deg)
                for i in range(NUM_JOINTS):
                    if p_joint_indices[i] is not None:
                         p.resetJointState(robotId, p_joint_indices[i], targetValue=current_angles_rad[i])

                # Get current EF position from PyBullet FK
                link_state = p.getLinkState(robotId, end_effector_link_index)
                current_ef_position = np.array(link_state[0])
                # current_ef_orientation_quat = link_state[1] # Orientation if needed
                # current_ef_rpy_rad = p.getEulerFromQuaternion(current_ef_orientation_quat)
                # current_ef_rpy_deg = np.rad2deg(current_ef_rpy_rad)

                # 3. Calculate IK using PyBullet for the first NUM_IK_JOINTS
                # Target orientation: We only control pitch directly via keyboard.
                # Let IK determine roll and yaw based on position target.
                # Construct target orientation quaternion with desired pitch.
                # Assume target roll and yaw are 0 relative to the base or let IK solve freely.
                # For simplicity, let's try without specifying orientation first (targetOrientation=None)
                # If that doesn't work well, construct quat from Euler [0, target_pitch_rad, 0]
                target_pitch_rad = np.deg2rad(target_pitch_deg)
                target_orientation_quat = p.getQuaternionFromEuler([0, target_pitch_rad, 0]) # Roll=0, Pitch=Target, Yaw=0

                # Get joint limits from PyBullet if needed (optional, IK might handle it)
                pb_lower_limits = []
                pb_upper_limits = []
                pb_joint_ranges = []
                for i in range(NUM_IK_JOINTS):
                    joint_info = p.getJointInfo(robotId, p_joint_indices[i])
                    pb_lower_limits.append(joint_info[8])
                    pb_upper_limits.append(joint_info[9])
                    pb_joint_ranges.append(joint_info[9] - joint_info[8])


                # PyBullet IK calculates for joints 0 to end_effector_link_index
                # We need the first NUM_IK_JOINTS = 4: Shoulder Pan/Lift, Elbow Flex, Wrist Flex
                # The p_joint_indices should map our first 4 JOINT_NAMES to the correct pybullet indices.
                ik_joint_indices = p_joint_indices[:NUM_IK_JOINTS]

                # Use current pose as starting point? PyBullet IK doesn't explicitly take initial guess like ours did.
                # It uses the current state of the simulation model.
                try:
                    # Note: calculateInverseKinematics returns angles in RADIANS
                    ik_solution_rad = p.calculateInverseKinematics(
                        bodyUniqueId=robotId,
                        endEffectorLinkIndex=end_effector_link_index,
                        targetPosition=target_ef_position.tolist(),
                        targetOrientation=target_orientation_quat, # Provide target pitch
                        lowerLimits=pb_lower_limits, # Use URDF limits
                        upperLimits=pb_upper_limits, # Use URDF limits
                        jointRanges=pb_joint_ranges, # Use URDF ranges
                        restPoses=[0.0] * len(ik_joint_indices), # Neutral rest pose
                        maxNumIterations=100,
                        residualThreshold=1e-4
                    )
                    # Ensure we only take the first NUM_IK_JOINTS solutions returned
                    ik_solution_angles_deg = np.rad2deg(ik_solution_rad[:NUM_IK_JOINTS])

                except Exception as ik_err:
                     print(f"Warning: PyBullet IK failed: {ik_err}", file=sys.stderr)
                     # Keep previous angles or zeros if IK fails? Keep ik_solution_angles_deg as zeros for now.


            else:
                # Cannot run FK or IK if PyBullet is not enabled or EF link not found
                # Actions will be based on zero IK result + keyboard overrides for wrist/gripper
                 current_ef_position = target_ef_position # No FK, assume target is actual for viz
                 print("Warning: PyBullet disabled or EF link not found. IK cannot be calculated.", file=sys.stderr)


            # 4. Construct Full Action Command (Angles in Degrees)
            # Combine IK solution with direct keyboard control for wrist/gripper
            action_angles_deg = np.zeros(NUM_JOINTS)
            action_angles_deg[:NUM_IK_JOINTS] = ik_solution_angles_deg # Result from PyBullet IK
            action_angles_deg[WRIST_ROLL_INDEX] = target_wrist_roll_deg # Wrist roll (index 4)
            action_angles_deg[GRIPPER_INDEX] = target_gripper_deg   # Gripper (index 5)

            # Update current angles based on command for smoother Viz next iteration
            # and as the base for the next PyBullet state reset before FK/IK
            current_joint_angles_deg = action_angles_deg.copy()
            # current_ef_position is updated above using PyBullet FK


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