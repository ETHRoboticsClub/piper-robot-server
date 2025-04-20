# --- Keyboard Teleoperation ---
# This script allows for teleoperation of the robot using the keyboard
# Modified from rabhishek100's uploaded code on https://github.com/huggingface/lerobot/issues/568
import numpy as np
from forward_kinematics import forward_kinematics
from inverse_kinematics import iterative_ik
from lerobot.common.robot_devices.motors.feetech import FeetechMotorsBus, TorqueMode, FeetechMotorsBusConfig
import time
from pynput import keyboard
import pybullet as p
import pybullet_data
import math

# --- Experimentally Determined Calibration Values (for Viz & Control) --- 
# Reverted ZERO_STEPS[0] adjustment. Discrepancy is likely in FK vs URDF interpretation.
EXP_ZERO_STEPS = [2048.0, 2052.0, 2500.0, 2600.0, 3846.0, 2048.0]
EXP_DIRECTIONS = [-1, -1, -1, 1, -1, 1]

# --- Remove JSON Calibration Values --- 
# JSON_ZERO_STEPS = [2048.0, 2052.0, 2047.0, 2047.0] 
# JSON_DIRECTIONS = [-1.0, 1.0, 1.0, 1.0]         

# --- Calibration Data Functions --- 
def get_base_calibration():
    """Calibration for the first 4 joints (using EXP values for IK/Control Output)."""
    # Keep this using EXP values as it's used by angles_to_servo_steps
    if len(EXP_ZERO_STEPS) < 4 or len(EXP_DIRECTIONS) < 4:
        raise ValueError("Experimental calibration data incomplete.")
    return [
        {"zero_step": EXP_ZERO_STEPS[0], "direction": EXP_DIRECTIONS[0]}, 
        {"zero_step": EXP_ZERO_STEPS[1], "direction": EXP_DIRECTIONS[1]}, 
        {"zero_step": EXP_ZERO_STEPS[2], "direction": EXP_DIRECTIONS[2]}, 
        {"zero_step": EXP_ZERO_STEPS[3], "direction": EXP_DIRECTIONS[3]},
    ]

def get_full_calibration():
    """Calibration for all 6 joints (using EXP values for Visualization)."""
    # Keep this using EXP values as it drives the pybullet visualization
    if len(EXP_ZERO_STEPS) != 6 or len(EXP_DIRECTIONS) != 6:
        raise ValueError("Experimental calibration data incomplete.")
    calib_list = []
    for i in range(6):
        calib_list.append({
            "zero_step": EXP_ZERO_STEPS[i],
            "direction": EXP_DIRECTIONS[i]
        })
    return calib_list

# --- Angle Conversion --- 
def servo_steps_to_angles(steps):
    # Uses base calibration (first 4 joints)
    calib_data = get_base_calibration()
    if len(steps) != len(calib_data):
        raise ValueError(f"Expected {len(calib_data)} steps for servo_steps_to_angles.")
    degrees_per_step = 360.0 / 4096.0
    angle_values = []
    for i, step in enumerate(steps):
        cal = calib_data[i]
        zero_step = cal["zero_step"]
        direction = cal["direction"]
        angle_value = (step - zero_step) * direction * degrees_per_step
        angle_values.append(angle_value % 360)
    return angle_values

def angles_to_servo_steps(angles):
    # Uses base calibration (first 4 joints)
    calib_data = get_base_calibration()
    if len(angles) != len(calib_data):
        raise ValueError(f"Expected {len(calib_data)} angles for angles_to_servo_steps.")
    steps_per_degree = 4096 / 360.0
    step_values = []
    for i, angle in enumerate(angles):
        cal = calib_data[i]
        zero_step = cal["zero_step"]
        direction = cal["direction"]
        step_value = int(zero_step + direction * angle * steps_per_degree)
        step_values.append(step_value % 4096)
    return step_values

def steps_to_radians_for_viz(steps, calib_data):
    """Converts full 6 steps to radians for pybullet visualization."""
    if len(steps) != len(calib_data):
        raise ValueError(f"Expected {len(calib_data)} steps for visualization conversion.")
    radians_per_step = 2 * math.pi / 4096.0
    radian_values = []
    for i, step in enumerate(steps):
        cal = calib_data[i]
        zero_step = cal["zero_step"]
        direction = cal["direction"]
        # Calculate angle relative to zero, no modulo for pybullet
        angle_value = (step - zero_step) * direction * radians_per_step 
        radian_values.append(angle_value)
    return radian_values

# --- Revert to single get_degrees_from_steps using EXP values --- 
def get_degrees_from_steps(steps, num_joints=4):
    """Converts the first num_joints steps to degrees using EXP base calibration."""
    calib_data = get_base_calibration()[:num_joints]
    if len(steps) < num_joints:
         raise ValueError(f"Need at least {num_joints} steps for degree conversion.")
    degrees_per_step = 360.0 / 4096.0
    angle_values = []
    relevant_steps = steps[:num_joints]
    for i, step in enumerate(relevant_steps):
        cal = calib_data[i]
        zero_step = cal["zero_step"]
        direction = cal["direction"]
        angle_value = (step - zero_step) * direction * degrees_per_step
        angle_values.append(angle_value)
    return angle_values
# --- End revert --- 

# --- Robot and Motor Config --- 
follower_port = "/dev/ttyACM0"
follower_motors = {
    "Shoulder_Rotation": (1, "sts3215"), # Corresponds to motor controlling base rotation
    "Shoulder_Pitch": (2, "sts3215"),    # Corresponds to motor controlling shoulder lift
    "Elbow": (3, "sts3215"),             # Corresponds to motor controlling elbow flex
    "Wrist_Pitch": (4, "sts3215"),      # Corresponds to motor controlling wrist flex/pitch
    "Wrist_Roll": (5, "sts3215"),       # Corresponds to motor controlling wrist roll
    "Gripper": (6, "sts3215"),          # Corresponds to motor controlling gripper
}
follower_config = FeetechMotorsBusConfig(port=follower_port, motors=follower_motors)
# --- Define NUM_MOTORS --- 
NUM_MOTORS = len(follower_motors)

# --- PyBullet Setup --- 
def setup_pybullet():
    physicsClient = p.connect(p.GUI)
    # Add data path for loading plane.urdf
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0,0,-9.81) # Set gravity (though simulation is off)
    planeId = p.loadURDF("plane.urdf") # Load a ground plane

    # Load the robot URDF
    urdf_path = "URDF/SO_5DOF_ARM100_05d.SLDASM/urdf/SO_5DOF_ARM100_05d.SLDASM.urdf"
    robot_start_pos = [0, 0, 0] # Start at origin
    robot_start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    print(f"Loading URDF: {urdf_path}")
    robotId = p.loadURDF(urdf_path, robot_start_pos, robot_start_orientation, useFixedBase=1)

    # Get map of joint names to pybullet indices
    num_joints = p.getNumJoints(robotId)
    p_joint_indices = {}
    for i in range(num_joints):
        info = p.getJointInfo(robotId, i)
        joint_name = info[1].decode('UTF-8') # Get joint name
        # Assuming our follower_motors keys match relevant URDF joint names
        if joint_name in follower_motors:
            p_joint_indices[joint_name] = i
            # Disable velocity control for visualization
            p.setJointMotorControl2(robotId, i, p.VELOCITY_CONTROL, force=0)
    print(f"Mapped Pybullet Joint Indices: {p_joint_indices}")

    # --- Create Visual Markers --- 
    markers = {}
    # Target (Red)
    target_shape = p.createVisualShape(p.GEOM_SPHERE, radius=0.02, rgbaColor=[1, 0, 0, 0.8])
    markers['target'] = p.createMultiBody(baseVisualShapeIndex=target_shape, basePosition=[0,0,1])
    # Actual End Effector (Blue)
    actual_shape = p.createVisualShape(p.GEOM_SPHERE, radius=0.02, rgbaColor=[0, 0, 1, 0.8])
    markers['actual_ef'] = p.createMultiBody(baseVisualShapeIndex=actual_shape, basePosition=[0,0,1])
    # --- NO Frame Markers Needed --- 
    # We will draw lines instead
    # --- End Marker Creation --- 
        
    print(f"Pybullet Robot ID: {robotId}")
    print(f"Mapped Pybullet Joint Indices: {p_joint_indices}")
    return physicsClient, robotId, p_joint_indices, markers

# --- Initialize PyBullet --- 
physicsClient, robotId, p_joint_indices, viz_markers = setup_pybullet()

# --- Real Robot Connection --- 
follower_arm = FeetechMotorsBus(follower_config)
follower_arm.connect()

# --- Set Hardware Motion Profile Parameters --- 
print("Setting initial motor profile velocity and acceleration...")
try:
    # Use correct data_names from SCS_SERIES_CONTROL_TABLE
    ACCELERATION_NAME = "Acceleration" # Address 41
    GOAL_SPEED_NAME = "Goal_Speed"       # Address 46
    INITIAL_ACCELERATION = 20  # Value for Acceleration (Units? Time?)
    INITIAL_GOAL_SPEED = 1500 # Value for Goal Speed (Units? Steps/s?)

    # Unlock EPROM if necessary (Might not be needed if already unlocked or RAM register)
    # follower_arm.write("Lock", 0) 
    
    all_motor_names = list(follower_motors.keys())
    # Set Acceleration first
    follower_arm.write(ACCELERATION_NAME, INITIAL_ACCELERATION, motor_names=all_motor_names)
    print(f"  Set {ACCELERATION_NAME} to {INITIAL_ACCELERATION}")
    # Set Goal Speed next
    follower_arm.write(GOAL_SPEED_NAME, INITIAL_GOAL_SPEED, motor_names=all_motor_names)
    print(f"  Set {GOAL_SPEED_NAME} to {INITIAL_GOAL_SPEED}")
    
    # Re-lock EPROM if unlocked
    # follower_arm.write("Lock", 1)
    
    # Enable Torque AFTER setting profiles (or ensure it was enabled before)
    follower_arm.write("Torque_Enable", TorqueMode.ENABLED.value)
    print("  Torque Enabled.")
    
except Exception as e:
    print(f"\nWARNING: Failed to set initial motion parameters: {e}")
    print("Attempting to enable torque anyway...")
    try:
        follower_arm.write("Torque_Enable", TorqueMode.ENABLED.value)
        print("  Torque Enabled.")
    except Exception as torque_e:
        print(f"ERROR: Failed to enable torque: {torque_e}")
        # Decide how to handle this - maybe exit?
# --- End Profile Setup ---

current_positions = follower_arm.read("Present_Position")
full_calibration = get_full_calibration() 

# --- Reinstate Initial State Calculation --- 
print("Calculating initial state...")
try:
    # Calculate initial angles (0-3) from read steps
    angles = servo_steps_to_angles(current_positions[:4])
    print(f"Arm start angles (deg 0-3): {angles}")
    # Calculate initial EF position and RPY using FK
    ef_position, ef_angles_rpy = forward_kinematics(*angles)
    print(f"Initial End effector position: {ef_position}")
    print(f"Initial End effector angles (RPY): {ef_angles_rpy}")
    # Calculate initial T matrices for visualization
    _, _, T01_initial, T02_initial, T03_initial, T04_initial = forward_kinematics(*angles, return_intermediate_frames=True)
    # Set initial marker positions in PyBullet
    p.resetBasePositionAndOrientation(viz_markers['target'], ef_position, [0,0,0,1])
    p.resetBasePositionAndOrientation(viz_markers['actual_ef'], ef_position, [0,0,0,1]) # Start actual at target
except Exception as init_err:
    print(f"ERROR calculating initial state: {init_err}")
    print("Falling back to default initial state.")
    angles = [0.0] * 4
    ef_position = np.array([0.1, 0.0, 0.1]) # Default fallback position
    T01_initial, T02_initial, T03_initial, T04_initial = np.eye(4), np.eye(4), np.eye(4), np.eye(4)
# --- End Initial State Calculation ---

last_sent_target_steps = list(current_positions) 

# --- Debug Line IDs Initialization (Remains the same) --- 
debug_line_ids = {k: -1 for k in ['x1','y1','z1', 'x2','y2','z2', 'x3','y3','z3', 'x4','y4','z4']}

# --- Keyboard Setup --- 
# ... (variables, listener setup) ...

# --- Keyboard Listener and Control Variables --- 
step_size = 0.04 # <-- Increased target speed further
move_direction = {'x': 0, 'y': 0, 'z': 0}
wrist_roll_direction = 0
discrete_wrist_step_increment = 120 # How much target changes per keypress cycle
gripper_direction = 0
discrete_gripper_step_increment = 120 # How much target changes per keypress cycle
torque_enabled = True 
toggle_torque_requested = False

# --- Target State Variables (Updated by keys in main loop) ---
target_wrist_step = last_sent_target_steps[4] # Initialize with current state
target_gripper_step = last_sent_target_steps[5] # Initialize with current state

# --- Reinstate Keyboard Listener Functions --- 
def on_press(key):
    global move_direction, wrist_roll_direction, gripper_direction
    global toggle_torque_requested # Add flag
    try:
        # Movement Keys (Remapped)
        if key.char == 'w': move_direction['z'] = 1
        elif key.char == 's': move_direction['z'] = -1
        elif key.char == 'a': move_direction['y'] = 1
        elif key.char == 'd': move_direction['y'] = -1
        elif key.char == 'q': move_direction['x'] = 1
        elif key.char == 'e': move_direction['x'] = -1
        # --- Torque Toggle --- 
        elif key.char == 't':
            print(f"\n>>> TOGGLE torque requested (current: {'ENABLED' if torque_enabled else 'DISABLED'}) <<< ")
            toggle_torque_requested = True
        # --- End Torque --- 
    except AttributeError:
        # Arrow keys (unchanged)
        if key == keyboard.Key.up: wrist_roll_direction = 1
        elif key == keyboard.Key.down: wrist_roll_direction = -1
        elif key == keyboard.Key.left: gripper_direction = -1
        elif key == keyboard.Key.right: gripper_direction = 1

def on_release(key):
    global move_direction, wrist_roll_direction, gripper_direction
    try:
        # --- Remapped W/S and Q/E ---
        if key.char in ['q', 'e']: # Q/E control X
            move_direction['x'] = 0
        elif key.char in ['a', 'd']: # A/D control Y
            move_direction['y'] = 0
        elif key.char in ['w', 's']: # W/S control Z
            move_direction['z'] = 0
        # --- End Remapping ---
    except AttributeError:
        # Reset arrow key directions on release (unchanged)
        if key in [keyboard.Key.up, keyboard.Key.down]:
            wrist_roll_direction = 0
        elif key in [keyboard.Key.left, keyboard.Key.right]:
            gripper_direction = 0
# --- End Reinstate Functions ---

# --- Reinstate Listener Initialization --- 
listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()
# --- End Reinstate Initialization ---

max_step_change = 500 # Threshold for IK safety check (Now disabled below)

try:
    target_frequency = 40 
    target_interval = 1.0 / target_frequency
    last_command_time = time.monotonic() 
    last_measured_positions = list(current_positions[:])
    # T matrices are now initialized above
    T01_current, T02_current, T03_current, T04_current = T01_initial, T02_initial, T03_initial, T04_initial

    while True:
        loop_start_time = time.monotonic()
        current_time = time.monotonic()

        # --- Process Torque Toggle Flag --- 
        torque_proc_start = time.monotonic()
        if toggle_torque_requested:
            torque_enabled = not torque_enabled # Toggle state first
            mode = TorqueMode.ENABLED.value if torque_enabled else TorqueMode.DISABLED.value
            status_str = "ENABLED" if torque_enabled else "DISABLED"
            print(f"\nSetting torque to {status_str}...", end='')
            try:
                follower_arm.write("Torque_Enable", mode) # Apply to all
                print(" Done.")
                if not torque_enabled:
                    print("--- MANUAL MODE ACTIVE: Move arm physically --- ")
                else:
                    print("--- TORQUE ENABLED --- ")
            except Exception as e:
                print(f" Error: {e}")
            toggle_torque_requested = False # Reset flag
        torque_proc_dur = time.monotonic() - torque_proc_start

        # --- Read Actual Position --- 
        read_start = time.monotonic()
        read_error = False
        try:
            current_read = follower_arm.read("Present_Position")
            if len(current_read) == NUM_MOTORS:
                last_measured_positions = list(current_read) 
            else: read_error = True
        except Exception as read_err: read_error = True
        read_dur = time.monotonic() - read_start

        # --- Remove the Debug Line Removal Loop --- 
        # debug_lines_remove_start = time.monotonic()
        # active_line_ids = [id for id in debug_line_ids.values() if id != -1]
        # for line_id in active_line_ids:
        #     try: p.removeUserDebugItem(line_id)
        #     except: pass 
        # debug_line_ids = {k: -1 for k in debug_line_ids}
        debug_lines_remove_dur = 0 # Set duration to 0
        
        # --- Update PyBullet Visualization (Robot Part) --- 
        viz_start = time.monotonic()
        if not read_error and len(last_measured_positions) == NUM_MOTORS:
            # Update Robot Model 
            try:
                current_radians_viz = steps_to_radians_for_viz(last_measured_positions, full_calibration)
                motor_names = list(follower_motors.keys())
                for i, motor_name in enumerate(motor_names):
                    if motor_name in p_joint_indices:
                        joint_index = p_joint_indices[motor_name]
                        p.resetJointState(robotId, joint_index, targetValue=current_radians_viz[i])
            except Exception as e:
                print(f"Error updating viz joints: {e}")

            # Calculate FK & Update Markers/Axes 
            try:
                measured_angles_deg = get_degrees_from_steps(last_measured_positions, 4)
                actual_ef_position, _, T01_current, T02_current, T03_current, T04_current = forward_kinematics(*measured_angles_deg, return_intermediate_frames=True)
                p.resetBasePositionAndOrientation(viz_markers['actual_ef'], actual_ef_position, [0,0,0,1])
                
                axis_length = 0.05
                # --- Update/Create Frame 1 Axes (RGB) using replaceItemUniqueId --- 
                origin1 = T01_current[:3, 3]; rot_matrix1 = T01_current[:3, :3]
                debug_line_ids['x1'] = p.addUserDebugLine(origin1, origin1 + rot_matrix1[:, 0] * axis_length, [1, 0, 0], lineWidth=2, replaceItemUniqueId=debug_line_ids['x1'])
                debug_line_ids['y1'] = p.addUserDebugLine(origin1, origin1 + rot_matrix1[:, 1] * axis_length, [0, 1, 0], lineWidth=2, replaceItemUniqueId=debug_line_ids['y1'])
                debug_line_ids['z1'] = p.addUserDebugLine(origin1, origin1 + rot_matrix1[:, 2] * axis_length, [0, 0, 1], lineWidth=2, replaceItemUniqueId=debug_line_ids['z1'])
                # --- Update/Create Frame 2 Axes (RGB) --- 
                origin2 = T02_current[:3, 3]; rot_matrix2 = T02_current[:3, :3]
                debug_line_ids['x2'] = p.addUserDebugLine(origin2, origin2 + rot_matrix2[:, 0] * axis_length, [1, 0, 0], lineWidth=2, replaceItemUniqueId=debug_line_ids['x2'])
                debug_line_ids['y2'] = p.addUserDebugLine(origin2, origin2 + rot_matrix2[:, 1] * axis_length, [0, 1, 0], lineWidth=2, replaceItemUniqueId=debug_line_ids['y2'])
                debug_line_ids['z2'] = p.addUserDebugLine(origin2, origin2 + rot_matrix2[:, 2] * axis_length, [0, 0, 1], lineWidth=2, replaceItemUniqueId=debug_line_ids['z2'])
                # --- Update/Create Frame 3 Axes (RGB) --- 
                origin3 = T03_current[:3, 3]; rot_matrix3 = T03_current[:3, :3]
                debug_line_ids['x3'] = p.addUserDebugLine(origin3, origin3 + rot_matrix3[:, 0] * axis_length, [1, 0, 0], lineWidth=2, replaceItemUniqueId=debug_line_ids['x3'])
                debug_line_ids['y3'] = p.addUserDebugLine(origin3, origin3 + rot_matrix3[:, 1] * axis_length, [0, 1, 0], lineWidth=2, replaceItemUniqueId=debug_line_ids['y3'])
                debug_line_ids['z3'] = p.addUserDebugLine(origin3, origin3 + rot_matrix3[:, 2] * axis_length, [0, 0, 1], lineWidth=2, replaceItemUniqueId=debug_line_ids['z3'])
                # --- Update/Create Frame 4 Axes (CMY) --- 
                origin4 = T04_current[:3, 3]; rot_matrix4 = T04_current[:3, :3]
                debug_line_ids['x4'] = p.addUserDebugLine(origin4, origin4 + rot_matrix4[:, 0] * axis_length, [0, 1, 1], lineWidth=2, replaceItemUniqueId=debug_line_ids['x4'])
                debug_line_ids['y4'] = p.addUserDebugLine(origin4, origin4 + rot_matrix4[:, 1] * axis_length, [1, 0, 1], lineWidth=2, replaceItemUniqueId=debug_line_ids['y4'])
                debug_line_ids['z4'] = p.addUserDebugLine(origin4, origin4 + rot_matrix4[:, 2] * axis_length, [1, 1, 0], lineWidth=2, replaceItemUniqueId=debug_line_ids['z4'])

            except Exception as fk_err:
                print(f"Error during FK/Axes Viz: {fk_err}") 
        viz_dur = time.monotonic() - viz_start
        # --- End PyBullet Update --- 
        
        # --- Update Target State Based on Keyboard Input ---
        target_update_start = time.monotonic()
        # Update EF position target
        delta_x = move_direction['x'] * step_size
        delta_y = move_direction['y'] * step_size
        delta_z = move_direction['z'] * step_size
        ef_position[0] += delta_x
        ef_position[1] += delta_y
        ef_position[2] += delta_z
        # Update Wrist/Gripper step targets based on direction flags
        if wrist_roll_direction != 0:
            delta_wrist = wrist_roll_direction * discrete_wrist_step_increment
            target_wrist_step = (target_wrist_step + delta_wrist) % 4096
        if gripper_direction != 0:
            delta_gripper = gripper_direction * discrete_gripper_step_increment
            target_gripper_step = (target_gripper_step + delta_gripper) % 4096
        target_update_dur = time.monotonic() - target_update_start
        
        # --- Update Target Position Marker (Red) --- 
        marker_update_start = time.monotonic()
        p.resetBasePositionAndOrientation(viz_markers['target'], ef_position, [0,0,0,1])
        marker_update_dur = time.monotonic() - marker_update_start

        # --- Control Logic (Gated by Time Interval) --- 
        control_logic_dur = 0
        control_block_executed = False
        ik_dur, fk_dur, safety_check_dur, write_dur = 0, 0, 0, 0

        if torque_enabled and (current_time - last_command_time >= target_interval):
            control_logic_start = time.monotonic()
            control_block_executed = True
            
            # --- IK calculation --- 
            ik_start_time = time.monotonic()
            initial_guess_for_ik = angles 
            _, current_fk_ef_angles = forward_kinematics(*initial_guess_for_ik) 
            current_pitch = current_fk_ef_angles[1] 
            updated_angles = iterative_ik(ef_position, current_pitch, initial_guess_for_ik) 
            ik_dur = time.monotonic() - ik_start_time 
            
            # --- FK Calculation --- 
            fk_start_time = time.monotonic()
            final_pos, _ = forward_kinematics(*updated_angles) 
            fk_dur = time.monotonic() - fk_start_time 

            # --- Convert to final target steps --- 
            target_base_steps = angles_to_servo_steps(updated_angles)
            final_target_steps = [0] * NUM_MOTORS
            final_target_steps[0] = target_base_steps[0]
            final_target_steps[1] = target_base_steps[1]
            final_target_steps[2] = target_base_steps[2]
            final_target_steps[3] = target_base_steps[3]
            final_target_steps[4] = target_wrist_step 
            final_target_steps[5] = target_gripper_step 
            
            # --- Temporarily Disable Safety Check --- 
            safety_check_start = time.monotonic()
            jump_detected = False # Assume safe for now
            # for i in range(4): # Only check base joints
            #     diff = final_target_steps[i] - last_sent_target_steps[i]
            #     if diff > 2048: diff -= 4096
            #     elif diff < -2048: diff += 4096
            #     # Use original max_step_change threshold
            #     if abs(diff) > max_step_change: 
            #         print(f"IK jump detected on joint {i}: LastSent={last_sent_target_steps[i]}, NewTarget={final_target_steps[i]}")
            #         jump_detected = True
            #         break
            safety_check_dur = time.monotonic() - safety_check_start # Still measure dummy time

            # --- Write Goal_Position (Always attempt now) --- 
            write_start_time = time.monotonic()
            # if not jump_detected: # Removed condition
            follower_arm.write("Goal_Position", np.array(final_target_steps))
            angles = updated_angles[:] # Store angles corresponding to the sent command
            last_sent_target_steps = final_target_steps[:] # Update last sent steps
            # else: Jump detected, DO NOT send command or update angles/last_sent
            write_dur = time.monotonic() - write_start_time 
            
            last_command_time = current_time 
            control_logic_dur = time.monotonic() - control_logic_start 
            
        # --- Loop Timing Output ---
        loop_dur = time.monotonic() - loop_start_time
        if control_block_executed or (loop_start_time % 1.0 < 0.05):
             timing_str = f"Loop: {loop_dur:.4f}s | Read: {read_dur:.4f}s | Viz: {viz_dur:.4f}s | TgtUpd: {target_update_dur:.4f}s | MkrUpd: {marker_update_dur:.4f}s | TqChk: {torque_proc_dur:.4f}s"
             if control_block_executed:
                 timing_str += f" | Ctrl(IK:{ik_dur:.4f} FK:{fk_dur:.4f} Safety:{safety_check_dur:.4f} Write:{write_dur:.4f}) Total:{control_logic_dur:.4f}s"
             else:
                  timing_str += " | CtrlLogic: Not Executed"
             print(timing_str)

        # Small sleep
        time.sleep(1./240.) 

except KeyboardInterrupt:
    print("\nTeleoperation ended.")
except RuntimeError as e:
    print(str(e))
finally:
    # Cleanup
    print("Cleaning up...")
    if listener and listener.is_alive():
        listener.stop()
        print("Keyboard listener stopped.")
    if follower_arm and follower_arm.is_connected:
        try:
            # Attempt to disable torque before disconnecting
            print("Attempting to disable torque...")
            all_motor_ids = [m[0] for m in follower_motors.values()]
            follower_arm.write("Torque_Enable", TorqueMode.DISABLED.value)
            time.sleep(0.5)
        except Exception as e_torque:
            print(f"Could not disable torque on exit: {e_torque}")
        finally:
             follower_arm.disconnect()
             print("Follower arm disconnected.")
    if physicsClient >= 0:
        p.disconnect()
        print("PyBullet disconnected.")
    print("Cleanup finished.")
