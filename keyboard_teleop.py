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
follower_arm.write("Torque_Enable", TorqueMode.ENABLED.value)
current_positions = follower_arm.read("Present_Position")
full_calibration = get_full_calibration() # Get full calibration data
time.sleep(1) # Reduced sleep after enabling torque
# --- End Real Robot Connection --- 

# --- Initial State Calculation --- 
positions = current_positions[0:4]
print(f"Arm start positions (steps 0-3): {positions}")
angles = servo_steps_to_angles(positions)
print(f"Arm start angles (deg 0-3): {angles}")
ef_position, ef_angles = forward_kinematics(*angles)
print(f"End effector start position: {ef_position}")
print(f"End effector start angles: {ef_angles}")
# --- Calculate initial frame poses --- 
try:
    angles_init_deg = get_degrees_from_steps(current_positions, 4)
    # Call FK to get initial actual pos and full T01, T02, T03, T04 matrices
    actual_ef_position, _, T01_initial, T02_initial, T03_initial, T04_initial = forward_kinematics(*angles_init_deg, return_intermediate_frames=True)
    # Update initial marker positions
    p.resetBasePositionAndOrientation(viz_markers['target'], ef_position, [0,0,0,1])
    p.resetBasePositionAndOrientation(viz_markers['actual_ef'], actual_ef_position, [0,0,0,1])
    # Store initial matrices for drawing axes in the loop
except Exception as init_fk_err:
    print(f"Error calculating initial FK poses: {init_fk_err}")
    actual_ef_position = ef_position.copy() # Fallback
    T01_initial = np.eye(4) # Fallback T01
    T02_initial = np.eye(4) # Fallback T02
    T03_initial = np.eye(4) # Fallback T03
    T04_initial = np.eye(4) # Fallback T04

final_pos = ef_position.copy() 
# --- End Initial State Calculation ---

# --- Keyboard Listener and Control Variables --- 
step_size = 0.01 # Reduced step size again to prevent overshoots
move_direction = {'x': 0, 'y': 0, 'z': 0}
wrist_roll_direction = 0
gripper_direction = 0
discrete_step_increment = 120
torque_enabled = True # Add torque state
toggle_torque_requested = False # Add flag

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

listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()
# --- End Keyboard Listener --- 

max_step_change = 500
# Initialize offsets from the actual read positions (using all 6)
wrist_roll_offset = current_positions[4]
gripper_offset = current_positions[5]

# --- Main Control Loop --- 
try:
    target_frequency = 20 # Hz
    target_interval = 1.0 / target_frequency
    last_command_time = time.monotonic() # Initialize before loop
    # Store the latest full measured position for visualization
    last_measured_positions = current_positions[:] 
    T01_current = T01_initial # Initialize with the starting pose
    T02_current = T02_initial # Initialize T02 as well
    T03_current = T03_initial # Initialize T03
    T04_current = T04_initial # Initialize T04
    debug_line_ids = [] # Keep track of drawn lines

    while True:
        current_time = time.monotonic()

        # --- Process Torque Toggle Flag --- 
        if toggle_torque_requested:
            torque_enabled = not torque_enabled # Toggle state first
            mode = TorqueMode.ENABLED.value if torque_enabled else TorqueMode.DISABLED.value
            status_str = "ENABLED" if torque_enabled else "DISABLED"
            print(f"\nSetting torque to {status_str}...", end='')
            try:
                all_motor_ids = [m[0] for m in follower_motors.values()] # Get IDs
                follower_arm.write("Torque_Enable", mode) # Apply to all
                print(" Done.")
                if not torque_enabled:
                    print("--- MANUAL MODE ACTIVE: Move arm physically --- ")
                else:
                    print("--- TORQUE ENABLED --- ")
            except Exception as e:
                print(f" Error: {e}")
            toggle_torque_requested = False # Reset flag
        # --- End Torque Processing --- 

        # --- Read Actual Position --- 
        read_error = False
        try:
            current_read = follower_arm.read("Present_Position")
            if len(current_read) == NUM_MOTORS:
                last_measured_positions = list(current_read) 
            else: read_error = True
        except Exception as read_err: read_error = True

        # --- Remove previous debug lines --- 
        for line_id in debug_line_ids:
            p.removeUserDebugItem(line_id)
        debug_line_ids.clear()
        # --- 

        # --- Update PyBullet Visualization --- 
        if len(last_measured_positions) == NUM_MOTORS:
            # --- Update Robot Model --- 
            current_radians_viz = steps_to_radians_for_viz(last_measured_positions, full_calibration)
            motor_names = list(follower_motors.keys())
            for i in range(len(motor_names)):
                motor_name = motor_names[i]
                if motor_name in p_joint_indices:
                    joint_index = p_joint_indices[motor_name]
                    p.resetJointState(robotId, joint_index, targetValue=current_radians_viz[i])
            
            # --- Calculate FK for Measured State & Update Markers/Axes --- 
            try:
                measured_angles_deg = get_degrees_from_steps(last_measured_positions, 4)
                print(f"DEBUG: Measured Angles (deg): {measured_angles_deg}") # Print calculated angles
                # Get actual EF pos and full T01, T02, T03, T04 matrices based on measurement
                actual_ef_position, _, T01_current, T02_current, T03_current, T04_current = forward_kinematics(*measured_angles_deg, return_intermediate_frames=True)
                
                # Update Actual EF (Blue) marker
                p.resetBasePositionAndOrientation(viz_markers['actual_ef'], actual_ef_position, [0,0,0,1])
                
                axis_length = 0.05 # Length of axes lines

                # --- Draw Frame 1 Axes (RGB) --- 
                origin1 = T01_current[:3, 3]
                rot_matrix1 = T01_current[:3, :3]
                x_axis_end1 = origin1 + rot_matrix1[:, 0] * axis_length
                y_axis_end1 = origin1 + rot_matrix1[:, 1] * axis_length
                z_axis_end1 = origin1 + rot_matrix1[:, 2] * axis_length
                
                line_id_x1 = p.addUserDebugLine(origin1, x_axis_end1, [1, 0, 0], lineWidth=2) # Red X
                line_id_y1 = p.addUserDebugLine(origin1, y_axis_end1, [0, 1, 0], lineWidth=2) # Green Y
                line_id_z1 = p.addUserDebugLine(origin1, z_axis_end1, [0, 0, 1], lineWidth=2) # Blue Z
                debug_line_ids.extend([line_id_x1, line_id_y1, line_id_z1])

                # --- Draw Frame 2 Axes (RGB) --- 
                origin2 = T02_current[:3, 3]
                rot_matrix2 = T02_current[:3, :3]
                x_axis_end2 = origin2 + rot_matrix2[:, 0] * axis_length
                y_axis_end2 = origin2 + rot_matrix2[:, 1] * axis_length
                z_axis_end2 = origin2 + rot_matrix2[:, 2] * axis_length
                
                # Change Frame 2 to RGB
                line_id_x2 = p.addUserDebugLine(origin2, x_axis_end2, [1, 0, 0], lineWidth=2) # Red X
                line_id_y2 = p.addUserDebugLine(origin2, y_axis_end2, [0, 1, 0], lineWidth=2) # Green Y
                line_id_z2 = p.addUserDebugLine(origin2, z_axis_end2, [0, 0, 1], lineWidth=2) # Blue Z
                debug_line_ids.extend([line_id_x2, line_id_y2, line_id_z2])

                # --- Draw Frame 3 Axes (RGB) --- 
                origin3 = T03_current[:3, 3]
                rot_matrix3 = T03_current[:3, :3]
                x_axis_end3 = origin3 + rot_matrix3[:, 0] * axis_length
                y_axis_end3 = origin3 + rot_matrix3[:, 1] * axis_length
                z_axis_end3 = origin3 + rot_matrix3[:, 2] * axis_length
                
                # Change Frame 3 to RGB
                line_id_x3 = p.addUserDebugLine(origin3, x_axis_end3, [1, 0, 0], lineWidth=2) # Red X
                line_id_y3 = p.addUserDebugLine(origin3, y_axis_end3, [0, 1, 0], lineWidth=2) # Green Y
                line_id_z3 = p.addUserDebugLine(origin3, z_axis_end3, [0, 0, 1], lineWidth=2) # Blue Z
                debug_line_ids.extend([line_id_x3, line_id_y3, line_id_z3])

                # --- Draw Frame 4 Axes (CMY) --- 
                origin4 = T04_current[:3, 3]
                rot_matrix4 = T04_current[:3, :3]
                x_axis_end4 = origin4 + rot_matrix4[:, 0] * axis_length
                y_axis_end4 = origin4 + rot_matrix4[:, 1] * axis_length
                z_axis_end4 = origin4 + rot_matrix4[:, 2] * axis_length
                
                line_id_x4 = p.addUserDebugLine(origin4, x_axis_end4, [0, 1, 1], lineWidth=2) # Cyan X
                line_id_y4 = p.addUserDebugLine(origin4, y_axis_end4, [1, 0, 1], lineWidth=2) # Magenta Y
                line_id_z4 = p.addUserDebugLine(origin4, z_axis_end4, [1, 1, 0], lineWidth=2) # Yellow Z
                debug_line_ids.extend([line_id_x4, line_id_y4, line_id_z4])

                # --- End Draw Axes ---
            except Exception as fk_err:
                 print(f"Error in FK calculation/drawing: {fk_err}") # Print error
                 pass

        # --- Update Target Position Marker (Red) --- 
        p.resetBasePositionAndOrientation(viz_markers['target'], ef_position, [0,0,0,1])
        # --- End PyBullet Update --- 

        # --- Control Logic (Gated by Time Interval) --- 
        # Only run if torque is enabled
        if torque_enabled and (current_time - last_command_time >= target_interval):
            # Calculate potential change based on current direction flags
            delta_x = move_direction['x'] * step_size
            delta_y = move_direction['y'] * step_size
            delta_z = move_direction['z'] * step_size
            delta_wrist = wrist_roll_direction * discrete_step_increment
            delta_gripper = gripper_direction * discrete_step_increment

            # --- Check if any movement is requested --- 
            is_moving = delta_x != 0 or delta_y != 0 or delta_z != 0 or \
                        delta_wrist != 0 or delta_gripper != 0

            if is_moving:
                # --- Movement Requested: Proceed with control logic --- 
                block_start_time = time.monotonic() # Time the start of the block

                # Update target end effector position
                ef_position[0] += delta_x
                ef_position[1] += delta_y
                ef_position[2] += delta_z

                # Update target wrist_roll_offset and gripper_offset
                wrist_roll_offset = (wrist_roll_offset + delta_wrist) % 4096
                gripper_offset = (gripper_offset + delta_gripper) % 4096

                print(f"Target EF position: {ef_position}")

                # --- Time the IK calculation --- 
                ik_start_time = time.monotonic()
                # IK needs initial guess (`angles`). This variable should represent angles 
                # consistent with the FK/IK internal model. Let's calculate it using the FK conversion.
                current_angles_for_ik = get_degrees_from_steps(last_measured_positions, 4)
                # Pitch angle target should also be relative to FK's frame
                _, current_fk_ef_angles = forward_kinematics(*current_angles_for_ik)
                current_pitch = current_fk_ef_angles[1] 
                
                updated_angles = iterative_ik(ef_position, current_pitch, current_angles_for_ik)
                
                # --- FK Calculation (uses default return for control loop) --- 
                # Calculate where the commanded angles *should* put the end effector
                final_pos, ef_angles = forward_kinematics(*updated_angles) 
                
                final_error = ef_position - final_pos # Error between target and FK result
                print("Position error:", final_error, "Norm:", np.linalg.norm(final_error))

                updated_steps = angles_to_servo_steps(updated_angles)
                print(f"Target servo steps (main joints): {updated_steps}")

                # Read current positions *just before* the safety check
                read_start_time = time.monotonic()
                current_positions_now = follower_arm.read("Present_Position")
                read_duration = time.monotonic() - read_start_time
                current_motors_now = current_positions_now[0:4]
                
                # Safety check against actual current position
                jump_detected = False
                for i, (c, u) in enumerate(zip(current_motors_now, updated_steps)):
                    current_wrist = current_positions_now[4]
                    current_gripper = current_positions_now[5]
                    target_wrist = wrist_roll_offset
                    target_gripper = gripper_offset
                    if abs(u - c) > max_step_change:
                        print(f"Large jump detected on main joint {i}: Current={c}, Target={u}")
                        jump_detected = True
                        break
                if not jump_detected:
                    if abs(target_wrist - current_wrist) > max_step_change:
                         diff = abs(target_wrist - current_wrist)
                         if diff > 2048: diff = 4096 - diff
                         if diff > max_step_change:
                            print(f"Large jump detected on wrist_roll: Current={current_wrist}, Target={target_wrist}")
                            jump_detected = True
                if not jump_detected:
                     if abs(target_gripper - current_gripper) > max_step_change:
                         diff = abs(target_gripper - current_gripper)
                         if diff > 2048: diff = 4096 - diff
                         if diff > max_step_change:
                            print(f"Large jump detected on gripper: Current={current_gripper}, Target={target_gripper}")
                            jump_detected = True
                            
                if jump_detected:
                    print(f"(Read took {read_duration:.4f}s before jump)")
                    raise RuntimeError("Sudden large jump detected. Stopping.")
                
                # If safety checks pass, send command and update state
                command_steps = updated_steps[:] 
                command_steps.append(wrist_roll_offset)
                command_steps.append(gripper_offset)

                # --- Time the write command --- 
                write_start_time = time.monotonic()
                follower_arm.write("Goal_Position", np.array(command_steps))
                write_duration = time.monotonic() - write_start_time
                # --- End write timing --- 
                
                # Update the angles state *only when moving*
                angles = updated_angles[:]

                # Update the time of the last command *sent*
                last_command_time = current_time

                # --- Print total block duration --- 
                block_duration = time.monotonic() - block_start_time
                print(f"(Read: {read_duration:.4f}s, Write: {write_duration:.4f}s, Total Block: {block_duration:.4f}s)")
            # else: No movement requested, do nothing, robot holds last position
            
        # Small sleep to allow PyBullet GUI to update
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
