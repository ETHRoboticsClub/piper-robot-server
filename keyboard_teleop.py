# --- Keyboard Teleoperation ---
# This script allows for teleoperation of the robot using the keyboard
# Modified from rabhishek100's uploaded code on https://github.com/huggingface/lerobot/issues/568
import numpy as np
from forward_kinematics import forward_kinematics
from inverse_kinematics import iterative_ik
from lerobot.common.robot_devices.motors.feetech import FeetechMotorsBus, TorqueMode, FeetechMotorsBusConfig
import time
from pynput import keyboard

def servo_steps_to_angles(steps):
    if len(steps) != 4:
        raise ValueError("Expected 4 steps for main joints.")
    calibration = [
        {"zero_step": 2048, "direction": -1},
        {"zero_step": 2052, "direction": 1},
        {"zero_step": 2047, "direction": 1},
        {"zero_step": 2047, "direction": 1},
    ]
    degrees_per_step = 360.0 / 4096.0
    angle_values = []
    for i, step in enumerate(steps):
        zero_step = calibration[i]["zero_step"]
        direction = calibration[i]["direction"]
        angle_value = (step - zero_step) * direction * degrees_per_step
        angle_values.append(angle_value % 360)
    return angle_values

def angles_to_servo_steps(angles):
    if len(angles) != 4:
        raise ValueError("Expected 4 angles for main joints.")
    calibration = [
        {"zero_step": 2048, "direction": -1},
        {"zero_step": 2052, "direction": 1},
        {"zero_step": 2047, "direction": 1},
        {"zero_step": 2047, "direction": 1},
    ]
    steps_per_degree = 4096 / 360.0
    step_values = []
    for i, angle in enumerate(angles):
        zero_step = calibration[i]["zero_step"]
        direction = calibration[i]["direction"]
        step_value = int(zero_step + direction * angle * steps_per_degree)
        step_values.append(step_value % 4096)
    return step_values

follower_port = "/dev/ttyACM0"
follower_motors = {
    "shoulder_pan": [1, "sts3215"],
    "shoulder_lift": [2, "sts3215"],
    "elbow_flex": [3, "sts3215"],
    "wrist_flex": [4, "sts3215"],
    "wrist_roll": [5, "sts3215"],
    "gripper": [6, "sts3215"],
}
follower_config = FeetechMotorsBusConfig(port=follower_port, motors=follower_motors)
follower_arm = FeetechMotorsBus(follower_config)

follower_arm.connect()
follower_arm.write("Torque_Enable", TorqueMode.ENABLED.value)
current_positions = follower_arm.read("Present_Position")
time.sleep(2)

positions = current_positions[0:4]
print(f"Arm start positions: {positions}")
angles = servo_steps_to_angles(positions)
print(f"Arm start angles: {angles}")
ef_position, ef_angles = forward_kinematics(*angles)
print(f"End effector start position: {ef_position}")
print(f"End effector start angles: {ef_angles}")

step_size = 0.002 # Increased slightly for speed
move_direction = {'x': 0, 'y': 0, 'z': 0} # Store direction (-1, 0, 1)

# For teleoperation of wrist_roll (index 4) and gripper (index 5)
wrist_roll_direction = 0 # Store direction (-1, 0, 1)
gripper_direction = 0  # Store direction (-1, 0, 1)
discrete_step_increment = 20 # Increased slightly for speed

def on_press(key):
    global move_direction, wrist_roll_direction, gripper_direction
    try:
        if key.char == 'w':
            move_direction['x'] = 1
        elif key.char == 's':
            move_direction['x'] = -1
        elif key.char == 'a':
            move_direction['y'] = 1
        elif key.char == 'd':
            move_direction['y'] = -1
        elif key.char == 'q':
            move_direction['z'] = 1
        elif key.char == 'e':
            move_direction['z'] = -1
    except AttributeError:
        # Handle arrow keys
        if key == keyboard.Key.up:
            wrist_roll_direction = 1 # Corresponds to positive increment
        elif key == keyboard.Key.down:
            wrist_roll_direction = -1 # Corresponds to negative increment
        elif key == keyboard.Key.left:
            gripper_direction = -1 # Corresponds to negative increment (e.g., open)
        elif key == keyboard.Key.right:
            gripper_direction = 1 # Corresponds to positive increment (e.g., close)

def on_release(key):
    global move_direction, wrist_roll_direction, gripper_direction
    try:
        if key.char in ['w', 's']:
            move_direction['x'] = 0
        elif key.char in ['a', 'd']:
            move_direction['y'] = 0
        elif key.char in ['q', 'e']:
            move_direction['z'] = 0
    except AttributeError:
        # Reset arrow key directions on release
        if key in [keyboard.Key.up, keyboard.Key.down]:
            wrist_roll_direction = 0
        elif key in [keyboard.Key.left, keyboard.Key.right]:
            gripper_direction = 0

listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

max_step_change = 500
# Initialize offsets from the actual read positions
wrist_roll_offset = current_positions[4]
gripper_offset = current_positions[5]

try:
    target_frequency = 20 # Hz
    target_interval = 1.0 / target_frequency
    last_command_time = time.monotonic() # Initialize before loop

    while True:
        # Loop runs as fast as possible, checking time condition
        current_time = time.monotonic()

        # --- Always process inputs (implicitly handled by listener thread) ---
        # The direction flags (move_direction, etc.) should be up-to-date here

        if current_time - last_command_time >= target_interval:
            # --- Time interval elapsed, proceed with control logic --- 

            # Calculate actual change based on current direction flags
            delta_x = move_direction['x'] * step_size
            delta_y = move_direction['y'] * step_size
            delta_z = move_direction['z'] * step_size

            delta_wrist = wrist_roll_direction * discrete_step_increment
            delta_gripper = gripper_direction * discrete_step_increment

            # Update target end effector position
            ef_position[0] += delta_x
            ef_position[1] += delta_y
            ef_position[2] += delta_z

            # Update target wrist_roll_offset and gripper_offset
            # Ensure wrap around 4096
            wrist_roll_offset = (wrist_roll_offset + delta_wrist) % 4096
            gripper_offset = (gripper_offset + delta_gripper) % 4096

            print(f"Target EF position: {ef_position}") # Renamed print for clarity

            # Use the current pitch angle from ef_angles as the target for IK
            current_pitch = ef_angles[1]
            updated_angles = iterative_ik(ef_position, current_pitch, angles)
            print(f"Calculated angles: {updated_angles}") # Renamed print

            # Update ef_angles for the next iteration based on the new FK result
            final_pos, ef_angles = forward_kinematics(*updated_angles)
            final_error = ef_position - final_pos
            print("Position error:", final_error, "Norm:", np.linalg.norm(final_error)) # Renamed print

            updated_steps = angles_to_servo_steps(updated_angles)
            print(f"Target servo steps (main joints): {updated_steps}") # Renamed print

            # Read current positions *just before* the safety check
            current_positions_now = follower_arm.read("Present_Position")
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
                print("Stopping due to large jump detection.")
                raise RuntimeError("Sudden large jump detected. Stopping.")
            
            # If safety checks pass, send command and update state

            # Append the updated wrist_roll and gripper target offsets
            command_steps = updated_steps[:] # Create a copy
            command_steps.append(wrist_roll_offset)
            command_steps.append(gripper_offset)

            follower_arm.write("Goal_Position", np.array(command_steps))
            
            # Update the angles state *after* commanding, ready for the next loop's IK initial guess
            angles = updated_angles[:]

            # Update the time of the last command
            last_command_time = current_time
        
        # --- No sleep here, loop continues immediately ---

except KeyboardInterrupt:
    print("Teleoperation ended.")
except RuntimeError as e:
    print(str(e))
finally:
    listener.stop()
