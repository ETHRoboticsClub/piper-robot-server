import numpy as np
from forward_kinematics import forward_kinematics
from inverse_kinematics import iterative_ik
from lerobot.common.robot_devices.motors.feetech import FeetechMotorsBus, TorqueMode
import time
from pynput import keyboard

def servo_steps_to_angles(steps):
    if len(steps) != 4:
        raise ValueError("Expected 4 steps for main joints.")
    calibration = [
        {"zero_step": 2047, "direction": -1},
        {"zero_step": 54,   "direction": 1},
        {"zero_step": 25,   "direction": 1},
        {"zero_step": 2095, "direction": 1},
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
        {"zero_step": 2047, "direction": -1},
        {"zero_step": 54,   "direction": 1},
        {"zero_step": 25,   "direction": 1},
        {"zero_step": 2095, "direction": 1},
    ]
    steps_per_degree = 4096 / 360.0
    step_values = []
    for i, angle in enumerate(angles):
        zero_step = calibration[i]["zero_step"]
        direction = calibration[i]["direction"]
        step_value = int(zero_step + direction * angle * steps_per_degree)
        step_values.append(step_value % 4096)
    return step_values

follower_port = "/dev/tty.usbmodem58CD1774031"
follower_arm = FeetechMotorsBus(
    port=follower_port,
    motors={
        "shoulder_pan": (6, "sts3215"),
        "shoulder_lift": (5, "sts3215"),
        "elbow_flex": (4, "sts3215"),
        "wrist_flex": (3, "sts3215"),
        "wrist_roll": (2, "sts3215"),
        "gripper": (1, "sts3215"),
    },
)

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



step_size = 0.005
movement = {'x': 0, 'y': 0, 'z': 0}

# For teleoperation of wrist_roll (index 4) and gripper (index 5)
# We'll treat them like continuous increments, similar to x,y,z
wrist_roll_movement = 0
gripper_movement = 0
teleop_increment = 50  # Smaller increments per loop

def on_press(key):
    global wrist_roll_movement, gripper_movement
    try:
        if key.char == 'w':
            movement['x'] = step_size
        elif key.char == 's':
            movement['x'] = -step_size
        elif key.char == 'a':
            movement['y'] = step_size
        elif key.char == 'd':
            movement['y'] = -step_size
        elif key.char == 'q':
            movement['z'] = step_size
        elif key.char == 'e':
            movement['z'] = -step_size
    except AttributeError:
        # Handle arrow keys
        if key == keyboard.Key.up:
            wrist_roll_movement = teleop_increment
        elif key == keyboard.Key.down:
            wrist_roll_movement = -teleop_increment
        elif key == keyboard.Key.left:
            gripper_movement = -teleop_increment
        elif key == keyboard.Key.right:
            gripper_movement = teleop_increment

def on_release(key):
    global wrist_roll_movement, gripper_movement
    try:
        if key.char in ['w', 's']:
            movement['x'] = 0
        elif key.char in ['a', 'd']:
            movement['y'] = 0
        elif key.char in ['q', 'e']:
            movement['z'] = 0
    except AttributeError:
        # Reset arrow key movements on release
        if key in [keyboard.Key.up, keyboard.Key.down]:
            wrist_roll_movement = 0
        elif key in [keyboard.Key.left, keyboard.Key.right]:
            gripper_movement = 0

listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

max_step_change = 500
wrist_roll_offset = current_positions[4]
gripper_offset = current_positions[5]

try:
    while True:
        # Update end effector position
        ef_position[0] += movement['x']
        ef_position[1] += movement['y']
        ef_position[2] += movement['z']

        # Update wrist_roll_offset and gripper_offset from movements
        wrist_roll_offset = (wrist_roll_offset + wrist_roll_movement) % 4096
        gripper_offset = (gripper_offset + gripper_movement) % 4096

        print(f"Updated EF position: {ef_position}")

        updated_angles = iterative_ik(ef_position, 90, angles)
        print(f"Updated angles: {updated_angles}")

        final_pos, _ = forward_kinematics(*updated_angles)
        final_error = ef_position - final_pos
        print("Final position error:", final_error, "Norm:", np.linalg.norm(final_error))

        updated_steps = angles_to_servo_steps(updated_angles)
        print(f"Updated servo steps (main joints): {updated_steps}")

        current_positions = follower_arm.read("Present_Position")
        current_motors = current_positions[0:4]
        

        for c, u in zip(current_motors, updated_steps):
            if abs(u - c) > max_step_change:
                print("jump detected")
                raise RuntimeError(f"Sudden large jump detected: Current={c}, Target={u}. Stopping.")

        # Append the updated wrist_roll and gripper
        updated_steps.append(wrist_roll_offset)
        updated_steps.append(gripper_offset)

        follower_arm.write("Goal_Position", np.array(updated_steps))
        angles = updated_angles[:]
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Teleoperation ended.")
except RuntimeError as e:
    print(str(e))
finally:
    listener.stop()
