import numpy as np
from lerobot.common.robot_devices.motors.feetech import FeetechMotorsBus, TorqueMode, FeetechMotorsBusConfig
import time
from pynput import keyboard
import sys
import json # Import json for saving

# --- Calibration and Motor Config (Copied from keyboard_teleop.py) ---
# Use the updated calibration values
calibration = [
    {"zero_step": 2048, "direction": -1}, # shoulder_pan (motor index 0 in list, ID 1)
    {"zero_step": 2052, "direction": 1},  # shoulder_lift (motor index 1 in list, ID 2)
    {"zero_step": 2047, "direction": 1},  # elbow_flex (motor index 2 in list, ID 3)
    {"zero_step": 2047, "direction": 1},  # wrist_flex (motor index 3 in list, ID 4)
    # Calibration only needed for the first 4 controllable via IK in the other script
]

follower_port = "/dev/ttyACM0" # Make sure this is correct
# --- Updated Motor IDs --- 
follower_motors = {
    "shoulder_pan": (1, "sts3215"),
    "shoulder_lift": (2, "sts3215"),
    "elbow_flex": (3, "sts3215"),
    "wrist_flex": (4, "sts3215"),
    "wrist_roll": (5, "sts3215"),
    "gripper": (6, "sts3215"),
}
follower_config = FeetechMotorsBusConfig(port=follower_port, motors=follower_motors)
# --- End Copied Section ---

# Mapping from motor index (0-3) used in the loop to motor name/ID for reading
motor_names_in_order = list(follower_motors.keys()) # Get ["shoulder_pan", "shoulder_lift", ...]

def servo_step_to_angle(step, motor_index):
    """Converts a single step value for a specific motor index (0-3) to an angle."""
    if not 0 <= motor_index < len(calibration):
        raise ValueError(f"Motor index {motor_index} out of range for calibration.")

    cal = calibration[motor_index]
    zero_step = cal["zero_step"]
    direction = cal["direction"]
    degrees_per_step = 360.0 / 4096.0

    # Calculate angle, handle potential wrap around carefully if needed,
    # but for limits, the raw difference from zero is usually fine.
    angle_value = (step - zero_step) * direction * degrees_per_step

    # Let's keep the angle potentially outside 0-360 for limits if that makes sense,
    # or wrap it? Wrapping might be confusing for limits. Let's not wrap for now.
    # angle_value = angle_value % 360
    return angle_value

def wait_for_spacebar():
    """Waits for the spacebar to be pressed."""
    pressed = False
    def on_press(key):
        nonlocal pressed
        if key == keyboard.Key.space:
            print("... Spacebar pressed.")
            pressed = True
            return False # Stop listener

    print("Press the spacebar to continue...")
    # Use a non-blocking listener and wait
    listener = keyboard.Listener(on_press=on_press)
    listener.start()
    listener.join() # Wait until listener stops (on space press)

# Main script logic
if __name__ == "__main__":
    limits_data = [] # Store limits as a list of dictionaries
    follower_arm = None
    listener = None
    all_motor_ids = [m[0] for m in follower_motors.values()] # Needed for finally block

    try:
        print(f"Connecting to follower arm on port {follower_port}...")
        follower_arm = FeetechMotorsBus(follower_config)
        follower_arm.connect()
        print("Connected.")

        print("Disabling torque for all motors. Move the arm manually.")
        follower_arm.write("Torque_Enable", TorqueMode.DISABLED.value)
        time.sleep(1)

        num_motors_to_calibrate = 4
        for i in range(num_motors_to_calibrate):
            motor_name = motor_names_in_order[i]
            # --- Use Corrected ID for Prompt --- 
            motor_id = follower_motors[motor_name][0] 
            print("-"*20)
            # --- Updated Prompt --- 
            print(f"Calibrating MOTOR {motor_id} ({motor_name})") 
            print("-"*20)

            # --- Get Limit 1 --- 
            # --- Updated Prompt --- 
            print(f"Move motor {motor_id} ({motor_name}) to its FIRST physical limit.")
            wait_for_spacebar()
            current_positions = follower_arm.read("Present_Position")
            # Assuming read returns in the order defined in follower_motors
            # --- Convert to standard Python int --- 
            limit1_steps = int(current_positions[i]) 
            print(f"--> Limit 1 for {motor_name} recorded: {limit1_steps} steps")

            # --- Get Limit 2 --- 
            # --- Updated Prompt --- 
            print(f"\nNow move motor {motor_id} ({motor_name}) to its OTHER physical limit.")
            wait_for_spacebar()
            current_positions = follower_arm.read("Present_Position")
            # --- Convert to standard Python int --- 
            limit2_steps = int(current_positions[i])
            print(f"--> Limit 2 for {motor_name} recorded: {limit2_steps} steps")

            # min/max will now use standard ints
            min_steps = min(limit1_steps, limit2_steps)
            max_steps = max(limit1_steps, limit2_steps)
            limits_data.append({
                "id": motor_id, # Store correct ID
                "name": motor_name,
                "min_steps": min_steps, # Now guaranteed to be standard int
                "max_steps": max_steps  # Now guaranteed to be standard int
            })
            print(f"Stored limits for {motor_name}: Min={min_steps}, Max={max_steps} steps.")
            time.sleep(0.5)

        # --- Process and Save Results --- 
        print("\n" + "="*30)
        print("Calibration Complete. Calculating degrees and preparing JSON output:")
        print("="*30)

        # Add degree limits to the dictionaries
        for i, limit_entry in enumerate(limits_data):
            min_steps = limit_entry["min_steps"]
            max_steps = limit_entry["max_steps"]
            # Use index `i` (0-3) for the calibration lookup
            min_angle = servo_step_to_angle(min_steps, i) 
            max_angle = servo_step_to_angle(max_steps, i)
            limit_entry["min_degrees"] = round(min_angle, 2) # Round for cleaner JSON
            limit_entry["max_degrees"] = round(max_angle, 2)

            print(f"Motor {limit_entry['id']} ({limit_entry['name']}):")
            print(f"  Steps: [{limit_entry['min_steps']}, {limit_entry['max_steps']}]")
            print(f"  Degrees: [{limit_entry['min_degrees']}, {limit_entry['max_degrees']}]")

        # --- Save as JSON --- 
        filename = "limits.json"
        try:
            with open(filename, 'w') as f:
                json.dump(limits_data, f, indent=4) # Save the list of dictionaries as JSON
            print(f"\nResults saved to {filename}")
        except IOError as e:
            print(f"\nError saving results to {filename}: {e}")
        except TypeError as e:
             print(f"\nError converting results to JSON: {e}")

    except Exception as e:
        print(f"\nAn error occurred: {e}", file=sys.stderr)
    finally:
        # --- Cleanup remains the same --- 
        if follower_arm and follower_arm.is_connected:
            try:
                print("\nRe-enabling torque for safety...")
                follower_arm.write("Torque_Enable", TorqueMode.ENABLED.value)
                follower_arm.disconnect()
                print("Disconnected.")
            except Exception as e:
                print(f"Error during cleanup: {e}", file=sys.stderr)
        if listener and listener.is_alive():
            listener.stop() 