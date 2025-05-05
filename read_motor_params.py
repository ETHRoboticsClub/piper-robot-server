import numpy as np
from lerobot.common.robot_devices.motors.feetech import FeetechMotorsBus, FeetechMotorsBusConfig, TorqueMode
import time

# --- Robot and Motor Config for UNTOUCHED Arm ---
# *** IMPORTANT: Verify this port is correct for the untouched arm ***
follower_port = "/dev/ttySO100follower" 
follower_port = "/dev/ttySO100leader" 

# Assuming the motor configuration is identical to the first arm
follower_motors = {
    "Shoulder_Rotation": (1, "sts3215"), # Base rotation
    "Shoulder_Pitch": (2, "sts3215"),    # Shoulder lift
    "Elbow": (3, "sts3215"),             # Elbow flex
    "Wrist_Pitch": (4, "sts3215"),      # Wrist flex/pitch
    "Wrist_Roll": (5, "sts3215"),       # Wrist roll
    "Gripper": (6, "sts3215"),          # Gripper
}
follower_config = FeetechMotorsBusConfig(port=follower_port, motors=follower_motors)
NUM_MOTORS = len(follower_motors)
all_motor_names = list(follower_motors.keys()) # Get names for reading

# --- Parameters to Read ---
# Use correct data_names from SCS_SERIES_CONTROL_TABLE (or equivalent for STS)
ACCELERATION_NAME = "Acceleration" # Address 41 in SCS, likely same for STS
GOAL_SPEED_NAME = "Goal_Speed"       # Address 46 in SCS, likely same for STS
parameters_to_read = [ACCELERATION_NAME, GOAL_SPEED_NAME]

# --- Connection and Reading Logic ---
follower_arm = None
try:
    print(f"Attempting to connect to arm at {follower_port}...")
    follower_arm = FeetechMotorsBus(follower_config)
    follower_arm.connect()
    print("Connected successfully.")

    # Optional: Check torque status first? Might not be necessary just for reading.
    # try:
    #     torque_status = follower_arm.read("Torque_Enable")
    #     print(f"Current Torque Status: {torque_status}")
    #     # Consider enabling torque if needed for reading, but unlikely.
    # except Exception as e:
    #     print(f"Warning: Could not read torque status: {e}")

    print(f"Reading parameters: {', '.join(parameters_to_read)}")

    # Read parameters for all motors at once if possible, or individually
    # The `read` function likely handles reading from all specified motors by default
    # if motor_names isn't provided for parameters like Acceleration/Goal_Speed.
    # Check library documentation if issues arise.
    
    read_values = {}
    all_succeeded = True
    for param_name in parameters_to_read:
        try:
            # Read the parameter for all motors defined in the bus
            values = follower_arm.read(param_name) 
            read_values[param_name] = values
            print(f"  Successfully read '{param_name}': {values}")
        except Exception as e:
            print(f"  ERROR reading '{param_name}': {e}")
            all_succeeded = False
            
    if all_succeeded:
        print("Successfully read all requested parameters.")
        # You can now use these 'read_values' dict
        # For example:
        # acc_values = read_values[ACCELERATION_NAME]
        # speed_values = read_values[GOAL_SPEED_NAME]
        # print(f"Acceleration values found: {acc_values}")
        # print(f"Goal Speed values found: {speed_values}")
    else:
        print("Finished reading parameters, but some errors occurred.")


except Exception as e:
    print(f"An error occurred during connection or reading: {e}")

finally:
    # Cleanup
    if follower_arm and follower_arm.is_connected:
        print("Disconnecting from the arm...")
        # Optional: Disable torque if it was potentially enabled? Unlikely needed here.
        # try:
        #     follower_arm.write("Torque_Enable", TorqueMode.DISABLED.value)
        #     time.sleep(0.1) # Short pause
        # except Exception as e_torque:
        #     print(f"Could not disable torque on exit: {e_torque}")
        follower_arm.disconnect()
        print("Follower arm disconnected.")
    else:
        print("No active connection to disconnect.")

print("Script finished.") 