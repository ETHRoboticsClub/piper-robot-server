import time
import numpy as np
import torch
import json
import os
import sys

# Add matplotlib import
import matplotlib.pyplot as plt

from lerobot.common.robot_devices.robots.manipulator import ManipulatorRobot
from lerobot.common.robot_devices.robots.configs import So100RobotConfig
from lerobot.common.robot_devices.motors.configs import FeetechMotorsBusConfig
from lerobot.common.robot_devices.utils import RobotDeviceNotConnectedError

# --- Constants ---
# Joint names order expected by So100RobotConfig/ManipulatorRobot
JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
NUM_JOINTS = len(JOINT_NAMES)

# Motor configuration (copy from lerobot_keyboard_ik.py or your setup)
common_motors = {
    "shoulder_pan": [1, "sts3215"],
    "shoulder_lift": [2, "sts3215"],
    "elbow_flex": [3, "sts3215"],
    "wrist_flex": [4, "sts3215"],
    "wrist_roll": [5, "sts3215"],
    "gripper": [6, "sts3215"],
}

# --- Robot Configuration ---
robot_config = So100RobotConfig(
    follower_arms={"main": FeetechMotorsBusConfig(port="/dev/ttySO100follower", motors=common_motors.copy())},
    # Disable cameras and leader arm if not needed for this test
    leader_arms={}, 
    cameras={}
)

# Instantiate the robot object
robot = ManipulatorRobot(robot_config)

def load_calibration_data(json_path="calibration/so100/arm_red_follower.json"):
    """Loads calibration data from the specified JSON file."""
    cache_json_path = os.path.join(".cache", json_path)
    effective_json_path = cache_json_path if os.path.exists(cache_json_path) else json_path

    if not os.path.exists(effective_json_path):
        print(f"ERROR: Calibration file not found at '{effective_json_path}' or '{json_path}'.", file=sys.stderr)
        return None
    
    try:
        with open(effective_json_path, 'r') as f:
            calib_data = json.load(f)
        print(f"Successfully loaded calibration data from: {effective_json_path}")
        return calib_data
    except (json.JSONDecodeError, ValueError, FileNotFoundError, Exception) as e:
        print(f"Error loading or parsing calibration file '{effective_json_path}': {e}.", file=sys.stderr)
        return None

def disable_follower_torque(robot_instance, arm_name="main"):
    """Disables torque for all motors on the specified follower arm."""
    print(f"\nDisabling torque for follower arm '{arm_name}'...")
    try:
        if hasattr(robot_instance, 'follower_arms') and arm_name in robot_instance.follower_arms:
             arm = robot_instance.follower_arms[arm_name]
             arm.write("Torque_Enable", 0)
             time.sleep(0.2)
             print(f"Torque disable command sent for arm '{arm_name}'.")
        else:
             print(f"Error: Follower arm '{arm_name}' not found.", file=sys.stderr)
    except Exception as e:
        print(f"Error disabling torque: {e}", file=sys.stderr)

if __name__ == "__main__":
    calib_data = load_calibration_data()
    if calib_data:
        print("\n--- Calibration Data --- ")
        print(f"Homing Offset: {calib_data.get('homing_offset')}")
        print(f"Start Pos    : {calib_data.get('start_pos')}")
        print(f"End Pos      : {calib_data.get('end_pos')}")
        print(f"Calib Mode   : {calib_data.get('calib_mode')}")
        print(f"Drive Mode   : {calib_data.get('drive_mode')}")
        print("------------------------")

    all_readings = [[] for _ in range(NUM_JOINTS)]
    run_duration = 20 # Seconds
    sample_rate_hz = 5
    sleep_interval = 1.0 / sample_rate_hz

    try:
        print("\nConnecting to the robot...")
        robot.connect()
        print("Robot connected successfully.")

        # Disable torque BEFORE asking user to move
        disable_follower_torque(robot, arm_name="main")

        # --- Record Joint Angles Over Time ---
        input(f"\n---> Press Enter to start recording. Move all joints through their full range for {run_duration} seconds... <---")
        print(f"\nRecording angles for {run_duration} seconds at ~{sample_rate_hz} Hz...")
        
        start_time = time.time()
        while time.time() - start_time < run_duration:
            loop_start = time.time()
            try:
                observation = robot.capture_observation()
                if "observation.state" in observation:
                    degrees_tensor = observation["observation.state"]
                    degrees = degrees_tensor.cpu().numpy()
                    if len(degrees) == NUM_JOINTS:
                        for i in range(NUM_JOINTS):
                            all_readings[i].append(degrees[i])
                    else:
                        print(f"Warning: Observation length mismatch ({len(degrees)} != {NUM_JOINTS})")
                else:
                    print("Warning: 'observation.state' not found.")
            except Exception as e:
                print(f"Error capturing observation during loop: {e}")
                # Optionally break or continue based on error type

            # Maintain approximate sample rate
            elapsed = time.time() - loop_start
            sleep_time = max(0, sleep_interval - elapsed)
            time.sleep(sleep_time)
            
        print("\nRecording complete.")

        # --- Analyze and Print Ranges --- #
        print("\n--- Observed Degree Ranges --- ")
        observed_min_deg = np.full(NUM_JOINTS, np.nan)
        observed_max_deg = np.full(NUM_JOINTS, np.nan)
        
        for i in range(NUM_JOINTS):
            joint_name = JOINT_NAMES[i]
            readings = all_readings[i]
            if readings:
                observed_min_deg[i] = np.min(readings)
                observed_max_deg[i] = np.max(readings)
                print(f"  {joint_name:<15}: Min={observed_min_deg[i]:8.2f}, Max={observed_max_deg[i]:8.2f} ({len(readings)} samples)")
            else:
                print(f"  {joint_name:<15}: No readings captured.")
        print("----------------------------")

        # --- Plotting Angle Distributions --- #
        print("\nGenerating histograms (plots will show sequentially)...")
        for i in range(NUM_JOINTS):
            joint_name = JOINT_NAMES[i]
            readings = all_readings[i]
            
            if readings and len(readings) > 1:
                plt.figure(figsize=(10, 5))
                # Use a decent number of bins to see gaps
                plt.hist(readings, bins=50, density=False, alpha=0.75)
                plt.title(f"Observed Angle Distribution: {joint_name}")
                plt.xlabel("Angle (Degrees)")
                plt.ylabel("Frequency (Number of Samples)")
                # Add min/max lines for reference
                plt.axvline(observed_min_deg[i], color='r', linestyle='dashed', linewidth=1, label=f'Min: {observed_min_deg[i]:.2f}')
                plt.axvline(observed_max_deg[i], color='g', linestyle='dashed', linewidth=1, label=f'Max: {observed_max_deg[i]:.2f}')
                plt.legend()
                plt.grid(True, axis='y', linestyle='--')
                # Show plots one by one (blocks until closed)
                plt.show()
            elif readings:
                 print(f"Skipping plot for {joint_name} - only 1 sample.")
            else:
                 print(f"Skipping plot for {joint_name} - no samples.")

    except KeyboardInterrupt:
        print("\nCtrl+C detected. Exiting early.")
    except Exception as e:
        print(f"\nAn unhandled error occurred: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # --- Cleanup ---
        print("\nInitiating shutdown sequence...")
        # Check if robot object exists and has the is_connected attribute/method
        robot_is_connected = False
        try:
            if robot and hasattr(robot, 'is_connected') and robot.is_connected:
                robot_is_connected = True
        except Exception as check_err:
             print(f"Could not check robot connection status: {check_err}")
             robot_is_connected = False
        
        if robot_is_connected:
            # Torque should already be disabled, but disable again just in case?
            # disable_follower_torque(robot, arm_name="main") # Maybe redundant
            
            # Disconnect robot only if it was connected
            try:
                 print("Disconnecting robot...")
                 robot.disconnect()
                 print("Robot disconnected.")
            except RobotDeviceNotConnectedError:
                 print("Robot was already disconnected.")
            except Exception as disc_err:
                 print(f"An error occurred during robot disconnect: {disc_err}", file=sys.stderr)
        else:
            print("Robot not connected or connection status unknown, skipping disconnect.")

        print("\nInvestigation complete. Please copy the 'Observed Degree Ranges' output.") 