import time
import numpy as np
import torch

from lerobot.common.robot_devices.robots.manipulator import ManipulatorRobot
from lerobot.common.robot_devices.robots.configs import So100RobotConfig

# 1. Create the robot configuration
#    You might need to adjust the port names if they are different on your system.
#    Check examples/10_use_so100.md or lerobot/common/robot_devices/robots/configs.py
#    for the default ports and update if necessary.
robot_config = So100RobotConfig(
    # Example: Override default ports if needed
    # leader_arms={"main": FeetechMotorsBusConfig(port="/dev/ttyYOUR_LEADER_PORT", ...)},
    # follower_arms={"main": FeetechMotorsBusConfig(port="/dev/ttyYOUR_FOLLOWER_PORT", ...)},
    # Disable cameras if you don't need them for direct control
    cameras={}
)

# 2. Instantiate the robot object
robot = ManipulatorRobot(robot_config)

try:
    # 3. Connect to the robot hardware (this also handles calibration loading/running)
    print("Connecting to the robot...")
    robot.connect()
    print("Robot connected successfully.")

    # Ensure calibration is active
    # robot.activate_calibration() # connect() should do this, but explicit call can be added if needed

    # Example: Get current state (joint positions) of the 'main_follower' arm
    print("Reading current state...")
    observation = robot.capture_observation()
    print("Current follower joint positions (degrees):")
    # Access the concatenated state tensor directly
    print(observation["observation.state"])

    # --- Example: Sending a specific joint position command ---
    # IMPORTANT: Actions are typically represented as a dictionary where keys
    #            are like 'arm_name/joint_positions' and values are numpy arrays.
    #            The order of joints in the array must match the configuration.
    #            Refer to the So100RobotConfig for the joint names/order:
    #            ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]

    # Define target joint positions in degrees for the 'main_follower' arm
    # Modify these values to move the arm to a desired pose. BE CAREFUL! Start with small movements.
    target_joint_positions_deg = np.array([
        0.0,   # shoulder_pan
        0.0, # shoulder_lift
        -10.0,  # elbow_flex
        0.0,   # wrist_flex
        0.0,   # wrist_roll
        50.0   # gripper (degrees might correspond to open/close)
    ])

    # Convert numpy array to torch tensor for send_action
    action_tensor_to_send = torch.from_numpy(target_joint_positions_deg).float()

    print(f"\nSending action tensor: {action_tensor_to_send}")
    # Pass the tensor directly to send_action
    robot.send_action(action_tensor_to_send)

    # Wait for the robot to reach the position (adjust sleep time as needed)
    print("Waiting for move to complete...")
    time.sleep(3)
    print("Move potentially complete.")

    # Read state again to see the result
    observation = robot.capture_observation()
    print("New follower joint positions (degrees):")
    print(observation["observation.state"])

    # --- Example: Simple back-and-forth motion ---
    print("\nStarting simple back-and-forth motion...")
    # Get initial positions from the state tensor
    initial_positions = observation["observation.state"]
    for _ in range(3):
        # Move slightly
        target_joint_positions_tensor = initial_positions + torch.tensor([10, 10, -10, 0, 0, 0], dtype=torch.float32)
        print(f"Moving to: {target_joint_positions_tensor}")
        robot.send_action(target_joint_positions_tensor)
        time.sleep(1.0)

        # Move back
        print(f"Moving back to: {initial_positions}")
        # Pass the initial_positions tensor directly
        robot.send_action(initial_positions)
        time.sleep(1.5)

    print("Motion finished.")


except Exception as e:
    print(f"An error occurred: {e}")

finally:
    try:
        # Check if the robot is connected before trying to disable torque
        if robot.is_connected:
            print("Disabling torque on follower motors...")
            for name, arm in robot.follower_arms.items():
                try:
                    # Write 0 to disable torque
                    arm.write("Torque_Enable", 0)
                    print(f"Torque disabled for arm: {name}")
                except Exception as torque_err:
                    print(f"Could not disable torque for arm {name}: {torque_err}")
    except AttributeError:
        # Handle cases where robot object might not be fully initialized if connect failed early
        print("Robot object state prevents torque disable check.")
    except Exception as final_err:
        print(f"An error occurred during torque disable: {final_err}")

    # 4. Disconnect (important to release hardware)
    print("Disconnecting robot...")
    # Check if disconnect method exists and robot object exists before calling
    if hasattr(robot, 'disconnect') and callable(getattr(robot, 'disconnect')):
         robot.disconnect()
    print("Robot disconnected.")
