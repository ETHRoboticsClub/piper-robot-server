#!/usr/bin/env python3
"""
Debug script to help determine lerobot angle compensation values.
This will help you figure out what angle transformations are needed
to make the PyBullet visualization match your physical robot poses.
"""

import sys
import numpy as np
import time

# Add the telegrip module to the path
sys.path.append('.')

def print_current_robot_state():
    """Print the current robot state for debugging."""
    from telegrip.core.robot_interface import RobotInterface
    from telegrip.config import TelegripConfig
    
    config = TelegripConfig()
    config.enable_pybullet = False  # Disable visualization for this debug
    
    try:
        robot_interface = RobotInterface(config)
        print("🔌 Connecting to robot...")
        robot_interface.connect()
        
        if robot_interface.is_connected:
            print("✅ Robot connected successfully!")
            print()
            
            for i in range(10):  # Read 10 samples
                try:
                    # Get arm angles the same way as _read_initial_state
                    left_angles = None
                    right_angles = None
                    
                    if robot_interface.left_robot and robot_interface.left_arm_connected:
                        observation = robot_interface.left_robot.get_observation()
                        if observation:
                            left_angles = np.array([
                                observation['shoulder_pan.pos'],
                                observation['shoulder_lift.pos'],
                                observation['elbow_flex.pos'],
                                observation['wrist_flex.pos'],
                                observation['wrist_roll.pos'],
                                observation['gripper.pos']
                            ])
                    
                    if robot_interface.right_robot and robot_interface.right_arm_connected:
                        observation = robot_interface.right_robot.get_observation()
                        if observation:
                            right_angles = np.array([
                                observation['shoulder_pan.pos'],
                                observation['shoulder_lift.pos'],
                                observation['elbow_flex.pos'],
                                observation['wrist_flex.pos'],
                                observation['wrist_roll.pos'],
                                observation['gripper.pos']
                            ])
                    
                    print(f"Sample {i+1}:")
                    if left_angles is not None:
                        print(f"  Left arm ACTUAL angles:    {left_angles}")
                        print(f"  Left arm INTERNAL angles:  {robot_interface.get_arm_angles('left')}")
                    else:
                        print(f"  Left arm: Not connected")
                        
                    if right_angles is not None:
                        print(f"  Right arm ACTUAL angles:   {right_angles}")
                        print(f"  Right arm INTERNAL angles: {robot_interface.get_arm_angles('right')}")
                    else:
                        print(f"  Right arm: Not connected")
                    print()
                    
                    time.sleep(1)
                    
                except Exception as e:
                    print(f"Error reading angles: {e}")
                    break
        else:
            print("❌ Failed to connect to robot")
            
    except Exception as e:
        print(f"Error: {e}")
    finally:
        try:
            robot_interface.disconnect()
            print("🔌 Robot disconnected")
        except:
            pass

def test_angle_compensation():
    """Test different angle compensation values."""
    print("=" * 60)
    print("ANGLE COMPENSATION TESTING")
    print("=" * 60)
    
    # Example current robot angles (replace with actual readings)
    left_angles = np.array([-3.6, -99.9, 97.9, 76.1, -100.6, 1.6])
    right_angles = np.array([2.2, -89.3, 97.8, 72.1, -3.5, 1.7])
    
    joint_names = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
    
    print("Current robot readings:")
    print(f"Left arm:  {left_angles}")
    print(f"Right arm: {right_angles}")
    print()
    
    # Test different compensation strategies
    compensations_to_test = [
        {"name": "No compensation", "transforms": [{"sign": 1, "offset": 0}] * 6},
        {"name": "Negate shoulder_lift", "transforms": [
            {"sign": 1, "offset": 0},   # shoulder_pan
            {"sign": -1, "offset": 0},  # shoulder_lift
            {"sign": 1, "offset": 0},   # elbow_flex
            {"sign": 1, "offset": 0},   # wrist_flex
            {"sign": 1, "offset": 0},   # wrist_roll
            {"sign": 1, "offset": 0},   # gripper
        ]},
        {"name": "Add 180° to elbow_flex", "transforms": [
            {"sign": 1, "offset": 0},     # shoulder_pan
            {"sign": 1, "offset": 0},     # shoulder_lift
            {"sign": 1, "offset": 180},   # elbow_flex
            {"sign": 1, "offset": 0},     # wrist_flex
            {"sign": 1, "offset": 0},     # wrist_roll
            {"sign": 1, "offset": 0},     # gripper
        ]},
        {"name": "Negate elbow_flex", "transforms": [
            {"sign": 1, "offset": 0},   # shoulder_pan
            {"sign": 1, "offset": 0},   # shoulder_lift
            {"sign": -1, "offset": 0},  # elbow_flex
            {"sign": 1, "offset": 0},   # wrist_flex
            {"sign": 1, "offset": 0},   # wrist_roll
            {"sign": 1, "offset": 0},   # gripper
        ]},
    ]
    
    for comp in compensations_to_test:
        print(f"\n{comp['name']}:")
        
        # Apply compensation to left arm
        compensated_left = []
        for i, angle in enumerate(left_angles):
            transform = comp['transforms'][i]
            compensated_angle = (angle * transform["sign"]) + transform["offset"]
            compensated_left.append(compensated_angle)
        
        # Apply compensation to right arm  
        compensated_right = []
        for i, angle in enumerate(right_angles):
            transform = comp['transforms'][i]
            compensated_angle = (angle * transform["sign"]) + transform["offset"]
            compensated_right.append(compensated_angle)
        
        print(f"  Left compensated:  {np.array(compensated_left)}")
        print(f"  Right compensated: {np.array(compensated_right)}")

def main():
    """Main function to run different debug modes."""
    print("LEROBOT ANGLE DEBUG TOOL")
    print("=" * 60)
    print("Choose an option:")
    print("1. Read current robot state")
    print("2. Test angle compensation strategies")
    print()
    
    try:
        choice = input("Enter choice (1-2): ").strip()
        
        if choice == "1":
            print_current_robot_state()
        elif choice == "2":
            test_angle_compensation()
            
    except KeyboardInterrupt:
        print("\nExiting...")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main() 