#!/usr/bin/env python3
"""
Test script to verify simulation interface works correctly.
"""

import time
import numpy as np
from piper_teleop.config import TelegripConfig, config
from piper_teleop.robot_server.core.robot_interface import RobotInterface

def test_simulation():
    print("🧪 Testing Piper Simulation Interface\n")
    
    # Create config for simulation
    config.enable_robot = False
    config.sim_name = "pybullet"  # Use PyBullet
    config.enable_visualization = True

    print(f"Config: enable_robot={config.enable_robot}, sim_name={config.sim_name}")
    print(f"URDF path: {config.get_absolute_urdf_path()}")
    
    # Create robot interface
    robot_interface = RobotInterface(config)
    
    # Setup kinematics
    print("\n📐 Setting up kinematics...")
    robot_interface.setup_kinematics()
    
    # Connect to simulation
    print("\n🔌 Connecting to simulation...")
    success = robot_interface.connect()
    
    if not success:
        print("❌ Failed to connect to simulation")
        return
    
    print("✅ Connected successfully!")
    
    # Get initial state
    print("\n📊 Getting initial robot state...")
    obs = robot_interface.get_observation()
    print(f"Left arm observation: {obs['left']}")
    print(f"Right arm observation: {obs['right']}")
    
    # Test sending commands
    print("\n🎮 Testing command sending...")
    test_angles = np.array([0.0, 0.5, -0.5, 0.0, 0.0, 0.0,  # Left arm
                            0.0, 0.5, -0.5, 0.0, 0.0, 0.0,  # Right arm
                            0.02, 0.5])  # Grippers
    
    robot_interface.update_arm_angles(test_angles)
    
    for i in range(10):
        robot_interface.send_command()
        time.sleep(0.1)
        print(f"Step {i+1}/10 - Sending commands...")

    # Get updated state
    print("\n📊 Getting updated robot state...")
    obs = robot_interface.get_observation()
    print(f"Left arm observation: {obs['left']}")
    print(f"Right arm observation: {obs['right']}")
    
    # Test end effector poses
    print("\n🎯 Testing end effector poses...")
    left_ee = robot_interface.get_end_effector_transform("left")
    right_ee = robot_interface.get_end_effector_transform("right")
    print(f"Left EE transform:\n{left_ee}")
    print(f"Right EE transform:\n{right_ee}")
    
    # Return to initial position
    print("\n⏪ Returning to initial position...")
    robot_interface.return_to_initial_position()
    
    time.sleep(2)
    
    # Disconnect
    print("\n🔌 Disconnecting...")
    robot_interface.disconnect()
    
    print("\n✅ Test completed successfully!")

if __name__ == "__main__":
    test_simulation()
