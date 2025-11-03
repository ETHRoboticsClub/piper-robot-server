#!/usr/bin/env python3
"""
Test script to demonstrate camera views in PyBullet simulation.
Shows third-person view and gripper-mounted cameras.
"""

import time
import numpy as np
import matplotlib.pyplot as plt
from piper_teleop.config import config
from piper_teleop.robot_server.core.robot_interface import RobotInterface

def test_cameras():
    print("🎥 Testing Camera Views in PyBullet Simulation\n")
    
    # Create config for simulation with GUI
    config.enable_robot = False
    config.sim_name = "pybullet"
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
    
    # Move arms to a visible position
    print("\n🎮 Moving arms to test position...")
    test_angles = np.array([0.0, 0.5, -0.5, 0.0, 0.0, 0.0,  # Left arm
                            0.0, 0.5, -0.5, 0.0, 0.0, 0.0,  # Right arm
                            0.02, 0.02])  # Grippers
    
    robot_interface.update_arm_angles(test_angles)
    
    for i in range(20):
        robot_interface.send_command()
        time.sleep(0.05)
    
    print("✅ Arms in position")
    
    # Get simulation interface
    sim = robot_interface.left_robot.sdk
    
    # Capture different camera views
    print("\n📸 Capturing camera images...")
    
    # Third-person view
    print("  - Third-person view camera")
    third_person_img = sim.get_camera_image(camera_type="third_person")
    
    # Left wrist camera
    print("  - Left wrist camera")
    left_wrist_img = sim.get_camera_image(camera_type="wrist", arm_id="left")

    # Right wrist camera
    print("  - Right wrist camera")
    right_wrist_img = sim.get_camera_image(camera_type="wrist", arm_id="right")

    # Display images
    print("\n🖼️  Displaying camera views...")
    fig, axes = plt.subplots(3, 3, figsize=(15, 10))
    fig.suptitle('PyBullet Simulation Camera Views', fontsize=16)
    
    # Third-person view
    if 'rgb' in third_person_img:
        axes[0, 0].imshow(third_person_img['rgb'])
        axes[0, 0].set_title('Third-Person View (RGB)')
        axes[0, 0].axis('off')
        
        axes[1, 0].imshow(third_person_img['depth'], cmap='plasma')
        axes[1, 0].set_title('Third-Person View (Depth)')
        axes[1, 0].axis('off')

        axes[2, 0].imshow(third_person_img['segmentation'], cmap='tab20')
        axes[2, 0].set_title('Third-Person View (Segmentation)')
        axes[2, 0].axis('off')
    
    if 'rgb' in left_wrist_img:
        axes[0, 1].imshow(left_wrist_img['rgb'])
        axes[0, 1].set_title('Left Wrist Camera (RGB)')
        axes[0, 1].axis('off')

        axes[1, 1].imshow(left_wrist_img['depth'], cmap='plasma')
        axes[1, 1].set_title('Left Wrist Camera (Depth)')
        axes[1, 1].axis('off')

        axes[2, 1].imshow(left_wrist_img['segmentation'], cmap='tab20')
        axes[2, 1].set_title('Left Wrist Camera (Segmentation)')
        axes[2, 1].axis('off')

    if 'rgb' in right_wrist_img:
        axes[0, 2].imshow(right_wrist_img['rgb'])
        axes[0, 2].set_title('Right Wrist Camera (RGB)')
        axes[0, 2].axis('off')

        axes[1, 2].imshow(right_wrist_img['depth'], cmap='plasma')
        axes[1, 2].set_title('Right Wrist Camera (Depth)')
        axes[1, 2].axis('off')

        axes[2, 2].imshow(right_wrist_img['segmentation'], cmap='tab20')
        axes[2, 2].set_title('Right Wrist Camera (Segmentation)')
        axes[2, 2].axis('off')

    plt.tight_layout()
    plt.savefig('camera_views.png', dpi=150, bbox_inches='tight')
    print("✅ Saved camera views to 'camera_views.png'")
    plt.show()
    
    # Keep simulation running for a bit
    print("\n⏸️  Simulation will run for 5 more seconds...")
    time.sleep(5)
    
    # Disconnect
    print("\n🔌 Disconnecting...")
    robot_interface.disconnect()
    
    print("\n✅ Test completed successfully!")

if __name__ == "__main__":
    test_cameras()
