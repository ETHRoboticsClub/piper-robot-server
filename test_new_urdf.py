#!/usr/bin/env python3
"""
Test script to verify the new URDF configuration works correctly.
"""

import os
import sys
import pybullet as p
import numpy as np

# Add the telegrip module to the path
sys.path.append('.')

from telegrip.config import URDF_PATH, END_EFFECTOR_LINK_NAME, URDF_TO_INTERNAL_NAME_MAP, JOINT_ANGLE_COMPENSATION
from telegrip.core.visualizer import PyBulletVisualizer

def test_urdf_loading():
    """Test basic URDF loading."""
    print("=" * 50)
    print("Testing URDF Loading")
    print("=" * 50)
    
    print(f"URDF Path: {URDF_PATH}")
    print(f"File exists: {os.path.exists(URDF_PATH)}")
    
    if not os.path.exists(URDF_PATH):
        print("❌ URDF file not found!")
        return False
    
    # Test PyBullet loading
    p.connect(p.DIRECT)
    try:
        robot_id = p.loadURDF(URDF_PATH)
        num_joints = p.getNumJoints(robot_id)
        print(f"✅ Successfully loaded URDF with {num_joints} joints")
        
        # Print joint information
        print("\nJoint Information:")
        for i in range(num_joints):
            joint_info = p.getJointInfo(robot_id, i)
            joint_name = joint_info[1].decode()
            link_name = joint_info[12].decode()
            print(f"  Joint {i}: '{joint_name}' -> Link: '{link_name}'")
        
        p.disconnect()
        return True
    except Exception as e:
        print(f"❌ Error loading URDF: {e}")
        p.disconnect()
        return False

def test_joint_mapping():
    """Test joint name mapping."""
    print("\n" + "=" * 50)
    print("Testing Joint Mapping")
    print("=" * 50)
    
    print("URDF to Internal Name Mapping:")
    for urdf_name, internal_name in URDF_TO_INTERNAL_NAME_MAP.items():
        print(f"  '{urdf_name}' -> '{internal_name}'")
    
    print(f"\nEnd Effector Link: '{END_EFFECTOR_LINK_NAME}'")
    
    return True

def test_visualizer():
    """Test the PybulletVisualizer with the new URDF."""
    print("\n" + "=" * 50)
    print("Testing PybulletVisualizer")
    print("=" * 50)
    
    try:
        # Create visualizer (this should load the URDF)
        visualizer = PyBulletVisualizer(URDF_PATH, use_gui=False)  # Use headless mode for testing
        print("✅ Successfully created PyBulletVisualizer")
        
        # Setup the visualizer
        if not visualizer.setup():
            print("❌ Failed to setup PyBulletVisualizer")
            return False
        
        # Test setting joint angles
        test_angles = [0.0, 0.5, -0.5, 0.2, -0.2, 0.1]
        print(f"Testing with joint angles: {test_angles}")
        
        # Update arm (this should work with the new joint mapping)
        test_angles_np = np.array(test_angles)
        visualizer.update_robot_pose(test_angles_np, "left")
        print("✅ Successfully updated arm visualization")
        
        visualizer.disconnect()
        return True
        
    except Exception as e:
        print(f"❌ Error with PybulletVisualizer: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_joint_compensation():
    """Test joint angle compensation."""
    print("\n" + "=" * 50)
    print("Testing Joint Angle Compensation")
    print("=" * 50)
    
    print("Joint Angle Compensation Settings:")
    for joint_name, compensation in JOINT_ANGLE_COMPENSATION.items():
        print(f"  {joint_name}: sign={compensation['sign']}, offset={compensation['offset']}")
    
    # Test compensation with example angles
    test_angles = [10.0, 20.0, -30.0, 15.0, -25.0, 5.0]
    print(f"\nOriginal angles: {test_angles}")
    
    compensated_angles = []
    joint_names = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
    
    for i, angle in enumerate(test_angles):
        if i < len(joint_names):
            joint_name = joint_names[i]
            compensation = JOINT_ANGLE_COMPENSATION[joint_name]
            compensated_angle = (angle * compensation["sign"]) + compensation["offset"]
            compensated_angles.append(compensated_angle)
            print(f"  {joint_name}: {angle}° -> {compensated_angle}°")
    
    print("✅ Joint compensation test completed")
    return True

def main():
    """Run all tests."""
    print("Testing New URDF Configuration")
    print("=" * 80)
    
    tests = [
        test_urdf_loading,
        test_joint_mapping,
        test_visualizer,
        test_joint_compensation
    ]
    
    results = []
    for test in tests:
        try:
            result = test()
            results.append(result)
        except Exception as e:
            print(f"❌ Test failed with exception: {e}")
            import traceback
            traceback.print_exc()
            results.append(False)
    
    print("\n" + "=" * 80)
    print("Test Results Summary")
    print("=" * 80)
    
    test_names = ["URDF Loading", "Joint Mapping", "PybulletVisualizer", "Joint Compensation"]
    for i, (name, result) in enumerate(zip(test_names, results)):
        status = "✅ PASS" if result else "❌ FAIL"
        print(f"{name}: {status}")
    
    overall_result = all(results)
    print(f"\nOverall Result: {'✅ ALL TESTS PASSED' if overall_result else '❌ SOME TESTS FAILED'}")
    
    return overall_result

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1) 