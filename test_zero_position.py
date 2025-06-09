#!/usr/bin/env python3
"""
Test script to move arms to zero position and see what the robot considers "zero".
This helps understand the robot's internal coordinate system after the lerobot update.
"""

import sys
import numpy as np
import time

# Add the telegrip module to the path
sys.path.append('.')

def test_zero_position():
    """Move both arms to zero position and observe the result."""
    from telegrip.core.robot_interface import RobotInterface
    from telegrip.config import TelegripConfig
    
    config = TelegripConfig()
    config.enable_pybullet = False  # Disable visualization for this test
    
    try:
        robot_interface = RobotInterface(config)
        print("🔌 Connecting to robot...")
        robot_interface.connect()
        
        if robot_interface.is_connected:
            print("✅ Robot connected successfully!")
            print()
            
            # Read current positions
            print("📍 CURRENT POSITIONS:")
            if robot_interface.left_arm_connected:
                left_current = robot_interface.get_actual_arm_angles("left")
                print(f"  Left arm:  {left_current.round(1)}")
            if robot_interface.right_arm_connected:
                right_current = robot_interface.get_actual_arm_angles("right")
                print(f"  Right arm: {right_current.round(1)}")
            print()
            
            # Engage the robot
            print("🔌 Engaging robot motors...")
            robot_interface.engage()
            
            # Set zero position for both arms
            zero_position = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            
            print(f"🎯 MOVING TO ZERO POSITION: {zero_position}")
            robot_interface.left_arm_angles = zero_position.copy()
            robot_interface.right_arm_angles = zero_position.copy()
            
            # Send commands repeatedly to ensure movement
            print("📤 Sending zero position commands...")
            for i in range(50):  # Send for 5 seconds at 10Hz
                success = robot_interface.send_command()
                if not success:
                    print(f"⚠️  Warning: Command {i+1} failed")
                time.sleep(0.1)
                
                # Print progress every 10 commands
                if (i+1) % 10 == 0:
                    print(f"  Sent {i+1}/50 commands...")
            
            print("✅ Movement commands sent")
            print()
            
            # Wait a bit for movement to complete
            print("⏳ Waiting for movement to complete...")
            time.sleep(2)
            
            # Read final positions
            print("📍 FINAL POSITIONS (what robot thinks is 'zero'):")
            if robot_interface.left_arm_connected:
                left_final = robot_interface.get_actual_arm_angles("left")
                print(f"  Left arm:  {left_final.round(1)}")
            if robot_interface.right_arm_connected:
                right_final = robot_interface.get_actual_arm_angles("right")
                print(f"  Right arm: {right_final.round(1)}")
            print()
            
            print("📊 SUMMARY:")
            print("If the arms moved to a reasonable 'zero' position, these final")
            print("readings show what the robot's internal coordinate system considers")
            print("to be zero. If they're close to [0,0,0,0,0,0], the calibration is good.")
            print("If not, there might be an offset in the robot's coordinate system.")
            
        else:
            print("❌ Failed to connect to robot")
            
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            robot_interface.disconnect()
            print("🔌 Robot disconnected")
        except:
            pass

def test_current_position_only():
    """Just read and display current robot positions without moving."""
    from telegrip.core.robot_interface import RobotInterface
    from telegrip.config import TelegripConfig
    
    config = TelegripConfig()
    config.enable_pybullet = False
    
    try:
        robot_interface = RobotInterface(config)
        print("🔌 Connecting to robot...")
        robot_interface.connect()
        
        if robot_interface.is_connected:
            print("✅ Robot connected successfully!")
            print()
            
            print("📍 CURRENT ROBOT POSITIONS:")
            if robot_interface.left_arm_connected:
                left_angles = robot_interface.get_actual_arm_angles("left")
                print(f"  Left arm:  {left_angles.round(1)}")
            if robot_interface.right_arm_connected:
                right_angles = robot_interface.get_actual_arm_angles("right")
                print(f"  Right arm: {right_angles.round(1)}")
            print()
            
            print("These are the raw angle readings from the robot hardware.")
            
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

def main():
    """Main function to choose test mode."""
    print("ROBOT ZERO POSITION TEST")
    print("=" * 50)
    print("Choose an option:")
    print("1. Move arms to zero position [0,0,0,0,0,0]")
    print("2. Just read current positions (no movement)")
    print()
    
    try:
        choice = input("Enter choice (1-2): ").strip()
        
        if choice == "1":
            print("\n⚠️  WARNING: This will move the robot arms!")
            print("Make sure the arms have clear movement space.")
            confirm = input("Continue? (y/N): ").strip().lower()
            if confirm == 'y':
                test_zero_position()
            else:
                print("Operation cancelled.")
        elif choice == "2":
            test_current_position_only()
        else:
            print("Invalid choice")
            
    except KeyboardInterrupt:
        print("\nExiting...")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main() 