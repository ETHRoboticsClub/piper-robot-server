#!/usr/bin/env python3
"""
Script to record reference poses for the SO100 robot arms.
Manually position the arms in good configurations and press Enter to record them.
These poses will be used as additional reference points for IK solving to prevent getting stuck.
"""

import os
import sys
import json
import time
import numpy as np
import logging
from pathlib import Path

# Add the teleop package to the path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from teleop.config import TeleopConfig, REFERENCE_POSES_FILE, NUM_JOINTS
from teleop.core.robot_interface import RobotInterface

# Setup logging
logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
logger = logging.getLogger(__name__)

class PoseRecorder:
    """Records robot poses for use as IK reference poses."""
    
    def __init__(self):
        self.config = TeleopConfig()
        self.robot_interface = RobotInterface(self.config)
        self.reference_poses = {
            'left': [],
            'right': []
        }
        
    def connect_robot(self) -> bool:
        """Connect to the robot."""
        logger.info("Connecting to robot...")
        if not self.robot_interface.connect():
            logger.error("Failed to connect to robot")
            return False
        
        logger.info("✅ Robot connected successfully")
        
        # Disengage motors for manual positioning
        logger.info("Disengaging motors for manual positioning...")
        if self.robot_interface.disengage():
            logger.info("✅ Motors disengaged - you can now manually position the arms")
        else:
            logger.warning("⚠️ Failed to disengage motors, but continuing...")
        
        return True
    
    def read_current_poses(self) -> dict:
        """Read current joint angles for both arms from robot hardware."""
        try:
            # Read actual joint positions from robot hardware
            if not self.robot_interface.robot:
                logger.error("No robot connection available")
                return None
                
            # Capture current observation from robot
            observation = self.robot_interface.robot.capture_observation()
            if not observation or "observation.state" not in observation:
                logger.error("Failed to capture robot observation")
                return None
            
            # Extract joint state
            current_state = observation["observation.state"].cpu().numpy()
            logger.debug(f"Read joint state shape: {current_state.shape}, values: {current_state}")
            
            # Parse dual arm configuration
            if len(current_state) == NUM_JOINTS * 2:  # Dual arm
                left_angles = current_state[:NUM_JOINTS]
                right_angles = current_state[NUM_JOINTS:]
            elif len(current_state) == NUM_JOINTS:  # Single arm fallback
                logger.warning("Single arm state detected, using for both arms")
                left_angles = current_state
                right_angles = current_state
            else:
                logger.error(f"Unexpected joint state length: {len(current_state)}")
                return None
            
            return {
                'left': left_angles.tolist(),
                'right': right_angles.tolist()
            }
            
        except Exception as e:
            logger.error(f"Failed to read current poses from robot hardware: {e}")
            return None
    
    def record_poses(self):
        """Interactive pose recording session."""
        print("\n" + "="*60)
        print("SO100 Robot Reference Pose Recorder")
        print("="*60)
        print("This script will help you record reference poses for both arms.")
        print("These poses will be used to improve IK solving by providing")
        print("additional starting points to prevent getting stuck in local minima.")
        print("\nInstructions:")
        print("1. Motors are DISENGAGED - you can safely move the arms by hand")
        print("2. Manually position the robot arms in good configurations")
        print("3. Press Enter to record the current pose")
        print("4. Record 2-3 diverse poses for each arm")
        print("5. Type 'done' when finished")
        print(f"6. The poses will be saved to '{REFERENCE_POSES_FILE}'")
        print("="*60)
        
        pose_count = 0
        
        while True:
            print(f"\n📍 Recording pose #{pose_count + 1}")
            print("Position the robot arms in a good configuration...")
            
            user_input = input("Press Enter to record current pose (or 'done' to finish): ").strip().lower()
            
            if user_input == 'done':
                break
                
            # Read current poses
            current_poses = self.read_current_poses()
            if current_poses is None:
                print("❌ Failed to read current poses. Try again.")
                continue
            
            # Add to reference poses
            self.reference_poses['left'].append(current_poses['left'])
            self.reference_poses['right'].append(current_poses['right'])
            pose_count += 1
            
            print(f"✅ Recorded pose #{pose_count}")
            print(f"   Left arm:  {[f'{x:.1f}' for x in current_poses['left']]}")
            print(f"   Right arm: {[f'{x:.1f}' for x in current_poses['right']]}")
            
            # Show joint names for clarity
            joint_names = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
            print(f"   Joint breakdown:")
            for i, name in enumerate(joint_names):
                print(f"     {name}: L={current_poses['left'][i]:.1f}°, R={current_poses['right'][i]:.1f}°")
            
            if pose_count >= 2:
                print(f"\n✅ Good! You have {pose_count} reference poses recorded.")
                print("You can record more poses or type 'done' to finish.")
    
    def save_poses(self):
        """Save recorded poses to file."""
        if not self.reference_poses['left'] and not self.reference_poses['right']:
            print("❌ No poses recorded. Nothing to save.")
            return False
        
        # Get the configured file path
        filename = Path(REFERENCE_POSES_FILE)
        
        # Create parent directory if it doesn't exist
        filename.parent.mkdir(parents=True, exist_ok=True)
        
        try:
            with open(filename, 'w') as f:
                json.dump(self.reference_poses, f, indent=2)
            
            print(f"\n✅ Saved {len(self.reference_poses['left'])} reference poses to {filename}")
            print("These poses will now be used by the IK solver to improve performance.")
            return True
            
        except Exception as e:
            logger.error(f"Failed to save poses: {e}")
            return False
    
    def disconnect_robot(self):
        """Disconnect from the robot."""
        try:
            self.robot_interface.disconnect()
            logger.info("Robot disconnected")
        except Exception as e:
            logger.error(f"Error disconnecting robot: {e}")

def main():
    """Main function to read and save robot poses."""
    print("🤖 SO100 Reference Pose Recorder")
    print("=" * 40)
    
    # Cache file for reference poses
    cache_file = Path("reference_poses.json")
    
    recorder = PoseRecorder()
    
    try:
        # Connect to robot
        if not recorder.connect_robot():
            sys.exit(1)
        
        # Wait a moment for connection to stabilize
        time.sleep(1)
        
        # Record poses interactively
        recorder.record_poses()
        
        # Save recorded poses
        recorder.save_poses()
        
    except KeyboardInterrupt:
        print("\n\n🛑 Recording interrupted by user")
    except Exception as e:
        logger.error(f"Unexpected error: {e}")
        sys.exit(1)
    finally:
        # Always disconnect
        recorder.disconnect_robot()

if __name__ == "__main__":
    main() 