"""
Robot interface module for the SO100 teleoperation system.
Provides a clean wrapper around ManipulatorRobot with safety checks and convenience methods.
"""

import numpy as np
import torch
import time
import logging
import os
from typing import Optional, Dict, Tuple

from lerobot.common.robot_devices.robots.manipulator import ManipulatorRobot
from lerobot.common.robot_devices.robots.configs import So100RobotConfig
from lerobot.common.robot_devices.motors.configs import FeetechMotorsBusConfig
from lerobot.common.robot_devices.utils import RobotDeviceNotConnectedError

from ..config import (
    TelegripConfig, COMMON_MOTORS, NUM_JOINTS, JOINT_NAMES,
    GRIPPER_OPEN_ANGLE, GRIPPER_CLOSED_ANGLE, 
    WRIST_FLEX_INDEX, URDF_TO_INTERNAL_NAME_MAP
)
from .kinematics import ForwardKinematics, IKSolver

logger = logging.getLogger(__name__)


class RobotInterface:
    """High-level interface for SO100 robot control with safety features."""
    
    def __init__(self, config: TelegripConfig):
        self.config = config
        self.robot = None
        self.is_connected = False
        self.is_engaged = False  # New state for motor engagement
        
        # Individual arm connection status
        self.left_arm_connected = False
        self.right_arm_connected = False
        
        # Joint state
        self.left_arm_angles = np.zeros(NUM_JOINTS)
        self.right_arm_angles = np.zeros(NUM_JOINTS)
        
        # Joint limits (will be set by visualizer)
        self.joint_limits_min_deg = np.full(NUM_JOINTS, -180.0)
        self.joint_limits_max_deg = np.full(NUM_JOINTS, 180.0)
        
        # Kinematics solvers (will be set after PyBullet setup)
        self.fk_solvers = {'left': None, 'right': None}
        self.ik_solvers = {'left': None, 'right': None}
        
        # Control timing
        self.last_send_time = 0
        
        # Error tracking - separate for each arm
        self.left_arm_errors = 0
        self.right_arm_errors = 0
        self.general_errors = 0
        self.max_arm_errors = 3  # Allow fewer errors per arm before marking as disconnected
        self.max_general_errors = 8  # Allow more general errors before full disconnection
        
        # Initial positions for safe shutdown
        self.initial_left_arm = np.array([2, 185, 175, 72, 4, 0])
        self.initial_right_arm = np.array([2, 185, 175, 72, -4, 0])
    
    def setup_robot_config(self) -> So100RobotConfig:
        """Create robot configuration based on current settings."""
        logger.info(f"Setting up robot config with ports: {self.config.follower_ports}")
        return So100RobotConfig(
            follower_arms={
                "left": FeetechMotorsBusConfig(port=self.config.follower_ports["left"], motors=COMMON_MOTORS.copy()),
                "right": FeetechMotorsBusConfig(port=self.config.follower_ports["right"], motors=COMMON_MOTORS.copy())
            },
            leader_arms={},  # Explicitly disable leader arms to prevent using default ports
            cameras={}
        )
    
    def connect(self) -> bool:
        """Connect to the robot."""
        if not self.config.enable_robot:
            logger.info("Robot disabled in configuration")
            return False
        
        try:
            robot_config = self.setup_robot_config()
            logger.info("Connecting to robot...")
            self.robot = ManipulatorRobot(robot_config)
            self.robot.connect()
            self.is_connected = True
            
            # Initially assume both arms are connected
            self.left_arm_connected = True
            self.right_arm_connected = True
            logger.info("✅ Robot connected successfully")
            
            # Read initial state
            self._read_initial_state()
            return True
            
        except Exception as e:
            logger.error(f"❌ Failed to connect to robot: {e}")
            self.is_connected = False
            self.left_arm_connected = False
            self.right_arm_connected = False
            return False
    
    def _read_initial_state(self):
        """Read initial joint state from robot."""
        try:
            observation = self.robot.capture_observation()
            if observation and "observation.state" in observation:
                initial_state = observation["observation.state"].cpu().numpy()
                logger.info(f"Initial joint state shape: {initial_state.shape}")
                logger.info(f"Initial joint state values: {initial_state}")
                
                if len(initial_state) == NUM_JOINTS * 2:  # Dual arm configuration
                    self.left_arm_angles = initial_state[:NUM_JOINTS].copy()
                    self.right_arm_angles = initial_state[NUM_JOINTS:].copy()
                    logger.info(f"Left arm initial: {self.left_arm_angles.round(1)}")
                    logger.info(f"Right arm initial: {self.right_arm_angles.round(1)}")
                elif len(initial_state) == NUM_JOINTS:  # Single arm
                    self.left_arm_angles = initial_state.copy()
                    self.right_arm_angles = initial_state.copy()
                    logger.info(f"Single arm state used for both: {initial_state.round(1)}")
                else:
                    logger.warning(f"Unexpected state length {len(initial_state)}, using defaults")
                    
        except Exception as e:
            logger.warning(f"Could not read initial robot state: {e}")
        
        # Ensure grippers start in open position regardless of robot's current state
        self.left_arm_angles[5] = GRIPPER_OPEN_ANGLE
        self.right_arm_angles[5] = GRIPPER_OPEN_ANGLE
        logger.info(f"Set grippers to open position ({GRIPPER_OPEN_ANGLE}°)")
    
    def setup_kinematics(self, physics_client, robot_ids: Dict, joint_indices: Dict, 
                        end_effector_link_indices: Dict, joint_limits_min_deg: np.ndarray, 
                        joint_limits_max_deg: np.ndarray):
        """Setup kinematics solvers using PyBullet components for both arms."""
        self.joint_limits_min_deg = joint_limits_min_deg.copy()
        self.joint_limits_max_deg = joint_limits_max_deg.copy()
        
        # Setup solvers for both arms
        for arm in ['left', 'right']:
            self.fk_solvers[arm] = ForwardKinematics(
                physics_client, robot_ids[arm], joint_indices[arm], end_effector_link_indices[arm]
            )
            
            self.ik_solvers[arm] = IKSolver(
                physics_client, robot_ids[arm], joint_indices[arm], end_effector_link_indices[arm],
                joint_limits_min_deg, joint_limits_max_deg, arm_name=arm
            )
        
        logger.info("Kinematics solvers initialized for both arms")
    
    def get_current_end_effector_position(self, arm: str) -> np.ndarray:
        """Get current end effector position for specified arm."""
        if arm == "left":
            angles = self.left_arm_angles
        elif arm == "right":
            angles = self.right_arm_angles
        else:
            raise ValueError(f"Invalid arm: {arm}")
        
        if self.fk_solvers[arm]:
            position, _ = self.fk_solvers[arm].compute(angles)
            return position
        else:
            default_position = np.array([0.2, 0.0, 0.15])
            return default_position
    
    def solve_ik(self, arm: str, target_position: np.ndarray, 
                 target_orientation: Optional[np.ndarray] = None) -> np.ndarray:
        """Solve inverse kinematics for specified arm."""
        if arm == "left":
            current_angles = self.left_arm_angles
        elif arm == "right":
            current_angles = self.right_arm_angles
        else:
            raise ValueError(f"Invalid arm: {arm}")
        
        if self.ik_solvers[arm]:
            return self.ik_solvers[arm].solve(target_position, target_orientation, current_angles)
        else:
            return current_angles[:3]  # Return current angles if no IK solver
    
    def clamp_joint_angles(self, joint_angles: np.ndarray) -> np.ndarray:
        """Clamp joint angles to safe limits with margins for problem joints."""
        # Create a copy to avoid modifying the original
        processed_angles = joint_angles.copy()
        
        # First, normalize angles that can wrap around (like shoulder_pan)
        # Check if first joint (shoulder_pan) is outside limits but can be wrapped
        shoulder_pan_idx = 0
        shoulder_pan_angle = processed_angles[shoulder_pan_idx]
        min_limit = self.joint_limits_min_deg[shoulder_pan_idx]  # -120.3°
        max_limit = self.joint_limits_max_deg[shoulder_pan_idx]  # +120.3°
        
        # Try to wrap the angle to an equivalent angle within limits
        if shoulder_pan_angle < min_limit or shoulder_pan_angle > max_limit:
            # Try wrapping by ±360°
            for offset in [-360.0, 360.0]:
                wrapped_angle = shoulder_pan_angle + offset
                if min_limit <= wrapped_angle <= max_limit:
                    logger.debug(f"Wrapped shoulder_pan from {shoulder_pan_angle:.1f}° to {wrapped_angle:.1f}°")
                    processed_angles[shoulder_pan_idx] = wrapped_angle
                    break
        
        # Apply standard joint limits to all joints
        return np.clip(processed_angles, self.joint_limits_min_deg, self.joint_limits_max_deg)
    
    def update_arm_angles(self, arm: str, ik_angles: np.ndarray, wrist_flex: float, wrist_roll: float, gripper: float):
        """Update joint angles for specified arm with IK solution and direct wrist/gripper control."""
        if arm == "left":
            target_angles = self.left_arm_angles
        elif arm == "right":
            target_angles = self.right_arm_angles
        else:
            raise ValueError(f"Invalid arm: {arm}")
        
        # Update first 3 joints with IK solution
        target_angles[:3] = ik_angles
        
        # Set wrist angles directly
        target_angles[3] = wrist_flex
        target_angles[4] = wrist_roll
        
        # Handle gripper separately (clamp to gripper limits)
        target_angles[5] = np.clip(gripper, GRIPPER_OPEN_ANGLE, GRIPPER_CLOSED_ANGLE)
        
        # Apply joint limits to all joints (except gripper which we handle specially)
        clamped_angles = self.clamp_joint_angles(target_angles)
        
        # Preserve gripper control (don't clamp gripper if it was set intentionally)
        clamped_angles[5] = target_angles[5]
        
        if arm == "left":
            self.left_arm_angles = clamped_angles
        else:
            self.right_arm_angles = clamped_angles
    
    def engage(self) -> bool:
        """Engage robot motors (start sending commands)."""
        if not self.is_connected:
            logger.warning("Cannot engage robot: not connected")
            return False
        
        self.is_engaged = True
        logger.info("🔌 Robot motors ENGAGED - commands will be sent")
        return True
    
    def disengage(self) -> bool:
        """Disengage robot motors (stop sending commands)."""
        if not self.is_connected:
            logger.info("Robot already disconnected")
            return True
        
        try:
            # Return to safe position before disengaging
            self.return_to_initial_position()
            
            # Disable torque
            self.disable_torque()
            
            self.is_engaged = False
            logger.info("🔌 Robot motors DISENGAGED - commands stopped")
            return True
            
        except Exception as e:
            logger.error(f"Error disengaging robot: {e}")
            return False
    
    def send_command(self) -> bool:
        """Send current joint angles to robot."""
        if not self.is_connected or not self.robot:
            return False
        
        # Only send commands if robot is engaged
        if not self.is_engaged:
            return False
        
        current_time = time.time()
        if current_time - self.last_send_time < self.config.send_interval:
            return False
        
        # Update arm connection status based on device file existence
        self.update_arm_connection_status()
        
        try:
            # Create concatenated command for dual arm robot
            concatenated_angles = np.concatenate([self.left_arm_angles, self.right_arm_angles])
            action_tensor = torch.from_numpy(concatenated_angles).float()
            
            self.robot.send_action(action_tensor)
            self.last_send_time = current_time
            
            # Reset ALL error counters on successful send
            self.left_arm_errors = 0
            self.right_arm_errors = 0
            self.general_errors = 0
            
            return True
            
        except Exception as e:
            error_str = str(e).lower()
            logger.error(f"Error sending robot command: {e}")
            
            # Try to detect which specific arm failed
            left_arm_error = ("ttySO100red" in error_str or "ttys100red" in error_str or 
                            "left" in error_str.lower())
            right_arm_error = ("ttySO100blue" in error_str or "ttys100blue" in error_str or 
                             "right" in error_str.lower())
            
            if left_arm_error:
                self.left_arm_errors += 1
                logger.warning(f"⚠️  LEFT ARM communication error (count: {self.left_arm_errors})")
                    
            elif right_arm_error:
                self.right_arm_errors += 1
                logger.warning(f"⚠️  RIGHT ARM communication error (count: {self.right_arm_errors})")
                    
            else:
                # General error - couldn't determine which arm
                self.general_errors += 1
                logger.warning(f"⚠️  General robot communication error (count: {self.general_errors})")
            
            return False
    
    def set_gripper(self, arm: str, closed: bool):
        """Set gripper state for specified arm."""
        angle = GRIPPER_CLOSED_ANGLE if closed else GRIPPER_OPEN_ANGLE
        
        if arm == "left":
            self.left_arm_angles[5] = angle
        elif arm == "right":
            self.right_arm_angles[5] = angle
        else:
            raise ValueError(f"Invalid arm: {arm}")
    
    def get_arm_angles(self, arm: str) -> np.ndarray:
        """Get current joint angles for specified arm."""
        if arm == "left":
            angles = self.left_arm_angles.copy()
        elif arm == "right":
            angles = self.right_arm_angles.copy()
        else:
            raise ValueError(f"Invalid arm: {arm}")
        
        return angles
    
    def return_to_initial_position(self):
        """Move robot arms back to initial positions."""
        if not self.is_connected:
            return
        
        try:
            logger.info("🏠 Moving robot arms back to initial positions...")
            
            # Set to initial positions
            self.left_arm_angles = self.initial_left_arm.copy()
            self.right_arm_angles = self.initial_right_arm.copy()
            
            # Force send command immediately, bypassing engagement and timing checks
            if self.robot:
                concatenated_angles = np.concatenate([self.left_arm_angles, self.right_arm_angles])
                action_tensor = torch.from_numpy(concatenated_angles).float()
                self.robot.send_action(action_tensor)
                logger.info("🏠 Commanded return to initial positions")
                
                # Wait for movement - give arms time to reach home position
                time.sleep(1.5)
            else:
                logger.warning("🏠 No robot interface available for home positioning")
            
        except Exception as e:
            logger.error(f"Error moving to initial position: {e}")
    
    def disable_torque(self):
        """Disable torque on all robot joints."""
        if not self.is_connected or not self.robot:
            return
        
        try:
            logger.info("Disabling torque on follower motors...")
            if hasattr(self.robot, 'follower_arms'):
                for arm_name, arm in self.robot.follower_arms.items():
                    try:
                        arm.write("Torque_Enable", 0)
                        logger.info(f"Disabled torque on arm: {arm_name}")
                    except Exception as e:
                        logger.warning(f"Could not disable torque on arm {arm_name}: {e}")
        except Exception as e:
            logger.error(f"Error disabling torque: {e}")
    
    def disconnect(self):
        """Disconnect from robot."""
        if not self.is_connected:
            return
        
        try:
            # Return to safe position first
            self.return_to_initial_position()
            
            # Disable torque
            self.disable_torque()
            
            # Disconnect
            self.robot.disconnect()
            self.is_connected = False
            logger.info("Robot disconnected")
            
        except Exception as e:
            logger.error(f"Error disconnecting robot: {e}")
    
    def get_arm_connection_status(self, arm: str) -> bool:
        """Get connection status for specific arm based on device file existence."""
        # Only check device file existence - ignore overall robot connection status
        if arm == "left":
            device_path = self.config.follower_ports["left"]
            return os.path.exists(device_path)
        elif arm == "right":
            device_path = self.config.follower_ports["right"] 
            return os.path.exists(device_path)
        else:
            return False

    def update_arm_connection_status(self):
        """Update individual arm connection status based on device file existence."""
        if self.is_connected:
            self.left_arm_connected = os.path.exists(self.config.follower_ports["left"])
            self.right_arm_connected = os.path.exists(self.config.follower_ports["right"])
    
    @property
    def status(self) -> Dict:
        """Get robot status information."""
        return {
            "connected": self.is_connected,
            "left_arm_connected": self.left_arm_connected,
            "right_arm_connected": self.right_arm_connected,
            "left_arm_angles": self.left_arm_angles.tolist(),
            "right_arm_angles": self.right_arm_angles.tolist(),
            "joint_limits_min": self.joint_limits_min_deg.tolist(),
            "joint_limits_max": self.joint_limits_max_deg.tolist(),
        } 