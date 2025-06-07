"""
Robot interface module for the SO100 teleoperation system.
Provides a clean wrapper around ManipulatorRobot with safety checks and convenience methods.
"""

import numpy as np
import torch
import time
import logging
from typing import Optional, Dict, Tuple

from lerobot.common.robot_devices.robots.manipulator import ManipulatorRobot
from lerobot.common.robot_devices.robots.configs import So100RobotConfig
from lerobot.common.robot_devices.motors.configs import FeetechMotorsBusConfig
from lerobot.common.robot_devices.utils import RobotDeviceNotConnectedError

from ..config import (
    TeleopConfig, COMMON_MOTORS, NUM_JOINTS, JOINT_NAMES,
    GRIPPER_OPEN_ANGLE, GRIPPER_CLOSED_ANGLE, 
    WRIST_FLEX_INDEX, ELBOW_LOWER_MARGIN_DEG, WRIST_FLEX_MARGINS_DEG
)
from .kinematics import ForwardKinematics, IKSolver

logger = logging.getLogger(__name__)


class RobotInterface:
    """High-level interface for SO100 robot control with safety features."""
    
    def __init__(self, config: TeleopConfig):
        self.config = config
        self.robot = None
        self.is_connected = False
        
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
            logger.info("✅ Robot connected successfully")
            
            # Read initial state
            self._read_initial_state()
            return True
            
        except Exception as e:
            logger.error(f"❌ Failed to connect to robot: {e}")
            self.is_connected = False
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
                joint_limits_min_deg, joint_limits_max_deg
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
            return np.array([0.2, 0.0, 0.15])  # Default position
    
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
        # Start with standard clamping
        clamped = np.clip(joint_angles, self.joint_limits_min_deg, self.joint_limits_max_deg)
        
        # Apply elbow margin to prevent hyperextension (joint index 2 = elbow_flex)
        elbow_idx = 2  # elbow_flex index
        elbow_lower_margin = ELBOW_LOWER_MARGIN_DEG  # 5.0 from config
        if elbow_lower_margin > 0:
            elbow_min_with_margin = self.joint_limits_min_deg[elbow_idx] + elbow_lower_margin
            clamped[elbow_idx] = np.clip(joint_angles[elbow_idx], elbow_min_with_margin, self.joint_limits_max_deg[elbow_idx])
        
        # Apply special margins to wrist_flex to prevent getting stuck at limits
        wrist_flex_margin = WRIST_FLEX_MARGINS_DEG
        wrist_flex_min = self.joint_limits_min_deg[WRIST_FLEX_INDEX] + wrist_flex_margin["lower"]
        wrist_flex_max = self.joint_limits_max_deg[WRIST_FLEX_INDEX] - wrist_flex_margin["upper"]
        
        clamped[WRIST_FLEX_INDEX] = np.clip(
            joint_angles[WRIST_FLEX_INDEX], 
            wrist_flex_min, 
            wrist_flex_max
        )
        
        return clamped
    
    def clamp_wrist_angle(self, angle: float, joint_name: str) -> float:
        """Clamp a single wrist angle with margins to prevent getting stuck at limits."""
        if joint_name == "wrist_flex":
            joint_idx = WRIST_FLEX_INDEX
            margin = WRIST_FLEX_MARGINS_DEG
            min_limit = self.joint_limits_min_deg[joint_idx] + margin["lower"]
            max_limit = self.joint_limits_max_deg[joint_idx] - margin["upper"]
            return np.clip(angle, min_limit, max_limit)
        else:
            # For other joints, use standard limits
            if joint_name == "wrist_roll":
                joint_idx = 4  # WRIST_ROLL_INDEX
            elif joint_name == "gripper":
                joint_idx = 5  # GRIPPER_INDEX
            else:
                return angle
            
            return np.clip(angle, self.joint_limits_min_deg[joint_idx], self.joint_limits_max_deg[joint_idx])
    
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
        
        # Apply margins to wrist angles before setting them
        target_angles[3] = self.clamp_wrist_angle(wrist_flex, "wrist_flex")
        target_angles[4] = self.clamp_wrist_angle(wrist_roll, "wrist_roll")
        target_angles[5] = self.clamp_wrist_angle(gripper, "gripper")
        
        # Apply final clamping (this will also handle the IK joints)
        clamped_angles = self.clamp_joint_angles(target_angles)
        
        # Preserve gripper control (don't clamp gripper if it was set intentionally)
        clamped_angles[5] = target_angles[5]
        
        if arm == "left":
            self.left_arm_angles = clamped_angles
        else:
            self.right_arm_angles = clamped_angles
    
    def send_command(self) -> bool:
        """Send current joint angles to robot."""
        if not self.is_connected or not self.robot:
            return False
        
        current_time = time.time()
        if current_time - self.last_send_time < self.config.send_interval:
            return False
        
        try:
            # Create concatenated command for dual arm robot
            concatenated_angles = np.concatenate([self.left_arm_angles, self.right_arm_angles])
            action_tensor = torch.from_numpy(concatenated_angles).float()
            
            self.robot.send_action(action_tensor)
            self.last_send_time = current_time
            return True
            
        except Exception as e:
            logger.error(f"Error sending robot command: {e}")
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
            return self.left_arm_angles.copy()
        elif arm == "right":
            return self.right_arm_angles.copy()
        else:
            raise ValueError(f"Invalid arm: {arm}")
    
    def return_to_initial_position(self):
        """Move robot arms back to initial positions."""
        if not self.is_connected:
            return
        
        try:
            logger.info("🏠 Moving robot arms back to initial positions...")
            
            # Set to initial positions
            self.left_arm_angles = self.initial_left_arm.copy()
            self.right_arm_angles = self.initial_right_arm.copy()
            
            # Send command
            self.send_command()
            logger.info("🏠 Commanded return to initial positions")
            
            # Wait for movement
            time.sleep(1.0)
            
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
    
    @property
    def status(self) -> Dict:
        """Get robot status information."""
        return {
            "connected": self.is_connected,
            "left_arm_angles": self.left_arm_angles.tolist(),
            "right_arm_angles": self.right_arm_angles.tolist(),
            "joint_limits_min": self.joint_limits_min_deg.tolist(),
            "joint_limits_max": self.joint_limits_max_deg.tolist(),
        } 