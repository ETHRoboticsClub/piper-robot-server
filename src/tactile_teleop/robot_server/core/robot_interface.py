"""
Robot interface module.
Provides a clean wrapper around robot devices with safety checks and convenience methods.
"""

import contextlib
import logging
import os
import sys
import time
from typing import Any, Tuple

import numpy as np
import pinocchio as pin

from tactile_teleop.config import NUM_JOINTS, TelegripConfig

from .geometry import transform2pose, xyzrpy2transform
from .kinematics import Arm_IK
from .piper import Piper, PiperConfig

logger = logging.getLogger(__name__)


@contextlib.contextmanager
def suppress_stdout_stderr():
    """Context manager to suppress stdout and stderr output at the file descriptor level."""
    # Save original file descriptors
    stdout_fd = sys.stdout.fileno()
    stderr_fd = sys.stderr.fileno()

    # Save original file descriptors
    saved_stdout_fd = os.dup(stdout_fd)
    saved_stderr_fd = os.dup(stderr_fd)

    try:
        # Open devnull
        devnull_fd = os.open(os.devnull, os.O_WRONLY)

        # Redirect stdout and stderr to devnull
        os.dup2(devnull_fd, stdout_fd)
        os.dup2(devnull_fd, stderr_fd)

        yield

    finally:
        # Restore original file descriptors
        os.dup2(saved_stdout_fd, stdout_fd)
        os.dup2(saved_stderr_fd, stderr_fd)

        # Close saved file descriptors
        os.close(saved_stdout_fd)
        os.close(saved_stderr_fd)
        os.close(devnull_fd)


class RobotInterface:
    """High-level interface for robot control with safety features."""

    def __init__(self, config: TelegripConfig, robot_enabled: bool = False):
        self.config = config
        self.left_robot = None
        self.right_robot = None
        self.is_enabled = robot_enabled
        self.is_connected = False

        # Individual arm connection status
        self.left_arm_connected = False
        self.right_arm_connected = False

        # Joint state
        self.left_arm_angles = np.zeros(NUM_JOINTS)
        self.right_arm_angles = np.zeros(NUM_JOINTS)

        self.ik_solvers = {"left": None, "right": None}

        # Control timing
        self.last_send_time = 0

        # Error tracking - separate for each arm
        self.left_arm_errors = 0
        self.right_arm_errors = 0
        self.general_errors = 0
        self.max_arm_errors = 3  # Allow fewer errors per arm before marking as disconnected
        self.max_general_errors = 8  # Allow more general errors before full disconnection

        # Initial positions for safe shutdown - restored original values
        self.initial_left_arm = xyzrpy2transform(0.19, 0.0, 0.2, 0, 1.57, 0)
        self.initial_right_arm = xyzrpy2transform(0.19, 0.0, 0.2, 0, 1.57, 0)

    def setup_robot_configs(self) -> Tuple[PiperConfig, PiperConfig]:
        """Create robot configurations for both arms."""

        left_config = PiperConfig(port="left_piper", id="left_follower")
        right_config = PiperConfig(port="right_piper", id="right_follower")

        return left_config, right_config

    def connect(self) -> bool:
        """Connect to robot hardware."""
        if self.is_connected:
            logger.info("Robot interface already connected")
            return True

        if not self.is_enabled:
            logger.info("Robot control is not enabled")
            return False

        try:
            left_config, right_config = self.setup_robot_configs()
            logger.info("Connecting to robot...")

            # Connect left arm - always suppress low-level CAN debug output
            try:
                with suppress_stdout_stderr():
                    self.left_robot = Piper(left_config)
                    self.left_robot.connect()
                self.left_arm_connected = True
                logger.info("✅ Left arm connected successfully")
            except Exception as e:
                logger.error(f"❌ Left arm connection failed: {e}")
                self.left_arm_connected = False

            # Connect right arm - always suppress low-level CAN debug output
            try:
                with suppress_stdout_stderr():
                    self.right_robot = Piper(right_config)
                    self.right_robot.connect()
                self.right_arm_connected = True
                logger.info("✅ Right arm connected successfully")
            except Exception as e:
                logger.error(f"❌ Right arm connection failed: {e}")
                self.right_arm_connected = False

            # Mark as connected if at least one arm is connected
            self.is_connected = self.left_arm_connected and self.right_arm_connected

            if not self.is_connected:
                logger.error("❌ Failed to connect any robot arms")

            return self.is_connected

        except Exception as e:
            logger.error(f"❌ Robot connection failed with exception: {e}")
            self.is_connected = False
            return False

    def setup_kinematics(self):
        """Setup kinematics solvers using PyBullet components for both arms."""
        # Setup solvers for both arms
        ground_height = self.config.ground_height
        for arm in ["left", "right"]:
            self.ik_solvers[arm] = Arm_IK(self.config.urdf_path, ground_height)
        logger.info("Kinematics solvers initialized for both arms with ground plane at height %.3f", ground_height)

    def get_end_effector_transform(self, arm: str) -> np.ndarray:
        """Get end effector pose for specified arm.

        Returns:
            np.ndarray: 4x4 transform matrix.
        """
        if arm == "left":
            return (
                self.left_robot.get_end_effector_transform()
                if self.left_robot
                else xyzrpy2transform(0.19, 0.0, 0.2, 0, 1.57, 0)
            )
        elif arm == "right":
            return (
                self.right_robot.get_end_effector_transform()
                if self.right_robot
                else xyzrpy2transform(0.19, 0.0, 0.2, 0, 1.57, 0)
            )
        else:
            raise ValueError(f"Invalid arm: {arm}")

    def solve_ik(self, arm: str, target_pose: np.ndarray, visualize: bool) -> np.ndarray:
        """Solve inverse kinematics for specified arm."""
        position, quaternion = transform2pose(target_pose)
        # transform2pose returns XYZW, but pin.Quaternion expects WXYZ
        target = pin.SE3(
            pin.Quaternion(quaternion[3], quaternion[0], quaternion[1], quaternion[2]),
            position,
        )
        sol_q, is_collision = self.ik_solvers[arm].ik_fun(target.homogeneous, 0, visualize=visualize)
        return sol_q, is_collision

    def update_arm_angles(self, arm: str, joint_angles: np.ndarray):
        """Update joint angles for specified arm."""
        if arm == "left":
            self.left_arm_angles = joint_angles
        else:
            self.right_arm_angles = joint_angles

    def send_command(self) -> bool:
        """Send current joint angles to robot using dictionary format."""
        if not self.is_connected or not self.is_enabled:
            return False

        current_time = time.time()
        if current_time - self.last_send_time < self.config.send_interval:
            return True  # Don't send too frequently

        try:
            # Send commands with dictionary format - no joint direction mapping
            success = True

            # Send left arm command - suppress low-level CAN debug output
            if self.left_robot and self.left_arm_connected:
                try:
                    action_dict = {
                        "joint_0.pos": float(self.left_arm_angles[0]),
                        "joint_1.pos": float(self.left_arm_angles[1]),
                        "joint_2.pos": float(self.left_arm_angles[2]),
                        "joint_3.pos": float(self.left_arm_angles[3]),
                        "joint_4.pos": float(self.left_arm_angles[4]),
                        "joint_5.pos": float(self.left_arm_angles[5]),
                        "joint_6.pos": float(self.left_arm_angles[6]),
                    }
                    with suppress_stdout_stderr():
                        self.left_robot.send_action(action_dict)
                except Exception as e:
                    logger.error(f"Error sending left arm command: {e}")
                    self.left_arm_errors += 1
                    if self.left_arm_errors > self.max_arm_errors:
                        self.left_arm_connected = False
                        logger.error("❌ Left arm disconnected due to repeated errors")
                    success = False

            # Send right arm command - suppress low-level CAN debug output
            if self.right_robot and self.right_arm_connected:
                try:
                    action_dict = {
                        "joint_0.pos": float(self.right_arm_angles[0]),
                        "joint_1.pos": float(self.right_arm_angles[1]),
                        "joint_2.pos": float(self.right_arm_angles[2]),
                        "joint_3.pos": float(self.right_arm_angles[3]),
                        "joint_4.pos": float(self.right_arm_angles[4]),
                        "joint_5.pos": float(self.right_arm_angles[5]),
                        "joint_6.pos": float(self.right_arm_angles[6]),
                    }
                    with suppress_stdout_stderr():
                        self.right_robot.send_action(action_dict)
                except Exception as e:
                    logger.error(f"Error sending right arm command: {e}")
                    self.right_arm_errors += 1
                    if self.right_arm_errors > self.max_arm_errors:
                        self.right_arm_connected = False
                        logger.error("❌ Right arm disconnected due to repeated errors")
                    success = False

            self.last_send_time = current_time
            return success

        except Exception as e:
            logger.error(f"Error sending robot command: {e}")
            self.general_errors += 1
            if self.general_errors > self.max_general_errors:
                self.is_connected = False
                logger.error("❌ Robot interface disconnected due to repeated errors")
            return False

    def set_gripper(self, arm: str, closed: bool):
        """Set gripper state for specified arm."""
        angle = 0.0 if closed else 0.07

        if arm == "left":
            self.left_arm_angles[6] = angle
        elif arm == "right":
            self.right_arm_angles[6] = angle
        else:
            raise ValueError(f"Invalid arm: {arm}")

    def return_to_initial_position(self):
        """Return both arms to initial position."""
        logger.info("⏪ Returning robot to initial position...")

        left_arm_angles, left_collision = self.solve_ik("left", self.initial_left_arm, visualize=True)
        right_arm_angles, right_collision = self.solve_ik("right", self.initial_right_arm, visualize=True)

        if left_collision:
            logger.error("❌ Left arm collision detected during return to initial position, setting angles to zero")
            self.left_arm_angles = np.zeros(NUM_JOINTS)

        if right_collision:
            logger.error("❌ Right arm collision detected during return to initial position, setting angles to zero")
            self.right_arm_angles = np.zeros(NUM_JOINTS)

        self.left_arm_angles = np.concatenate((left_arm_angles, [0.0]))
        self.right_arm_angles = np.concatenate((right_arm_angles, [0.0]))

        try:
            # Send commands for a few iterations to ensure movement
            for i in range(10):
                self.send_command()
                time.sleep(0.1)

            logger.info("✅ Robot returned to initial position")
        except Exception as e:
            logger.error(f"Error returning to initial position: {e}")

    def disconnect(self):
        """Disconnect from robot hardware."""
        if not self.is_connected:
            return

        logger.info("Disconnecting from robot...")

        # Return to initial positions if engaged
        if self.is_enabled:
            try:
                self.return_to_initial_position()
            except Exception as e:
                logger.error(f"Error returning to initial position: {e}")

        # Disconnect both arms
        if self.left_robot:
            try:
                self.left_robot.disconnect()
            except Exception as e:
                logger.error(f"Error disconnecting left arm: {e}")
            self.left_robot = None

        if self.right_robot:
            try:
                self.right_robot.disconnect()
            except Exception as e:
                logger.error(f"Error disconnecting right arm: {e}")
            self.right_robot = None

        self.is_connected = False
        self.is_enabled = False
        self.left_arm_connected = False
        self.right_arm_connected = False
        logger.info("🔌 Robot disconnected")

    def get_arm_connection_status(self, arm: str) -> bool:
        """Get connection status for specific arm based on device file existence."""
        # Only check device file existence - ignore overall robot connection status
        if arm == "left":
            return self.left_robot.is_connected if self.left_robot else False
        elif arm == "right":
            return self.right_robot.is_connected if self.right_robot else False
        else:
            return False

    def update_arm_connection_status(self):
        """Update individual arm connection status based on device file existence."""
        if self.is_connected:
            self.left_arm_connected = self.left_robot.is_connected if self.left_robot else False
            self.right_arm_connected = self.right_robot.is_connected if self.right_robot else False

    def get_observation(self) -> dict[str, Any]:
        """Get observation from robot."""
        return {
            "left_arm_obs": self.left_robot.get_observation() if self.left_robot else None,
            "right_arm_obs": self.right_robot.get_observation() if self.right_robot else None,
        }
