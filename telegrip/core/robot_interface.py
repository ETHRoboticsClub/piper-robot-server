"""
Robot interface module for the SO100 teleoperation system.
Provides a clean wrapper around robot devices with safety checks and convenience methods.
"""

import contextlib
import logging
import os
import sys
import time
from typing import Dict, Tuple

import numpy as np
import pinocchio as pin
from piper_control import piper_connect

from ..config import NUM_JOINTS, TelegripConfig
from .geometry import transform2pose
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
    """High-level interface for SO100 robot control with safety features."""

    def __init__(self, config: TelegripConfig):
        self.config = config
        self.left_robot = None
        self.right_robot = None
        self.is_connected = False
        self.is_engaged = True  # New state for motor engagement # TODO: set to False

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
        self.fk_solvers = {"left": None, "right": None}
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
        self.initial_left_arm = np.array([0, 0, 0, 0, 0, 0, 0])
        self.initial_right_arm = np.array([0, 0, 0, 0, 0, 0, 0])

    def setup_robot_configs(self) -> Tuple[PiperConfig, PiperConfig]:
        """Create robot configurations for both arms."""

        left_config = PiperConfig(port="can0", cameras={})
        # Set the robot name for calibration file lookup
        left_config.id = "left_follower"

        right_config = PiperConfig(port="can1", cameras={})
        # Set the robot name for calibration file lookup
        right_config.id = "right_follower"

        return left_config, right_config

    def connect(self) -> bool:
        """Connect to robot hardware."""
        if self.is_connected:
            logger.info("Robot interface already connected")
            return True

        if not self.config.enable_robot:
            logger.info("Robot interface disabled in config")
            self.is_connected = True  # Mark as "connected" for testing
            return True

        # Setup suppression if requested
        should_suppress = (
            self.config.log_level == "warning"
            or self.config.log_level == "critical"
            or self.config.log_level == "error"
        )

        try:
            left_config, right_config = self.setup_robot_configs()
            if not should_suppress:
                logger.info("Connecting to robot...")

            # Connect left arm
            try:
                piper_connect.activate()
                if should_suppress:
                    with suppress_stdout_stderr():
                        self.left_robot = Piper(left_config)
                        self.left_robot.connect()
                else:
                    self.left_robot = Piper(left_config)
                    self.left_robot.connect()
                self.left_arm_connected = True
                logger.info("✅ Left arm connected successfully")
            except Exception as e:
                logger.error(f"❌ Left arm connection failed: {e}")
                self.left_arm_connected = False

            # Connect right arm
            try:
                if should_suppress:
                    with suppress_stdout_stderr():
                        self.right_robot = Piper(right_config)
                        self.right_robot.connect()
                else:
                    self.right_robot = Piper(right_config)
                    self.right_robot.connect()
                self.right_arm_connected = True
                logger.info("✅ Right arm connected successfully")
            except Exception as e:
                logger.error(f"❌ Right arm connection failed: {e}")
                self.right_arm_connected = False

            # Mark as connected if at least one arm is connected
            self.is_connected = self.left_arm_connected or self.right_arm_connected

            if self.is_connected:
                # Initialize joint states
                self._read_initial_state()
                logger.info(
                    f"🤖 Robot interface connected: Left={self.left_arm_connected}, Right={self.right_arm_connected}"
                )
            else:
                logger.error("❌ Failed to connect any robot arms")

            return self.is_connected

        except Exception as e:
            logger.error(f"❌ Robot connection failed with exception: {e}")
            self.is_connected = False
            return False

    def _read_initial_state(self):
        """Read initial joint state from robot."""
        try:
            if self.left_robot and self.left_arm_connected:
                observation = self.left_robot.get_observation()
                if observation:
                    # Extract joint positions from observation
                    self.left_arm_angles = np.array(
                        [
                            observation["joint_0.pos"],
                            observation["joint_1.pos"],
                            observation["joint_2.pos"],
                            observation["joint_3.pos"],
                            observation["joint_4.pos"],
                            observation["joint_5.pos"],
                            observation["joint_6.pos"],
                        ]
                    )
                    logger.info(f"Left arm initial state: {self.left_arm_angles.round(1)}")

            if self.right_robot and self.right_arm_connected:
                observation = self.right_robot.get_observation()
                if observation:
                    # Extract joint positions from observation
                    self.right_arm_angles = np.array(
                        [
                            observation["joint_0.pos"],
                            observation["joint_1.pos"],
                            observation["joint_2.pos"],
                            observation["joint_3.pos"],
                            observation["joint_4.pos"],
                            observation["joint_5.pos"],
                            observation["joint_6.pos"],
                        ]
                    )
                    logger.info(f"Right arm initial state: {self.right_arm_angles.round(1)}")

        except Exception as e:
            logger.error(f"Error reading initial state: {e}")

    def setup_kinematics(self):
        """Setup kinematics solvers using PyBullet components for both arms."""
        # Setup solvers for both arms
        for arm in ["left", "right"]:
            self.ik_solvers[arm] = Arm_IK(self.config.urdf_path)
        logger.info("Kinematics solvers initialized for both arms")

    def solve_ik(self, arm: str, target_pose: np.ndarray) -> np.ndarray:
        """Solve inverse kinematics for specified arm."""
        position, quaternion = transform2pose(target_pose)
        # TODO: check if it is xyzw or wxyz
        target = pin.SE3(
            pin.Quaternion(quaternion[3], quaternion[0], quaternion[1], quaternion[2]),
            position,
        )
        sol_q, tau_ff, is_collision = self.ik_solvers[arm].ik_fun(target.homogeneous, 0)
        return sol_q

    def update_arm_angles(self, arm: str, joint_angles: np.ndarray):
        """Update joint angles for specified arm."""
        if arm == "left":
            self.left_arm_angles = joint_angles
        else:
            self.right_arm_angles = joint_angles

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
        """Send current joint angles to robot using dictionary format."""
        if not self.is_connected or not self.is_engaged:
            return False

        current_time = time.time()
        if current_time - self.last_send_time < self.config.send_interval:
            return True  # Don't send too frequently

        try:
            # Send commands with dictionary format - no joint direction mapping
            success = True

            # Send left arm command
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
                    self.left_robot.send_action(action_dict)
                except Exception as e:
                    logger.error(f"Error sending left arm command: {e}")
                    self.left_arm_errors += 1
                    if self.left_arm_errors > self.max_arm_errors:
                        self.left_arm_connected = False
                        logger.error("❌ Left arm disconnected due to repeated errors")
                    success = False

            # Send right arm command
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

    def get_arm_angles(self, arm: str) -> np.ndarray:
        """Get current joint angles for specified arm."""
        if arm == "left":
            angles = self.left_arm_angles.copy()
        elif arm == "right":
            angles = self.right_arm_angles.copy()
        else:
            raise ValueError(f"Invalid arm: {arm}")

        return angles

    def get_arm_angles_for_visualization(self, arm: str) -> np.ndarray:
        """Get current joint angles for specified arm, for PyBullet visualization."""
        # Return raw angles without any correction for proper diagnosis
        return self.get_arm_angles(arm)

    def get_actual_arm_angles(self, arm: str) -> np.ndarray:
        """Get actual joint angles from robot hardware (not commanded angles)."""
        try:
            if arm == "left" and self.left_robot and self.left_arm_connected:
                observation = self.left_robot.get_observation()
                if observation:
                    return np.array(
                        [
                            observation["joint_0.pos"],
                            observation["joint_1.pos"],
                            observation["joint_2.pos"],
                            observation["joint_3.pos"],
                            observation["joint_4.pos"],
                            observation["joint_5.pos"],
                            observation["joint_6.pos"],
                        ]
                    )
            elif arm == "right" and self.right_robot and self.right_arm_connected:
                observation = self.right_robot.get_observation()
                if observation:
                    return np.array(
                        [
                            observation["joint_0.pos"],
                            observation["joint_1.pos"],
                            observation["joint_2.pos"],
                            observation["joint_3.pos"],
                            observation["joint_4.pos"],
                            observation["joint_5.pos"],
                            observation["joint_6.pos"],
                        ]
                    )
        except Exception as e:
            logger.debug(f"Error reading actual arm angles for {arm}: {e}")

        # Fallback to commanded angles if we can't read actual angles
        return self.get_arm_angles(arm)

    def return_to_initial_position(self):
        """Return both arms to initial position."""
        logger.info("⏪ Returning robot to initial position...")

        try:
            # Set initial positions - no direction mapping
            self.left_arm_angles = self.initial_left_arm.copy()
            self.right_arm_angles = self.initial_right_arm.copy()

            # Send commands for a few iterations to ensure movement
            for i in range(10):
                self.send_command()
                time.sleep(0.1)

            logger.info("✅ Robot returned to initial position")
        except Exception as e:
            logger.error(f"Error returning to initial position: {e}")

    def disable_torque(self):
        """Disable torque on all robot joints."""
        if not self.is_connected:
            return

        try:
            logger.info("Disabling torque on follower motors...")

            # The new SO100Follower automatically handles torque disable on disconnect
            # We don't need to manually disable torque as it's handled by the robot class
            logger.info("Torque will be disabled automatically on disconnect")

        except Exception as e:
            logger.error(f"Error disabling torque: {e}")

    def disconnect(self):
        """Disconnect from robot hardware."""
        if not self.is_connected:
            return

        logger.info("Disconnecting from robot...")

        # Return to initial positions if engaged
        if self.is_engaged:
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
        self.is_engaged = False
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
