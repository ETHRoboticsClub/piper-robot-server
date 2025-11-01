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

from piper_teleop.config import NUM_JOINTS, TelegripConfig

from .geometry import transform2pose, xyzrpy2transform
from .kinematics import Arm_IK
from .piper import Piper, PiperConfig
from .sim_interface import PyBulletSimInterface

logger = logging.getLogger(__name__)


def arm_angles_to_action_dict(arm_angles):
    left_action_dict = {
        "joint_0.pos": float(arm_angles[6]),
        "joint_1.pos": float(arm_angles[7]),
        "joint_2.pos": float(arm_angles[8]),
        "joint_3.pos": float(arm_angles[9]),
        "joint_4.pos": float(arm_angles[10]),
        "joint_5.pos": float(arm_angles[11]),
        "joint_6.pos": float(arm_angles[12]),
    }
    right_action_dict = {
        "joint_0.pos": float(arm_angles[0]),
        "joint_1.pos": float(arm_angles[1]),
        "joint_2.pos": float(arm_angles[2]),
        "joint_3.pos": float(arm_angles[3]),
        "joint_4.pos": float(arm_angles[4]),
        "joint_5.pos": float(arm_angles[5]),
        "joint_6.pos": float(arm_angles[13]),
    }
    return {"left": left_action_dict, "right": right_action_dict}


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

    def __init__(self, config: TelegripConfig):
        self.config = config
        self.left_robot = None
        self.right_robot = None
        # Enable robot control when hardware enabled OR when running simulation
        self.is_enabled = config.enable_robot or config.run_in_newton
        self.is_connected = False

        # Individual arm connection status
        self.left_arm_connected = False
        self.right_arm_connected = False

        # Joint state
        self.arm_angles = np.zeros(NUM_JOINTS * 2)
        # arm angles
        # 0-5: left arm joints
        # 6-11: right arm joints
        # 12: left gripper
        # 13: right gripper

        self.ik_solver = None

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
        self.initial_right_arm = xyzrpy2transform(0.19, -0.57, 0.2, 0, 1.57, 0)

    def setup_robot_configs(self) -> Tuple[PiperConfig, PiperConfig]:
        """Create robot configurations for both arms."""
        # Determine if we're using simulation
        use_sim = self.config.run_in_newton or not self.is_enabled
        sim_type = "pybullet"
        # If URDF contains multiple arms (e.g., "dual_piper.urdf"), prefer a single shared simulator instance
        urdf_basename = os.path.basename(self.config.urdf_path or "").lower()
        sim_shared = False
        if "dual" in urdf_basename or "dual_piper" in urdf_basename:
            sim_shared = True

        left_config = PiperConfig(
            port="left_piper", 
            id="left_follower",
            use_sim=use_sim,
            sim_type=sim_type,
            sim_gui=self.config.enable_visualization,
            urdf_path=self.config.get_absolute_urdf_path()
        )
        right_config = PiperConfig(
            port="right_piper", 
            id="right_follower",
            use_sim=use_sim,
            sim_type=sim_type,
            sim_gui=False,  # Only show GUI for one arm to avoid clutter
            urdf_path=self.config.get_absolute_urdf_path()
        )

        # Mark configs as sharing a single simulator instance when appropriate
        left_config.sim_shared = sim_shared
        right_config.sim_shared = sim_shared

        # Ensure arm side identifiers are set correctly so per-arm sim calls work
        left_config.arm_side = "left"
        right_config.arm_side = "right"

        return left_config, right_config

    def connect(self) -> bool:
        """Connect to robot hardware or simulation."""
        if self.is_connected:
            logger.info("Robot interface already connected")
            return True

        try:
            left_config, right_config = self.setup_robot_configs()
            
            # Check if using simulation
            use_sim = left_config.use_sim
            
            if use_sim:
                logger.info(f"Connecting to {left_config.sim_type} simulation...")
            else:
                logger.info("Connecting to robot hardware...")

            # Connect left arm
            try:
                if use_sim and left_config.sim_shared:
                    # Create a single shared simulator instance and inject into both Piper wrappers
                    try:
                        sim = PyBulletSimInterface(
                            urdf_path=left_config.urdf_path,
                            use_gui=left_config.sim_gui,
                            robot_base_position=None,
                        )
                        sim.connect()
                        self.left_robot = Piper(left_config, sdk_instance=sim)
                        logger.info("✅ Left arm (simulation) connected (shared simulator)")
                    except Exception as e:
                        logger.error(f"❌ Failed to initialize shared simulation: {e}")
                        raise
                elif use_sim:
                    self.left_robot = Piper(left_config)
                    self.left_robot.connect()
                    logger.info("✅ Left arm (simulation) connected successfully")
                else:
                    with suppress_stdout_stderr():
                        self.left_robot = Piper(left_config)
                        self.left_robot.connect()
                    logger.info("✅ Left arm (hardware) connected successfully")
                self.left_arm_connected = True
            except Exception as e:
                logger.error(f"❌ Left arm connection failed: {e}")
                self.left_arm_connected = False

            # Connect right arm
            try:
                if use_sim and left_config.sim_shared:
                    # Shared simulator: create a second Piper wrapper that uses the same sim instance
                    sim_instance = self.left_robot.sdk
                    self.right_robot = Piper(right_config, sdk_instance=sim_instance)
                    logger.info("ℹ️ Right arm connected to shared simulation instance")
                elif use_sim:
                    self.right_robot = Piper(right_config)
                    self.right_robot.connect()
                    logger.info("✅ Right arm (simulation) connected successfully")
                else:
                    with suppress_stdout_stderr():
                        self.right_robot = Piper(right_config)
                        self.right_robot.connect()
                    logger.info("✅ Right arm (hardware) connected successfully")
                self.right_arm_connected = True
            except Exception as e:
                logger.error(f"❌ Right arm connection failed: {e}")
                self.right_arm_connected = False

            # Mark as connected if at least one arm is connected
            self.is_connected = self.left_arm_connected and self.right_arm_connected

            if not self.is_connected:
                logger.error("❌ Failed to connect any robot arms")
            else:
                mode = "simulation" if use_sim else "hardware"
                logger.info(f"🎉 Robot interface connected in {mode} mode")

            return self.is_connected

        except Exception as e:
            logger.error(f"❌ Robot connection failed with exception: {e}")
            self.is_connected = False
            return False

    def setup_kinematics(self):
        """Setup kinematics solvers using PyBullet components for both arms."""
        # Setup solvers for both arms
        ground_height = self.config.ground_height
        # Only enable meshcat visualization if explicitly requested
        # Simulation mode uses its own visualizer (PyBullet GUI)
        enable_viz = self.config.enable_visualization and not self.config.run_in_newton and self.is_enabled
        self.ik_solver = Arm_IK(self.config.urdf_path, ground_height, enable_visualization=enable_viz)
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

    def solve_ik(self, target_pose_1: np.ndarray, target_pose_2: np.ndarray, visualize: bool) -> np.ndarray:
        """Solve inverse kinematics for both arms."""
        position_1, quaternion_1 = transform2pose(target_pose_1)
        position_2, quaternion_2 = transform2pose(target_pose_2)
        # transform2pose returns XYZW, but pin.Quaternion expects WXYZ
        target_1 = pin.SE3(
            pin.Quaternion(quaternion_1[3], quaternion_1[0], quaternion_1[1], quaternion_1[2]),
            position_1,
        )
        target_2 = pin.SE3(
            pin.Quaternion(quaternion_2[3], quaternion_2[0], quaternion_2[1], quaternion_2[2]),
            position_2,
        )
        sol_q, is_collision = self.ik_solver.ik_fun(target_1.homogeneous, target_2.homogeneous, visualize=visualize)
        return sol_q, is_collision

    def update_arm_angles(self, joint_angles: np.ndarray):
        """Update joint angles for specified arm."""
        self.arm_angles = joint_angles

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
                    action_dict = arm_angles_to_action_dict(self.arm_angles)
                    with suppress_stdout_stderr():
                        self.left_robot.send_action(action_dict["left"])
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
                    action_dict = arm_angles_to_action_dict(self.arm_angles)
                    with suppress_stdout_stderr():
                        self.right_robot.send_action(action_dict["right"])
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
            self.arm_angles[12] = angle
        elif arm == "right":
            self.arm_angles[13] = angle
        else:
            raise ValueError(f"Invalid arm: {arm}")

    def return_to_initial_position(self):
        """Return both arms to initial position."""
        logger.info("⏪ Returning robot to initial position...")

        arm_angles, collision = self.solve_ik(self.initial_left_arm, self.initial_right_arm, visualize=True)

        if collision:
            logger.error("❌ Cannot return to initial position due to collision in IK solution")
            self.arm_angles = np.zeros(NUM_JOINTS) * 2

        self.arm_angles = np.concatenate((arm_angles, [0.0, 0.0]))

        try:
            # Send commands for a few iterations to ensure movement
            for i in range(10):
                self.send_command()

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
        if self.left_robot is None or self.right_robot is None:
            action_dict = arm_angles_to_action_dict(self.arm_angles)
        return {
            "left": self.left_robot.get_observation() if self.left_robot else action_dict["left"],
            "right": self.right_robot.get_observation() if self.right_robot else action_dict["right"],
        }
