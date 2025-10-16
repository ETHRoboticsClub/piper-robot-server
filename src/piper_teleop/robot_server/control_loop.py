import asyncio
import logging
import os
import time
from dataclasses import dataclass

import dotenv
import numpy as np
from tactile_teleop_sdk import TactileAPI

from piper_teleop.config import TelegripConfig

from .core.geometry import xyzrpy2transform
from .core.robot_interface import RobotInterface, arm_angles_to_action_dict
from .recorder import Recorder

dotenv.load_dotenv()

logger = logging.getLogger(__name__)


@dataclass
class ArmState:
    arm_name: str
    initial_transform: np.ndarray = xyzrpy2transform(0.19, 0.0, 0.2, 0, 1.57, 0)
    origin_transform: np.ndarray = xyzrpy2transform(0.19, 0.0, 0.2, 0, 1.57, 0)
    target_transform: np.ndarray | None = None
    gripper_closed: bool = True


class ControlLoop:
    """Control loop for the teleoperation system."""

    def __init__(self, config: TelegripConfig, robot_enabled: bool = False, visualize: bool = False, shared_img=None):
        self.config = config
        self.robot_interface = RobotInterface(config, robot_enabled)
        self.robot_enabled = robot_enabled
        self.visualize = visualize
        self.api = TactileAPI(api_key=os.getenv("TACTILE_API_KEY"))
        self.shared_img = shared_img
        if self.config.record:
            self.recorder = Recorder(repo_id=config.repo_id,
                                     resume=config.resume,
                                     task=config.task,
                                     root=config.root,
                                     single_arm=config.single_arm,
                                     cams = {'left' : (480, 640,3)},
                                     dof=config.dof,
                                     fps=config.fps,
                                     robot_type=config.robot_type,
                                     use_video=config.use_video,
                                     )
            self.recorder.start_recording()

    def update_arm_state(self, arm_goal, arm_state: ArmState) -> ArmState:
        if arm_goal.reset_to_init:
            arm_state.target_transform = arm_state.initial_transform
            arm_state.origin_transform = arm_state.initial_transform
        elif arm_goal.reset_reference:
            if self.robot_enabled:
                # NOTE: We use the last target transform as the origin transform since there is an offset between the target and the EEF transform
                arm_state.origin_transform = (
                    arm_state.target_transform
                    if arm_state.target_transform is not None
                    else arm_state.initial_transform
                )
        elif arm_goal.relative_transform is not None:
            relative_transform = arm_goal.relative_transform

            # Coordinate transform to local robot frame
            transformation_matrix = np.eye(4)
            transformation_matrix[:3, :3] = arm_state.origin_transform[:3, :3]

            relative_transform = np.linalg.inv(transformation_matrix) @ (relative_transform @ transformation_matrix)

            arm_state.target_transform = arm_state.origin_transform @ relative_transform

        if arm_goal.gripper_closed is False:
            arm_state.gripper_closed = False
        else:
            arm_state.gripper_closed = True

        return arm_state

    def update_robot(self, left_arm: ArmState, right_arm: ArmState):
        """Update robot with current control goals."""
        start_time_total = time.perf_counter()
        # Measure all IK time together
        start_time_ik = time.perf_counter()

        # Left arm IK
        if left_arm.target_transform is not None:
            ik_solution, is_collision = self.robot_interface.solve_ik(
                "left", left_arm.target_transform, visualize=self.visualize
            )
            current_gripper = 0.0 if left_arm.gripper_closed else 0.07
            if not is_collision:
                self.robot_interface.update_arm_angles("left", np.concatenate([ik_solution, [current_gripper]]))

        # Right arm IK
        if right_arm.target_transform is not None:
            ik_solution, is_collision = self.robot_interface.solve_ik(
                "right", right_arm.target_transform, visualize=self.visualize
            )
            current_gripper = 0.0 if right_arm.gripper_closed else 0.07
            if not is_collision:
                self.robot_interface.update_arm_angles("right", np.concatenate([ik_solution, [current_gripper]]))

        ik_time = time.perf_counter() - start_time_ik

        # Send commands
        start_time_send = time.perf_counter()
        if self.robot_enabled:
            self.robot_interface.send_command()
        send_time = time.perf_counter() - start_time_send

        total_time = time.perf_counter() - start_time_total
        overhead_time = total_time - ik_time - send_time

        # # Print all at once to minimize timing impact
        logger.debug(
            f"IK: {ik_time*1000:.1f}ms, CAN: {send_time*1000:.1f}ms, "
            f"Overhead: {overhead_time*1000:.1f}ms, Total: {total_time*1000:.1f}ms"
        )

    async def run(self):
        """Control loop for the teleoperation system."""
        left_arm = ArmState(arm_name="left")
        right_arm = ArmState(arm_name="right")
        self.robot_interface.setup_kinematics()
        await self.api.connect_vr_controller()
        if self.robot_enabled:
            try:
                self.robot_interface.connect()
            except Exception as e:
                logger.error(f"Error connecting to robot: {e}")
                return
            finally:
                self.robot_enabled = (
                    self.robot_interface.left_arm_connected and self.robot_interface.right_arm_connected
                )
        if self.robot_enabled:
            self.robot_interface.return_to_initial_position()

        while True:
            iteration_start = time.perf_counter()
            commands_start = time.perf_counter()

            commands_time = time.perf_counter() - commands_start

            left_arm_goal = await self.api.get_controller_goal("left")
            right_arm_goal = await self.api.get_controller_goal("right")
            left_arm = self.update_arm_state(left_arm_goal, left_arm)
            right_arm = self.update_arm_state(right_arm_goal, right_arm)

            if self.config.record:
                left_joints = arm_angles_to_action_dict(self.robot_interface.left_arm_angles)
                right_joints = arm_angles_to_action_dict(self.robot_interface.right_arm_angles)
                cams = {'observation.images.left': self.shared_img[0].numpy()}

            # Simulates blocking robot communication
            robot_start = time.perf_counter()
            self.update_robot(left_arm, right_arm)
            robot_time = time.perf_counter() - robot_start

            if self.config.record:
                left_joints_target = arm_angles_to_action_dict(self.robot_interface.left_arm_angles)
                right_joints_target = arm_angles_to_action_dict(self.robot_interface.right_arm_angles)
                self.recorder.add_observation( left_joints=left_joints,
                                               right_joints=right_joints,
                                               left_joints_target=left_joints_target,
                                               right_joints_target=right_joints_target,
                                               cams=cams)
                self.recorder.handle_keyboard_event()

            sleep_start = time.perf_counter()
            await asyncio.sleep(0.001)
            sleep_time = time.perf_counter() - sleep_start

            total_time = time.perf_counter() - iteration_start
            overhead_time = total_time - commands_time - robot_time - sleep_time

            # Single consolidated logging statement
            logger.debug(
                f"Loop: {total_time*1000:.1f}ms ({1/total_time:.1f}Hz) | "
                f"Cmd: {commands_time*1000:.1f}ms | "
                f"Robot: {robot_time*1000:.1f}ms | "
                f"Sleep: {sleep_time*1000:.1f}ms | "
                f"Overhead: {overhead_time*1000:.1f}ms"
                "\n================================================================================="
            )

    async def stop(self):
        """Stop the control loop."""
        await self.api.disconnect_vr_controller()
        if self.robot_enabled:
            self.robot_interface.disconnect()
