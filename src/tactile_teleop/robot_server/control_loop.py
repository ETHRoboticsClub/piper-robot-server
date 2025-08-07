import asyncio
import logging
import time
from dataclasses import dataclass
from typing import List

import numpy as np

from tactile_teleop.config import TelegripConfig

from .core.geometry import convert_to_robot_convention, xyzrpy2transform
from .core.robot_interface import RobotInterface
from .inputs.base import ControlGoal, EventType

logger = logging.getLogger(__name__)


@dataclass
class ArmState:
    arm_name: str
    initial_transform: np.ndarray = xyzrpy2transform(0.19, 0.0, 0.2, 0, 1.57, 0)
    origin_transform: np.ndarray | None = None
    target_transform: np.ndarray | None = None
    gripper_closed: bool = True


class ControlLoop:
    """Control loop for the teleoperation system."""

    def __init__(self, config: TelegripConfig, robot_enabled: bool = False, visualize: bool = False):
        self.config = config
        self.robot_interface = RobotInterface(config, robot_enabled)
        self.robot_enabled = robot_enabled
        self.visualize = visualize

    def update_arm_state(self, goals: List[ControlGoal], arm_state: ArmState) -> ArmState:
        """Process a list of control goals and update the arm states."""
        last_grip_active_goal = None
        for goal in goals:
            if goal.arm != arm_state.arm_name:
                continue

            if goal.event_type == EventType.GRIP_ACTIVE_INIT:
                if self.robot_enabled:
                    arm_state.origin_transform = self.robot_interface.get_end_effector_transform(arm_state.arm_name)
                else:
                    arm_state.origin_transform = arm_state.initial_transform
            elif goal.event_type == EventType.GRIP_ACTIVE:
                last_grip_active_goal = goal
            elif goal.event_type == EventType.GRIP_RELEASE:
                arm_state.target_transform = None
            elif goal.event_type == EventType.TRIGGER_ACTIVE:
                arm_state.gripper_closed = False
            elif goal.event_type == EventType.TRIGGER_RELEASE:
                arm_state.gripper_closed = True
            elif goal.event_type == EventType.RESET_BUTTON_RELEASE:
                # NOTE: When pressing grip right after reset, this may get overwritten and not actually reset
                arm_state.origin_transform = arm_state.initial_transform
                arm_state.target_transform = arm_state.initial_transform
            else:
                raise ValueError(f"Unknown event type: {goal.event_type}")

        if last_grip_active_goal is not None:
            logger.debug(f"{arm_state.arm_name} grip active")
            vr_reference_transform = convert_to_robot_convention(
                last_grip_active_goal.vr_reference_transform  # type: ignore
            )
            vr_target_transform = convert_to_robot_convention(last_grip_active_goal.vr_target_transform)  # type: ignore
            relative_transform = np.linalg.inv(vr_reference_transform) @ vr_target_transform
            arm_state.target_transform = arm_state.origin_transform @ relative_transform

        return arm_state

    def update_robot(self, left_arm: ArmState, right_arm: ArmState):
        """Update robot with current control goals."""
        start_time_total = time.perf_counter()
        # Measure all IK time together
        start_time_ik = time.perf_counter()

        # Left arm IK
        if left_arm.target_transform is not None:
            ik_solution = self.robot_interface.solve_ik("left", left_arm.target_transform, visualize=self.visualize)
            current_gripper = 0.0 if left_arm.gripper_closed else 0.07
            self.robot_interface.update_arm_angles("left", np.concatenate([ik_solution, [current_gripper]]))

        # Right arm IK
        if right_arm.target_transform is not None:
            ik_solution = self.robot_interface.solve_ik("right", right_arm.target_transform, visualize=self.visualize)
            current_gripper = 0.0 if right_arm.gripper_closed else 0.07
            self.robot_interface.update_arm_angles("right", np.concatenate([ik_solution, [current_gripper]]))

        ik_time = time.perf_counter() - start_time_ik

        # Send commands
        start_time_send = time.perf_counter()
        if self.robot_enabled:
            self.robot_interface.send_command()
        send_time = time.perf_counter() - start_time_send

        total_time = time.perf_counter() - start_time_total
        overhead_time = total_time - ik_time - send_time

        # # # Print all at once to minimize timing impact
        # logger.debug(
        #     f"IK: {ik_time*1000:.1f}ms, CAN: {send_time*1000:.1f}ms, "
        #     f"Overhead: {overhead_time*1000:.1f}ms, Total: {total_time*1000:.1f}ms"
        # )

    async def run(self, command_queue: asyncio.Queue):
        """Control loop for the teleoperation system."""
        left_arm = ArmState(arm_name="left")
        right_arm = ArmState(arm_name="right")
        self.robot_interface.setup_kinematics()
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
            try:
                goals = []
                while not command_queue.empty():
                    goals.append(command_queue.get_nowait())
                left_arm = self.update_arm_state(goals, left_arm)
                right_arm = self.update_arm_state(goals, right_arm)
            except Exception as e:
                logger.error(f"Error processing commands: {e}")
            commands_time = time.perf_counter() - commands_start

            # Simulates blocking robot communication
            robot_start = time.perf_counter()
            self.update_robot(left_arm, right_arm)
            robot_time = time.perf_counter() - robot_start

            sleep_start = time.perf_counter()
            await asyncio.sleep(0.001)
            sleep_time = time.perf_counter() - sleep_start

            total_time = time.perf_counter() - iteration_start
            overhead_time = total_time - commands_time - robot_time - sleep_time

            # # Single consolidated logging statement
            # logger.debug(
            #     f"Loop: {total_time*1000:.1f}ms ({1/total_time:.1f}Hz) | "
            #     f"Cmd: {commands_time*1000:.1f}ms | "
            #     f"Robot: {robot_time*1000:.1f}ms | "
            #     f"Sleep: {sleep_time*1000:.1f}ms | "
            #     f"Overhead: {overhead_time*1000:.1f}ms"
            #     "\n================================================================================="
            # )

    async def stop(self):
        """Stop the control loop."""
        if self.robot_enabled:
            self.robot_interface.disconnect()
