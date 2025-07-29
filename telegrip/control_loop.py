import asyncio
import logging
import time
from typing import List

import numpy as np

from .config import TelegripConfig
from .core.geometry import convert_to_robot_convention, xyzrpy2transform
from .core.robot_interface import RobotInterface
from .inputs.base import ControlGoal, EventType

logger = logging.getLogger(__name__)


class ArmState:
    """State tracking for a single robot arm."""

    def __init__(self, arm_name: str):
        self.arm_name = arm_name
        self.initial_transform = xyzrpy2transform(0.19, 0.0, 0.2, 0, 0, 0)
        self.origin_transform = None
        self.target_transform = None
        self.gripper_closed = True


def update_arm_state(
    goals: List[ControlGoal], arm_state: ArmState, robot_interface: RobotInterface, robot_enabled: bool = False
) -> ArmState:
    """Process a list of control goals and update the arm states."""
    last_grip_active_goal = None
    for goal in goals:
        if goal.arm != arm_state.arm_name:
            continue

        if goal.event_type == EventType.GRIP_ACTIVE_INIT:
            if robot_enabled:
                arm_state.origin_transform = robot_interface.get_end_effector_transform(arm_state.arm_name)
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
        print(f"{arm_state.arm_name} grip active")
        vr_reference_transform = convert_to_robot_convention(
            last_grip_active_goal.vr_reference_transform  # type: ignore
        )
        vr_target_transform = convert_to_robot_convention(last_grip_active_goal.vr_target_transform)  # type: ignore
        relative_transform = np.linalg.inv(vr_reference_transform) @ vr_target_transform
        arm_state.target_transform = arm_state.origin_transform @ relative_transform

    return arm_state


def update_robot(
    left_arm: ArmState,
    right_arm: ArmState,
    robot_interface: RobotInterface,
    robot_enabled: bool = False,
    visualize: bool = False,
):
    """Update robot with current control goals."""
    start_time_total = time.perf_counter()
    # Measure all IK time together
    start_time_ik = time.perf_counter()

    # Left arm IK
    if left_arm.target_transform is not None:
        ik_solution = robot_interface.solve_ik("left", left_arm.target_transform, visualize)
        current_gripper = 0.0 if left_arm.gripper_closed else 0.07
        robot_interface.update_arm_angles("left", np.concatenate([ik_solution, [current_gripper]]))

    # Right arm IK
    if right_arm.target_transform is not None:
        ik_solution = robot_interface.solve_ik("right", right_arm.target_transform, visualize=visualize)
        current_gripper = 0.0 if right_arm.gripper_closed else 0.07
        robot_interface.update_arm_angles("right", np.concatenate([ik_solution, [current_gripper]]))

    ik_time = time.perf_counter() - start_time_ik

    # Send commands
    start_time_send = time.perf_counter()
    if robot_enabled:
        robot_interface.send_command()
    send_time = time.perf_counter() - start_time_send

    total_time = time.perf_counter() - start_time_total
    overhead_time = total_time - ik_time - send_time

    # Print all at once to minimize timing impact
    print(
        f"IK: {ik_time*1000:.1f}ms, CAN: {send_time*1000:.1f}ms, "
        f"Overhead: {overhead_time*1000:.1f}ms, Total: {total_time*1000:.1f}ms"
    )


async def control_loop(
    command_queue: asyncio.Queue, config: TelegripConfig, robot_enabled: bool = True, visualize: bool = False
):
    """Control loop for the teleoperation system."""
    left_arm = ArmState("left")
    right_arm = ArmState("right")
    robot_interface = RobotInterface(config)
    robot_interface.setup_kinematics()
    if robot_enabled:
        robot_interface.connect()
        robot_interface.return_to_initial_position()

    while True:
        iteration_start = time.perf_counter()
        commands_start = time.perf_counter()
        try:
            command_count = 0
            goals = []
            while not command_queue.empty():
                goals.append(command_queue.get_nowait())
                # simulates goal processing time
                command_count += 1
            left_arm = update_arm_state(goals, left_arm, robot_interface, robot_enabled)
            right_arm = update_arm_state(goals, right_arm, robot_interface, robot_enabled)
        except Exception as e:
            logger.error(f"Error processing commands: {e}")
        commands_time = time.perf_counter() - commands_start

        # Simulates blocking robot communication
        robot_start = time.perf_counter()
        update_robot(left_arm, right_arm, robot_interface, robot_enabled, visualize)
        robot_time = time.perf_counter() - robot_start

        sleep_start = time.perf_counter()
        await asyncio.sleep(0.001)
        sleep_time = time.perf_counter() - sleep_start

        total_time = time.perf_counter() - iteration_start
        overhead_time = total_time - commands_time - robot_time - sleep_time

        # Single consolidated print statement
        print(
            f"Loop: {total_time*1000:.1f}ms ({1/total_time:.1f}Hz) | "
            f"Cmd: {commands_time*1000:.1f}ms | "
            f"Robot: {robot_time*1000:.1f}ms | "
            f"Sleep: {sleep_time*1000:.1f}ms | "
            f"Overhead: {overhead_time*1000:.1f}ms"
            "\n================================================================================="
        )
