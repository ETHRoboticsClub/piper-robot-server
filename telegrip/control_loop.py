"""
Main control loop for the teleoperation system.
Consumes control goals from the command queue and executes them via the robot interface.
"""

import asyncio
import logging
import queue  # Add import for thread-safe queue
import time
from typing import Dict, Optional

import numpy as np

from .config import GRIPPER_INDEX, TelegripConfig
from .core.geometry import pose2transform
from .core.robot_interface import RobotInterface

# PyBulletVisualizer will be imported on demand
from .inputs.base import ControlGoal, ControlMode

logger = logging.getLogger(__name__)


class ArmState:
    """State tracking for a single robot arm."""

    def __init__(self, arm_name: str):
        self.arm_name = arm_name
        self.mode = ControlMode.IDLE
        self.origin_pose = pose2transform(np.array([0.19, 0.0, 0.2]), np.array([0.0, 0.0, 0.0, 1.0]))
        self.target_pose = None

    def reset(self):
        """Reset arm state to idle."""
        self.mode = ControlMode.IDLE
        self.target_pose = None


class ControlLoop:
    """Main control loop that processes command queue and controls robot."""

    def __init__(
        self, command_queue: asyncio.Queue, config: TelegripConfig, control_commands_queue: Optional[queue.Queue] = None
    ):
        self.command_queue = command_queue
        self.control_commands_queue = control_commands_queue
        self.config = config

        # Components
        self.robot_interface = None
        self.visualizer = None
        self.keyboard_listener = None  # Reference to keyboard listener for commands

        # Arm states
        self.left_arm = ArmState("left")
        self.right_arm = ArmState("right")

        # Control timing
        self.last_log_time = 0
        self.log_interval = 1.0  # Log status every second

        # Debug flags
        self._queue_debug_logged = False
        self._process_debug_logged = False

        self.is_running = False

    async def setup(self) -> bool:
        """Setup robot interface and visualizer."""
        success = True
        setup_errors = []

        # Setup robot interface
        try:
            self.robot_interface = RobotInterface(self.config)
            if not self.robot_interface.connect():
                error_msg = "Robot interface failed to connect"
                logger.error(error_msg)
                setup_errors.append(error_msg)
                if self.config.enable_robot:
                    success = False
            self.robot_interface.setup_kinematics()
        except Exception as e:
            error_msg = f"Robot interface setup failed with exception: {e}"
            logger.error(error_msg)
            setup_errors.append(error_msg)
            if self.config.enable_robot:
                success = False

        # Report all setup issues
        if setup_errors:
            logger.error("Setup failed with the following errors:")
            for i, error in enumerate(setup_errors, 1):
                logger.error(f"  {i}. {error}")

        # Set robot interface on keyboard listener so it can get current positions
        if self.keyboard_listener and self.robot_interface:
            self.keyboard_listener.set_robot_interface(self.robot_interface)
            logger.info("Set robot interface on keyboard listener")

        return success

    async def start(self):
        """Start the control loop."""
        if not await self.setup():
            logger.error("Control loop setup failed")
            return

        self.is_running = True
        logger.info("Control loop started")

        # Initialize arm states with current robot positions
        self._initialize_arm_states()

        # Main control loop
        while self.is_running:
            try:
                # Process command queue
                await self._process_commands()

                # Update robot (with error resilience)
                self._update_robot_safely()

                # Update visualization
                if self.visualizer:
                    self._update_visualization()

                # Periodic logging
                self._periodic_logging()

                # Control rate
                await asyncio.sleep(self.config.send_interval)

            except Exception as e:
                logger.error(f"Error in control loop: {e}")
                await asyncio.sleep(0.1)

        logger.info("Control loop stopped")

    async def stop(self):
        """Stop the control loop."""
        self.is_running = False

        # Cleanup
        if self.robot_interface:
            self.robot_interface.disconnect()

        if self.visualizer:
            self.visualizer.disconnect()

    def _initialize_arm_states(self):
        """Initialize arm states with current robot positions."""
        if self.robot_interface:
            self.robot_interface.return_to_initial_position()

    async def _process_commands(self):
        """Process commands from the command queue."""
        try:
            # Process regular control goals
            while not self.command_queue.empty():
                goal = self.command_queue.get_nowait()
                await self._process_goal(goal)
        except Exception as e:
            logger.error(f"Error processing commands: {e}")
            import traceback

            logger.error(f"Traceback: {traceback.format_exc()}")

    async def _handle_command(self, command):
        """Handle individual commands."""
        action = command.get("action", "")
        logger.info(f"🔌 Processing control command: {action}")

        if action == "enable_keyboard":
            if self.keyboard_listener:
                await self.keyboard_listener.start()
                logger.info("🎮 Keyboard control ENABLED via API")
        elif action == "disable_keyboard":
            if self.keyboard_listener:
                await self.keyboard_listener.stop()
                logger.info("🎮 Keyboard control DISABLED via API")
        elif action == "web_keypress":
            # Handle individual keypress events from web interface
            if self.keyboard_listener and self.keyboard_listener.is_enabled:
                key = command.get("key")
                event = command.get("event")  # 'press' or 'release'
                logger.info(f"🎮 Processing web keypress: {key}_{event}")
                await self._handle_web_keypress(key, event)
            else:
                logger.warning("🎮 Keyboard control not enabled for web keypress")
        elif action == "robot_connect":
            logger.info("🔌 Processing robot_connect command")
            if self.robot_interface and self.robot_interface.is_connected:
                logger.info(f"🔌 Robot interface available and connected: {self.robot_interface.is_connected}")
                success = self.robot_interface.engage()
                if success:
                    logger.info("🔌 Robot motors ENGAGED via API")
                    # No need to sync keyboard targets - unified system handles this automatically
                else:
                    logger.error("❌ Failed to engage robot motors")
            else:
                logger.warning(
                    f"Cannot engage robot: interface={self.robot_interface is not None}, connected={self.robot_interface.is_connected if self.robot_interface else False}"
                )
        elif action == "robot_disconnect":
            logger.info("🔌 Processing robot_disconnect command")
            if self.robot_interface:
                logger.info(f"🔌 Robot interface available")
                success = self.robot_interface.disengage()
                if success:
                    logger.info("🔌 Robot motors DISENGAGED via API")
                    # Reset arm states to IDLE when robot is disengaged
                    self.left_arm.reset()
                    self.right_arm.reset()
                    logger.info("🔓 Both arms: Position control DEACTIVATED after robot disconnect")

                    # Hide visualization markers
                    if self.visualizer:
                        for arm in ["left", "right"]:
                            self.visualizer.hide_marker(f"{arm}_goal")
                            self.visualizer.hide_frame(f"{arm}_goal_frame")
                            self.visualizer.hide_marker(f"{arm}_target")
                            self.visualizer.hide_frame(f"{arm}_target_frame")
                else:
                    logger.error("❌ Failed to disengage robot motors")
            else:
                logger.warning("Cannot disengage robot: no robot interface")
        else:
            logger.warning(f"Unknown command: {action}")

    async def _handle_web_keypress(self, key: str, event: str):
        """Handle keypress events from web interface by delegating to keyboard listener."""
        if not self.keyboard_listener:
            return

        # Create a mock key object for compatibility with keyboard listener
        class MockKey:
            def __init__(self, char=None, key_type=None):
                self.char = char
                self.key_type = key_type

        # Map special keys
        special_keys = {"tab": "Key.tab", "enter": "Key.enter", "esc": "Key.esc"}

        try:
            if event == "press":
                if key in special_keys:
                    # Handle special keys
                    if key == "tab":
                        from pynput.keyboard import Key

                        mock_key = Key.tab
                    elif key == "enter":
                        from pynput.keyboard import Key

                        mock_key = Key.enter
                    elif key == "esc":
                        from pynput.keyboard import Key

                        mock_key = Key.esc
                    else:
                        return
                else:
                    # Handle character keys
                    mock_key = MockKey(char=key)

                # Call keyboard listener's on_press method
                self.keyboard_listener.on_press(mock_key)

            elif event == "release":
                if key in special_keys:
                    # Special keys don't typically need release handling
                    return
                else:
                    # Handle character key release
                    mock_key = MockKey(char=key)

                # Call keyboard listener's on_release method
                self.keyboard_listener.on_release(mock_key)

        except Exception as e:
            logger.error(f"Error handling web keypress {key}_{event}: {e}")

    async def _process_goal(self, goal: ControlGoal):
        """Execute a control goal."""
        arm_state = self.left_arm if goal.arm == "left" else self.right_arm
        # Handle mode changes (only if mode is specified)
        if goal.mode is not None and goal.mode != arm_state.mode:
            if goal.mode == ControlMode.POSITION_CONTROL:
                arm_state.mode = ControlMode.POSITION_CONTROL
                arm_state.target_pose = arm_state.origin_pose.copy()
                # Activate position control - always reset target to initial pose
                logger.info(f"🔒 {goal.arm.upper()} arm: Position control ACTIVATED (target reset to initial pose)")

            elif goal.mode == ControlMode.IDLE:
                # Deactivate position control
                arm_state.reset()
                logger.info(f"🔓 {goal.arm.upper()} arm: Position control DEACTIVATED")
            else:
                raise ValueError("Unknown mode")

        # Handle position control - both VR and keyboard now work the same way (absolute offset from origin)
        if goal.target_transform is not None and arm_state.mode == ControlMode.POSITION_CONTROL:
            if goal.metadata and goal.metadata.get("relative_transform", False):
                # Both VR and keyboard send absolute offset from robot origin position
                arm_state.target_pose = np.dot(arm_state.origin_pose, goal.target_transform)
            else:
                raise ValueError("Absolute position control is not supported anymore")

        # Handle gripper control (independent of mode)
        if goal.gripper_closed is not None and self.robot_interface:
            self.robot_interface.set_gripper(goal.arm, goal.gripper_closed)

    def _update_robot_safely(self):
        """Update robot with current control goals (with error handling)."""
        if not self.robot_interface:
            return

        try:
            self._update_robot()
        except Exception as e:
            logger.error(f"Error updating robot: {e}")
            # Don't shutdown, just continue - robot interface will handle connection issues

    def _update_robot(self):
        """Update robot with current control goals."""
        if not self.robot_interface:
            return

        # Update left arm (only if connected)
        if self.left_arm.mode == ControlMode.POSITION_CONTROL and self.left_arm.target_pose is not None:

            # Solve IK
            ik_solution = self.robot_interface.solve_ik("left", self.left_arm.target_pose)

            # Update robot angles
            current_gripper = self.robot_interface.get_arm_angles("left")[GRIPPER_INDEX]
            self.robot_interface.update_arm_angles("left", np.concatenate([ik_solution, [current_gripper]]))

        # Update right arm (only if connected)
        if self.right_arm.mode == ControlMode.POSITION_CONTROL and self.right_arm.target_pose is not None:

            # Solve IK
            ik_solution = self.robot_interface.solve_ik("right", self.right_arm.target_pose)

            # Update robot angles
            current_gripper = self.robot_interface.get_arm_angles("right")[GRIPPER_INDEX]
            self.robot_interface.update_arm_angles("right", np.concatenate([ik_solution, [current_gripper]]))

        # Send commands to robot
        if self.robot_interface.is_connected and self.robot_interface.is_engaged:
            self.robot_interface.send_command()

    def _update_visualization(self):
        """Update PyBullet visualization."""
        if not self.visualizer:
            return

        # Update robot poses for both arms using ACTUAL angles from robot hardware
        left_angles = self.robot_interface.get_actual_arm_angles("left")
        right_angles = self.robot_interface.get_actual_arm_angles("right")

        self.visualizer.update_robot_pose(left_angles, "left")
        self.visualizer.update_robot_pose(right_angles, "right")

        # Update visualization markers
        if self.left_arm.mode == ControlMode.POSITION_CONTROL:
            if self.left_arm.target_position is not None:
                # Show current end effector position
                current_pos = self.robot_interface.get_current_end_effector_position("left")
                self.visualizer.update_marker_position("left_target", current_pos)
                self.visualizer.update_coordinate_frame("left_target_frame", current_pos)

            if self.left_arm.goal_position is not None:
                # Show goal position
                self.visualizer.update_marker_position("left_goal", self.left_arm.goal_position)
                self.visualizer.update_coordinate_frame("left_goal_frame", self.left_arm.goal_position)
        else:
            # Hide markers when not in position control
            self.visualizer.hide_marker("left_target")
            self.visualizer.hide_marker("left_goal")
            self.visualizer.hide_frame("left_target_frame")
            self.visualizer.hide_frame("left_goal_frame")

        if self.right_arm.mode == ControlMode.POSITION_CONTROL:
            if self.right_arm.target_position is not None:
                # Show current end effector position
                current_pos = self.robot_interface.get_current_end_effector_position("right")
                self.visualizer.update_marker_position("right_target", current_pos)
                self.visualizer.update_coordinate_frame("right_target_frame", current_pos)

            if self.right_arm.goal_position is not None:
                # Show goal position
                self.visualizer.update_marker_position("right_goal", self.right_arm.goal_position)
                self.visualizer.update_coordinate_frame("right_goal_frame", self.right_arm.goal_position)
        else:
            # Hide markers when not in position control
            self.visualizer.hide_marker("right_target")
            self.visualizer.hide_marker("right_goal")
            self.visualizer.hide_frame("right_target_frame")
            self.visualizer.hide_frame("right_goal_frame")

        # Step simulation
        self.visualizer.step_simulation()

    def _periodic_logging(self):
        """Log status information periodically."""
        current_time = time.time()
        if current_time - self.last_log_time >= self.log_interval:
            self.last_log_time = current_time

            active_arms = []
            if self.left_arm.mode == ControlMode.POSITION_CONTROL:
                active_arms.append("LEFT")
            if self.right_arm.mode == ControlMode.POSITION_CONTROL:
                active_arms.append("RIGHT")

            if active_arms and self.robot_interface:
                left_angles = self.robot_interface.get_arm_angles("left")
                right_angles = self.robot_interface.get_arm_angles("right")
                logger.info(
                    f"🤖 Active control: {', '.join(active_arms)} | Left: {left_angles.round(1)} | Right: {right_angles.round(1)}"
                )

    @property
    def status(self) -> Dict:
        """Get current control loop status."""
        return {
            "running": self.is_running,
            "left_arm_mode": self.left_arm.mode.value,
            "right_arm_mode": self.right_arm.mode.value,
            "robot_connected": self.robot_interface.is_connected if self.robot_interface else False,
            "left_arm_connected": (
                self.robot_interface.get_arm_connection_status("left") if self.robot_interface else False
            ),
            "right_arm_connected": (
                self.robot_interface.get_arm_connection_status("right") if self.robot_interface else False
            ),
            "visualizer_connected": self.visualizer.is_connected if self.visualizer else False,
        }
