"""
Main control loop for the teleoperation system.
Consumes control goals from the command queue and executes them via the robot interface.
"""

import asyncio
import numpy as np
import logging
import time
from typing import Dict, Optional

from .config import TeleopConfig, NUM_JOINTS, WRIST_ROLL_INDEX, GRIPPER_INDEX
from .core.robot_interface import RobotInterface
from .core.visualizer import PyBulletVisualizer
from .inputs.base import ControlGoal, ControlMode

logger = logging.getLogger(__name__)


class ArmState:
    """State tracking for a single robot arm."""
    
    def __init__(self, arm_name: str):
        self.arm_name = arm_name
        self.mode = ControlMode.IDLE
        self.target_position = None
        self.goal_position = None  # For visualization
        self.origin_position = None  # Robot position when grip was activated
        self.origin_wrist_angle = 0.0
        self.current_wrist_roll = 0.0
        
    def reset(self):
        """Reset arm state to idle."""
        self.mode = ControlMode.IDLE
        self.target_position = None
        self.goal_position = None
        self.origin_position = None
        self.origin_wrist_angle = 0.0


class ControlLoop:
    """Main control loop that processes command queue and controls robot."""
    
    def __init__(self, command_queue: asyncio.Queue, config: TeleopConfig):
        self.command_queue = command_queue
        self.config = config
        
        # Components
        self.robot_interface = None
        self.visualizer = None
        
        # Arm states
        self.left_arm = ArmState("left")
        self.right_arm = ArmState("right")
        
        # Control timing
        self.last_log_time = 0
        self.log_interval = 1.0  # Log status every second
        
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
        except Exception as e:
            error_msg = f"Robot interface setup failed with exception: {e}"
            logger.error(error_msg)
            setup_errors.append(error_msg)
            if self.config.enable_robot:
                success = False
        
        # Setup PyBullet visualizer
        if self.config.enable_pybullet:
            try:
                self.visualizer = PyBulletVisualizer(
                    self.config.urdf_path, 
                    use_gui=True
                )
                if not self.visualizer.setup():
                    error_msg = "PyBullet visualizer setup failed"
                    logger.error(error_msg)
                    setup_errors.append(error_msg)
                    self.visualizer = None
                else:
                    # Connect kinematics to robot interface
                    joint_limits_min, joint_limits_max = self.visualizer.get_joint_limits
                    self.robot_interface.setup_kinematics(
                        self.visualizer.physics_client,
                        self.visualizer.robot_ids,  # Pass both robot instances
                        self.visualizer.joint_indices,  # Pass both joint index mappings
                        self.visualizer.end_effector_link_indices,  # Pass both end effector indices
                        joint_limits_min,
                        joint_limits_max
                    )
            except Exception as e:
                error_msg = f"PyBullet visualizer setup failed with exception: {e}"
                logger.error(error_msg)
                setup_errors.append(error_msg)
                self.visualizer = None
        
        # Report all setup issues
        if setup_errors:
            logger.error("Setup failed with the following errors:")
            for i, error in enumerate(setup_errors, 1):
                logger.error(f"  {i}. {error}")
        
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
                
                # Update robot
                self._update_robot()
                
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
            # Get current end effector positions
            left_pos = self.robot_interface.get_current_end_effector_position("left")
            right_pos = self.robot_interface.get_current_end_effector_position("right")
            
            # Get current wrist roll angles
            left_angles = self.robot_interface.get_arm_angles("left")
            right_angles = self.robot_interface.get_arm_angles("right")
            
            self.left_arm.current_wrist_roll = left_angles[WRIST_ROLL_INDEX]
            self.right_arm.current_wrist_roll = right_angles[WRIST_ROLL_INDEX]
            
            logger.info(f"Initialized left arm at position: {left_pos.round(3)}")
            logger.info(f"Initialized right arm at position: {right_pos.round(3)}")
    
    async def _process_commands(self):
        """Process commands from the command queue."""
        try:
            # Process all available commands
            while True:
                try:
                    goal = self.command_queue.get_nowait()
                    await self._execute_goal(goal)
                except asyncio.QueueEmpty:
                    break
        except Exception as e:
            logger.error(f"Error processing commands: {e}")
    
    async def _execute_goal(self, goal: ControlGoal):
        """Execute a control goal."""
        arm_state = self.left_arm if goal.arm == "left" else self.right_arm
        
        # Handle mode changes (only if mode is specified)
        if goal.mode is not None and goal.mode != arm_state.mode:
            if goal.mode == ControlMode.POSITION_CONTROL:
                # Activate position control
                arm_state.mode = ControlMode.POSITION_CONTROL
                
                # Set origin position for relative movement
                if self.robot_interface:
                    arm_state.origin_position = self.robot_interface.get_current_end_effector_position(goal.arm)
                    current_angles = self.robot_interface.get_arm_angles(goal.arm)
                    arm_state.origin_wrist_angle = current_angles[WRIST_ROLL_INDEX]
                
                logger.info(f"🔒 {goal.arm.upper()} arm: Position control ACTIVATED")
                
            elif goal.mode == ControlMode.IDLE:
                # Deactivate position control
                arm_state.reset()
                
                # Hide visualization markers
                if self.visualizer:
                    self.visualizer.hide_marker(f"{goal.arm}_goal")
                    self.visualizer.hide_frame(f"{goal.arm}_goal_frame")
                
                logger.info(f"🔓 {goal.arm.upper()} arm: Position control DEACTIVATED")
        
        # Handle position control
        if goal.target_position is not None and arm_state.mode == ControlMode.POSITION_CONTROL:
            if goal.metadata and goal.metadata.get("relative_position", False):
                # Relative position from VR controller
                if arm_state.origin_position is not None:
                    arm_state.target_position = arm_state.origin_position + goal.target_position
                    arm_state.goal_position = arm_state.target_position.copy()
            else:
                # Absolute position from keyboard
                arm_state.target_position = goal.target_position.copy()
                arm_state.goal_position = goal.target_position.copy()
            
            # Handle wrist roll
            if goal.wrist_roll_deg is not None:
                if goal.metadata and goal.metadata.get("relative_position", False):
                    # Relative wrist roll from VR
                    arm_state.current_wrist_roll = arm_state.origin_wrist_angle + goal.wrist_roll_deg
                else:
                    # Absolute wrist roll from keyboard
                    arm_state.current_wrist_roll = goal.wrist_roll_deg
        
        # Handle gripper control (independent of mode)
        if goal.gripper_closed is not None and self.robot_interface:
            self.robot_interface.set_gripper(goal.arm, goal.gripper_closed)
    
    def _update_robot(self):
        """Update robot with current control goals."""
        if not self.robot_interface:
            return
        
        # Update left arm
        if self.left_arm.mode == ControlMode.POSITION_CONTROL and self.left_arm.target_position is not None:
            # Solve IK
            ik_solution = self.robot_interface.solve_ik("left", self.left_arm.target_position)
            
            # Update robot angles
            current_gripper = self.robot_interface.get_arm_angles("left")[GRIPPER_INDEX]
            self.robot_interface.update_arm_angles(
                "left", 
                ik_solution, 
                self.left_arm.current_wrist_roll,
                current_gripper
            )
        
        # Update right arm
        if self.right_arm.mode == ControlMode.POSITION_CONTROL and self.right_arm.target_position is not None:
            # Solve IK
            ik_solution = self.robot_interface.solve_ik("right", self.right_arm.target_position)
            
            # Update robot angles
            current_gripper = self.robot_interface.get_arm_angles("right")[GRIPPER_INDEX]
            self.robot_interface.update_arm_angles(
                "right", 
                ik_solution, 
                self.right_arm.current_wrist_roll,
                current_gripper
            )
        
        # Send commands to robot
        self.robot_interface.send_command()
    
    def _update_visualization(self):
        """Update PyBullet visualization."""
        if not self.visualizer:
            return
        
        # Update robot poses for both arms
        left_angles = self.robot_interface.get_arm_angles("left")
        right_angles = self.robot_interface.get_arm_angles("right")
        
        self.visualizer.update_robot_pose(left_angles, 'left')
        self.visualizer.update_robot_pose(right_angles, 'right')
        
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
                logger.info(f"🤖 Active control: {', '.join(active_arms)} | Left: {left_angles.round(1)} | Right: {right_angles.round(1)}")
    
    @property
    def status(self) -> Dict:
        """Get current control loop status."""
        return {
            "running": self.is_running,
            "left_arm_mode": self.left_arm.mode.value,
            "right_arm_mode": self.right_arm.mode.value,
            "robot_connected": self.robot_interface.is_connected if self.robot_interface else False,
            "visualizer_connected": self.visualizer.is_connected if self.visualizer else False,
        } 