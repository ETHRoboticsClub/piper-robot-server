"""
Keyboard input listener for teleoperation control.
Adapted from the original lerobot_keyboard_ik.py script.
"""

import asyncio
import numpy as np
import logging
from typing import Dict
from pynput import keyboard
import threading

from .base import BaseInputProvider, ControlGoal, ControlMode
from ..config import TeleopConfig, POS_STEP, ANGLE_STEP, GRIPPER_STEP

logger = logging.getLogger(__name__)


class KeyboardListener(BaseInputProvider):
    """Keyboard input provider for teleoperation."""
    
    def __init__(self, command_queue: asyncio.Queue, config: TeleopConfig):
        super().__init__(command_queue)
        self.config = config
        
        # Keyboard listener
        self.listener = None
        self.listener_thread = None
        
        # Current control state
        self.active_arm = "left"  # Currently controlled arm
        self.target_position = np.array([0.2, 0.0, 0.15])  # Current target position
        self.target_wrist_roll = 0.0
        
        # Delta tracking for continuous movement
        self.delta_pos = np.zeros(3)
        self.delta_wrist_roll = 0.0
        self.delta_gripper = 0.0
        
        # Control flags
        self.position_control_active = False
        self.gripper_closed = False
    
    async def start(self):
        """Start the keyboard listener."""
        if not self.config.enable_keyboard:
            logger.info("Keyboard listener disabled in configuration")
            return
        
        self.is_running = True
        
        # Start keyboard listener in a separate thread
        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )
        
        self.listener_thread = threading.Thread(target=self.listener.start)
        self.listener_thread.daemon = True
        self.listener_thread.start()
        
        # Start control loop
        asyncio.create_task(self.control_loop())
        
        logger.info("Keyboard listener started")
        self.print_controls()
    
    async def stop(self):
        """Stop the keyboard listener."""
        self.is_running = False
        
        if self.listener:
            self.listener.stop()
        
        logger.info("Keyboard listener stopped")
    
    def print_controls(self):
        """Print keyboard control instructions."""
        logger.info("\n" + "="*50)
        logger.info("KEYBOARD TELEOPERATION CONTROLS")
        logger.info("="*50)
        logger.info("Position Control (World Frame):")
        logger.info("  A/D: Move Left/Right (+/- Y)")
        logger.info("  W/S: Move Forward/Backward (+/- X)")
        logger.info("  Q/E: Move Down/Up (+/- Z)")
        logger.info("")
        logger.info("Wrist Control:")
        logger.info("  Left/Right Arrow: Wrist Roll")
        logger.info("")
        logger.info("Gripper Control:")
        logger.info("  Space: Toggle Gripper Open/Closed")
        logger.info("")
        logger.info("Arm Selection:")
        logger.info("  1: Control Left Arm")
        logger.info("  2: Control Right Arm")
        logger.info("")
        logger.info("Control Mode:")
        logger.info("  Enter: Toggle Position Control On/Off")
        logger.info("  ESC: Exit")
        logger.info("="*50)
        logger.info(f"Currently controlling: {self.active_arm.upper()} arm")
        logger.info(f"Position control: {'ACTIVE' if self.position_control_active else 'INACTIVE'}")
    
    def on_press(self, key):
        """Handle key press events."""
        try:
            # Position control (WASD + QE)
            if key.char == 'a':
                self.delta_pos[1] = POS_STEP  # Left (+Y)
            elif key.char == 'd':
                self.delta_pos[1] = -POS_STEP  # Right (-Y)
            elif key.char == 'w':
                self.delta_pos[0] = POS_STEP   # Forward (+X)
            elif key.char == 's':
                self.delta_pos[0] = -POS_STEP  # Backward (-X)
            elif key.char == 'q':
                self.delta_pos[2] = -POS_STEP  # Down (-Z)
            elif key.char == 'e':
                self.delta_pos[2] = POS_STEP   # Up (+Z)
            
            # Arm selection
            elif key.char == '1':
                self.active_arm = "left"
                logger.info(f"Switched to controlling LEFT arm")
            elif key.char == '2':
                self.active_arm = "right"
                logger.info(f"Switched to controlling RIGHT arm")
            
            # Gripper control
            elif key.char == ' ':
                self.gripper_closed = not self.gripper_closed
                logger.info(f"{self.active_arm.upper()} gripper: {'CLOSED' if self.gripper_closed else 'OPENED'}")
                # Send gripper goal immediately
                self._send_gripper_goal()
        
        except AttributeError:
            # Special keys
            if key == keyboard.Key.left:
                self.delta_wrist_roll = -ANGLE_STEP  # CCW
            elif key == keyboard.Key.right:
                self.delta_wrist_roll = ANGLE_STEP   # CW
            elif key == keyboard.Key.enter:
                self.position_control_active = not self.position_control_active
                logger.info(f"Position control: {'ACTIVATED' if self.position_control_active else 'DEACTIVATED'}")
                # Send mode change goal
                self._send_mode_change_goal()
            elif key == keyboard.Key.esc:
                logger.info("ESC pressed. Stopping keyboard control.")
                self.is_running = False
                return False  # Stop the listener
    
    def on_release(self, key):
        """Handle key release events."""
        try:
            # Reset deltas on key release
            if key.char in ('a', 'd'):
                self.delta_pos[1] = 0
            elif key.char in ('w', 's'):
                self.delta_pos[0] = 0
            elif key.char in ('q', 'e'):
                self.delta_pos[2] = 0
        except AttributeError:
            if key in (keyboard.Key.left, keyboard.Key.right):
                self.delta_wrist_roll = 0
    
    def _send_gripper_goal(self):
        """Send gripper control goal to queue."""
        goal = ControlGoal(
            arm=self.active_arm,
            mode=ControlMode.IDLE,
            gripper_closed=self.gripper_closed,
            metadata={"source": "keyboard_gripper"}
        )
        # Put goal in queue (non-blocking)
        try:
            self.command_queue.put_nowait(goal)
        except:
            pass  # Queue might be full, ignore
    
    def _send_mode_change_goal(self):
        """Send mode change goal to queue."""
        mode = ControlMode.POSITION_CONTROL if self.position_control_active else ControlMode.IDLE
        goal = ControlGoal(
            arm=self.active_arm,
            mode=mode,
            metadata={"source": "keyboard_mode"}
        )
        # Put goal in queue (non-blocking)
        try:
            self.command_queue.put_nowait(goal)
        except:
            pass  # Queue might be full, ignore
    
    async def control_loop(self):
        """Main control loop that processes keyboard input and sends commands."""
        logger.info("Keyboard control loop started")
        
        while self.is_running:
            try:
                # Update target state based on deltas
                if self.position_control_active:
                    self.target_position += self.delta_pos
                    self.target_wrist_roll += self.delta_wrist_roll
                    
                    # Send position control goal if there's movement or wrist rotation
                    if np.any(self.delta_pos != 0) or self.delta_wrist_roll != 0:
                        goal = ControlGoal(
                            arm=self.active_arm,
                            mode=ControlMode.POSITION_CONTROL,
                            target_position=self.target_position.copy(),
                            wrist_roll_deg=self.target_wrist_roll,
                            metadata={
                                "source": "keyboard",
                                "relative_position": False
                            }
                        )
                        await self.send_goal(goal)
                
                # Sleep to control update rate
                await asyncio.sleep(0.05)  # 20Hz update rate
                
            except Exception as e:
                logger.error(f"Error in keyboard control loop: {e}")
                await asyncio.sleep(0.1)
        
        logger.info("Keyboard control loop stopped") 