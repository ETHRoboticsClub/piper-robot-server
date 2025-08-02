"""
VR WebSocket server for receiving controller data from web browsers.
Adapted from the original vr_robot_teleop.py script.
"""

import asyncio
import json
import logging
import os
import time
from typing import Dict, Optional

import numpy as np

from livekit import rtc

from camera_streaming.auth import generate_token

from ..config import TelegripConfig
from ..core.geometry import pose2transform
from .base import BaseInputProvider, ControlGoal, EventType


logger = logging.getLogger(__name__)

LIVEKIT_URL = os.getenv("LIVEKIT_URL")

class TextDecoder():
    def __init__(self):
        pass
    
    def decode(self, array):
        """
        exp:
            >>> textdecoder = TextDecoder()
            >>> textdecoder.decode([36])
            >>> $
        """
        if isinstance(array, list):
            return bytearray(array).decode('utf-8')
        elif isinstance(array, bytearray):
            return array.decode('utf-8')
        else:
            raise TypeError(f'expecting a list or bytearray got: {type(array)}')

class VRControllerState:
    """State tracking for a VR controller."""

    def __init__(self, hand: str):
        self.hand = hand
        self.grip_active = False
        self.trigger_active = False

        # Origin pose when grip was activated
        self.origin_transform: Optional[np.ndarray] = None  # 4x4 transform matrix

        # Current pose
        self.current_transform: Optional[np.ndarray] = None  # 4x4 transform matrix

    def reset_grip(self):
        """Reset grip state but preserve trigger state."""
        self.grip_active = False
        self.origin_transform = None
        self.current_transform = None
        

class VRControllerInputProvider(BaseInputProvider):
    """Input provider for VR controllers."""

    def __init__(self, command_queue: asyncio.Queue, config: TelegripConfig):
        self.logger = logging.getLogger(__name__)   
        super().__init__(command_queue)
        self.config = config
        
        # Controller states
        self.left_controller = VRControllerState("left")
        self.right_controller = VRControllerState("right")
        
        # Simple frequency tracking
        self.msg_count = 0
        self.start_time = None
        self._stats_lock = asyncio.Lock()
        
    async def start(self, room_name: str, participant_name: str):
        self.logger.info("Connecting to LiveKit room")
        
        # Check environment variables
        if not LIVEKIT_URL:
            self.logger.error("LIVEKIT_URL environment variables must be set")
            return

        self.room = rtc.Room()
        lk_token = generate_token(room_name, participant_identity=participant_name)
        
        # event handlers
        @self.room.on("participant_connected")
        def on_participant_connected(participant: rtc.RemoteParticipant):
            self.logger.info(f"Participant connected {participant.sid}, {participant.identity}")

        @self.room.on("data_received")
        def on_data_received(data: rtc.DataPacket):
            asyncio.create_task(self._handle_data_packet(data))

        # connect to livekit room
        try:
            await self.room.connect(LIVEKIT_URL, lk_token)
            self.logger.info("Connected to LiveKit room")
            while True:
                await asyncio.sleep(1) # livekit room stay alive signal
                self.logger.info("Livekit room stay alive signal")
        except Exception as e:
            self.logger.error(f"Failed to connect to LiveKit: {e}")
            return
        finally:
            await self.room.disconnect()
        
    async def stop(self):
        self.logger.info("Disconnecting from LiveKit room")
        if self.room:
            await self.room.disconnect()
         
    async def _handle_data_packet(self, packet: rtc.DataPacket) -> None:
        """Handle data packet coming from LiveKit."""
        processing_start = time.perf_counter()
        try:
            payload = json.loads(TextDecoder().decode(packet.data))
            await self._process_controller_data(payload)
        except json.JSONDecodeError:
            self.logger.warning(f"Received non-JSON message: {packet.data}")
            return
        except Exception as e:
            self.logger.error(f"Error processing VR data: {e}")
            return
        finally:
            # update stats under lock to avoid races across parallel tasks
            async with self._stats_lock:
                self.msg_count += 1
                if self.start_time is None:
                    self.start_time = processing_start
                elif self.msg_count % 100 == 0:
                    elapsed = time.perf_counter() - self.start_time
                    freq = self.msg_count / elapsed if elapsed else 0.0
                    self.logger.debug(f"📊 Message frequency: {freq:.1f} Hz ({self.msg_count} msgs)")
                    self.msg_count = 0
                    self.start_time = time.perf_counter()
            # log processing latency outside the lock – purely diagnostic
            self.logger.debug(
                f"🕒 VR message processing time: {(time.perf_counter() - processing_start)*1000:.1f}ms"
            )
            
    async def _process_controller_data(self, data: Dict):
        """Process incoming VR controller data."""

        # Handle new dual controller format
        if "leftController" in data and "rightController" in data:
            left_data = data["leftController"]
            right_data = data["rightController"]

            # Process left controller
            if left_data.get("position") and (left_data.get("gripActive", False) or left_data.get("trigger", 0) > 0.5):
                await self._process_single_controller("left", left_data)
            elif not left_data.get("gripActive", False) and self.left_controller.grip_active:
                await self._handle_grip_release("left")

            # Process right controller
            if right_data.get("position") and (
                right_data.get("gripActive", False) or right_data.get("trigger", 0) > 0.5
            ):
                await self._process_single_controller("right", right_data)
            elif not right_data.get("gripActive", False) and self.right_controller.grip_active:
                await self._handle_grip_release("right")

            return

        # Handle legacy single controller format
        hand = data.get("hand")

        # Handle explicit release messages
        if data.get("gripReleased"):
            await self._handle_grip_release(hand)
            return

        if data.get("triggerReleased"):
            await self._handle_trigger_release(hand)
            return

        if data.get("resetEvent"):
            await self._handle_reset_button_release(hand)
            return

    async def _process_single_controller(self, hand: str, data: Dict):
        """Process data for a single controller."""
        position = data.get("position", {})
        quaternion = data.get("quaternion", {})  # Get quaternion data directly
        grip_active = data.get("gripActive", False)
        trigger = data.get("trigger", 0)

        assert quaternion is not None and all(k in quaternion for k in ["x", "y", "z", "w"]), "Quaternion data missing"
        quaternion = np.array([quaternion["x"], quaternion["y"], quaternion["z"], quaternion["w"]])
        position = np.array([position["x"], position["y"], position["z"]])
        transform = pose2transform(position, quaternion)

        controller = self.left_controller if hand == "left" else self.right_controller

        # Handle trigger for gripper control
        trigger_active = trigger > 0.5
        if trigger_active != controller.trigger_active:
            controller.trigger_active = trigger_active

            # Send gripper control goal - do not specify mode to avoid interfering with position control
            # Reverse behavior: gripper open by default, closes when trigger pressed
            gripper_goal = ControlGoal(
                event_type=(EventType.TRIGGER_ACTIVE if trigger_active else EventType.TRIGGER_RELEASE),
                arm=hand,
                gripper_closed=not trigger_active,  # Inverted: closed when trigger NOT active
            )
            await self.send_goal(gripper_goal)

            logger.info(
                f"🤏 {hand.upper()} trigger {'ACTIVE' if trigger_active else 'RELEASED'} - gripper {'OPENED' if trigger_active else 'CLOSED'}"
            )

        # Handle grip button for arm movement control
        if grip_active:
            if not controller.grip_active:
                # Grip just activated - set origin and reset target position
                controller.grip_active = True
                controller.origin_transform = transform.copy()

                # Send reset signal to control loop to reset target position to current robot position
                reset_goal = ControlGoal(
                    event_type=EventType.GRIP_ACTIVE_INIT,
                    arm=hand,
                )
                await self.send_goal(reset_goal)

                logger.info(f"🔒 {hand.upper()} grip activated - arm control enabled")

            # Compute target position
            if controller.origin_transform is not None:

                # Create control goal with relative transform
                goal = ControlGoal(
                    event_type=EventType.GRIP_ACTIVE,
                    arm=hand,
                    vr_reference_transform=controller.origin_transform,
                    vr_target_transform=transform,
                )
                await self.send_goal(goal)

    async def _handle_grip_release(self, hand: str):
        """Handle grip release for a controller."""
        if hand == "left":
            controller = self.left_controller
        elif hand == "right":
            controller = self.right_controller
        else:
            return

        if controller.grip_active:
            controller.reset_grip()

            # Send idle goal to stop arm control
            goal = ControlGoal(
                event_type=EventType.GRIP_RELEASE,
                arm=hand,
            )
            await self.send_goal(goal)

            logger.info(f"🔓 {hand.upper()} grip released - arm control stopped")

    async def _handle_trigger_release(self, hand: str):
        """Handle trigger release for a controller."""
        controller = self.left_controller if hand == "left" else self.right_controller

        if controller.trigger_active:
            controller.trigger_active = False

            # Send gripper closed goal - reversed behavior: gripper closes when trigger released
            goal = ControlGoal(
                event_type=EventType.TRIGGER_RELEASE,
                arm=hand,
                gripper_closed=True,  # Close gripper when trigger released
            )
            await self.send_goal(goal)

            logger.info(f"🤏 {hand.upper()} trigger released - gripper CLOSED")

    async def _handle_reset_button_release(self, hand: str):
        """Handle X button release for a controller."""
        goal = ControlGoal(
            event_type=EventType.RESET_BUTTON_RELEASE,
            arm=hand,
        )
        await self.send_goal(goal)

        logger.info(f"🔓 {hand.upper()} reset button released - going to initial position")
        
    