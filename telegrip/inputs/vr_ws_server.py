"""
VR WebSocket server for receiving controller data from web browsers.
Adapted from the original vr_robot_teleop.py script.
"""

import asyncio
import json
import logging
import math
import ssl
from typing import Dict, Optional, Set, Tuple

import numpy as np
import websockets
from scipy.spatial.transform import Rotation as R

from ..config import TelegripConfig
from ..core.geometry import pose2transform, transform2pose, xyzrpy2transform
from .base import BaseInputProvider, ControlGoal, ControlMode

logger = logging.getLogger(__name__)


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


class VRWebSocketServer(BaseInputProvider):
    """WebSocket server for VR controller input."""

    def __init__(self, command_queue: asyncio.Queue, config: TelegripConfig):
        super().__init__(command_queue)
        self.config = config
        self.clients: Set = set()
        self.server = None

        # Controller states
        self.left_controller = VRControllerState("left")
        self.right_controller = VRControllerState("right")

    def setup_ssl(self) -> Optional[ssl.SSLContext]:
        """Setup SSL context for WebSocket server."""
        # Automatically generate SSL certificates if they don't exist
        if not self.config.ssl_files_exist:
            logger.info("SSL certificates not found for WebSocket server, attempting to generate them...")
            if not self.config.ensure_ssl_certificates():
                logger.error("Failed to generate SSL certificates for WebSocket server")
                return None

        ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        try:
            # Get absolute paths for SSL certificates
            cert_path, key_path = self.config.get_absolute_ssl_paths()
            ssl_context.load_cert_chain(certfile=cert_path, keyfile=key_path)
            logger.info("SSL certificate and key loaded successfully for WebSocket server")
            return ssl_context
        except ssl.SSLError as e:
            logger.error(f"Error loading SSL cert/key: {e}")
            return None

    async def start(self):
        """Start the WebSocket server."""
        if not self.config.enable_vr:
            logger.info("VR WebSocket server disabled in configuration")
            return

        ssl_context = self.setup_ssl()
        if ssl_context is None:
            logger.error("Failed to setup SSL for WebSocket server")
            return

        host = self.config.host_ip
        port = self.config.websocket_port

        try:
            self.server = await websockets.serve(self.websocket_handler, host, port, ssl=ssl_context)
            self.is_running = True
            logger.info(f"VR WebSocket server running on wss://{host}:{port}")
        except Exception as e:
            logger.error(f"Failed to start WebSocket server: {e}")

    async def stop(self):
        """Stop the WebSocket server."""
        self.is_running = False
        if self.server:
            self.server.close()
            await self.server.wait_closed()
            logger.info("VR WebSocket server stopped")

    async def websocket_handler(self, websocket, path=None):
        """Handle WebSocket connections from VR controllers."""
        client_address = websocket.remote_address
        logger.info(f"VR client connected: {client_address}")
        self.clients.add(websocket)

        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                    await self.process_controller_data(data)
                except json.JSONDecodeError:
                    logger.warning(f"Received non-JSON message: {message}")
                except Exception as e:
                    logger.error(f"Error processing VR data: {e}")

        except websockets.exceptions.ConnectionClosedOK:
            logger.info(f"VR client {client_address} disconnected normally")
        except websockets.exceptions.ConnectionClosedError as e:
            logger.warning(f"VR client {client_address} disconnected with error: {e}")
        except Exception as e:
            logger.error(f"Unexpected error with VR client {client_address}: {e}")
        finally:
            self.clients.discard(websocket)
            # Handle grip releases when client disconnects
            await self.handle_grip_release("left")
            await self.handle_grip_release("right")
            logger.info(f"VR client {client_address} cleanup complete")

    async def process_controller_data(self, data: Dict):
        """Process incoming VR controller data."""

        # Handle new dual controller format
        if "leftController" in data and "rightController" in data:
            left_data = data["leftController"]
            right_data = data["rightController"]

            # Process left controller
            if left_data.get("position") and (left_data.get("gripActive", False) or left_data.get("trigger", 0) > 0.5):
                await self.process_single_controller("left", left_data)
            elif not left_data.get("gripActive", False) and self.left_controller.grip_active:
                await self.handle_grip_release("left")

            # Process right controller
            if right_data.get("position") and (
                right_data.get("gripActive", False) or right_data.get("trigger", 0) > 0.5
            ):
                await self.process_single_controller("right", right_data)
            elif not right_data.get("gripActive", False) and self.right_controller.grip_active:
                await self.handle_grip_release("right")

            return

        # Handle legacy single controller format
        hand = data.get("hand")

        # Handle explicit release messages
        if data.get("gripReleased"):
            await self.handle_grip_release(hand)
            return

        if data.get("triggerReleased"):
            await self.handle_trigger_release(hand)
            return

        # Process single controller data
        if hand and data.get("position") and (data.get("gripActive", False) or data.get("trigger", 0) > 0.5):
            await self.process_single_controller(hand, data)

    async def process_single_controller(self, hand: str, data: Dict):
        """Process data for a single controller."""
        position = data.get("position", {})
        quaternion = data.get("quaternion", {})  # Get quaternion data directly
        grip_active = data.get("gripActive", False)
        trigger = data.get("trigger", 0)

        assert quaternion is not None and all(k in quaternion for k in ["x", "y", "z", "w"]), "Quaternion data missing"
        quaternion = np.array([quaternion["x"], quaternion["y"], quaternion["z"], quaternion["w"]])
        position = np.array([position["x"], position["y"], position["z"]])

        transform = pose2transform(position, quaternion)
        transform = convert_to_robot_convention(transform)

        controller = self.left_controller if hand == "left" else self.right_controller

        # Handle trigger for gripper control
        trigger_active = trigger > 0.5
        if trigger_active != controller.trigger_active:
            controller.trigger_active = trigger_active

            # Send gripper control goal - do not specify mode to avoid interfering with position control
            # Reverse behavior: gripper open by default, closes when trigger pressed
            gripper_goal = ControlGoal(
                arm=hand,
                gripper_closed=not trigger_active,  # Inverted: closed when trigger NOT active
                metadata={"source": "vr_trigger"},
            )
            await self.send_goal(gripper_goal)

            logger.info(f"🤏 {hand.upper()} gripper {'OPENED' if trigger_active else 'CLOSED'}")

        # Handle grip button for arm movement control
        if grip_active:
            if not controller.grip_active:
                # Grip just activated - set origin and reset target position
                controller.grip_active = True
                controller.origin_transform = transform.copy()

                # Send reset signal to control loop to reset target position to current robot position
                reset_goal = ControlGoal(
                    arm=hand,
                    mode=ControlMode.POSITION_CONTROL,  # Keep in position control
                    target_transform=None,  # Special signal
                    metadata={
                        "source": f"vr_grip_reset_{hand}",
                        "reset_target_to_zero": True,  # Signal to reset target to current position
                    },
                )
                await self.send_goal(reset_goal)

                logger.info(
                    f"🔒 {hand.upper()} grip activated - controlling {hand} arm (target reset to current position)"
                )

            # Compute target position
            if controller.origin_transform is not None:
                relative_transform = compute_relative_transform(transform, controller.origin_transform)

                # Create control goal with relative transform
                goal = ControlGoal(
                    arm=hand,
                    mode=ControlMode.POSITION_CONTROL,
                    target_transform=relative_transform,
                    metadata={
                        "source": "vr_grip",
                        "relative_transform": True,
                        "origin_transform": controller.origin_transform.copy(),
                    },
                )
                await self.send_goal(goal)

    async def handle_grip_release(self, hand: str):
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
            goal = ControlGoal(arm=hand, mode=ControlMode.IDLE, metadata={"source": "vr_grip_release"})
            await self.send_goal(goal)

            logger.info(f"🔓 {hand.upper()} grip released - arm control stopped")

    async def handle_trigger_release(self, hand: str):
        """Handle trigger release for a controller."""
        controller = self.left_controller if hand == "left" else self.right_controller

        if controller.trigger_active:
            controller.trigger_active = False

            # Send gripper closed goal - reversed behavior: gripper closes when trigger released
            goal = ControlGoal(
                arm=hand,
                gripper_closed=True,  # Close gripper when trigger released
                metadata={"source": "vr_trigger_release"},
            )
            await self.send_goal(goal)

            logger.info(f"🤏 {hand.upper()} gripper CLOSED (trigger released)")


def convert_to_robot_convention(transform_vr: np.ndarray) -> np.ndarray:
    """Convert position and quaternion to robot convention."""

    adj_mat = np.array([[0, 0, -1, 0], [-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]])

    # TODO: figure out what this weird transform is
    r_adj = xyzrpy2transform(0, 0, 0, -np.pi, 0, -np.pi / 2)
    transform_robot = adj_mat @ transform_vr
    transform_robot = np.dot(transform_robot, r_adj)
    return transform_robot


def compute_relative_transform(current_transform: np.ndarray, origin_transform: np.ndarray) -> np.ndarray:
    """Compute relative transform from VR origin to current position."""
    return np.linalg.inv(origin_transform) @ current_transform
