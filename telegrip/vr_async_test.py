"""
VR WebSocket server for receiving controller data from web browsers.
Adapted from the original vr_robot_teleop.py script.
"""

import asyncio
import json
import logging
import ssl
import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Dict, List, Optional, Set

import numpy as np
import websockets

from .config import TelegripConfig, config
from .core.geometry import pose2transform, xyzrpy2transform
from .core.robot_interface import RobotInterface
from .inputs.base import BaseInputProvider

logger = logging.getLogger(__name__)
import http.server


class EventType(Enum):
    """Control goal types."""

    IDLE = "idle"  # No button on vr controller pressed
    GRIP_ACTIVE_INIT = "grip_active_init"  # Grip button pressed first time
    GRIP_ACTIVE = "grip_active"  # Grip button held
    GRIP_RELEASE = "grip_release"  # Grip button released
    TRIGGER_ACTIVE = "trigger_active"  # Trigger button pressed
    TRIGGER_RELEASE = "trigger_release"  # Trigger button released
    RESET_BUTTON_RELEASE = "reset_button_release"  # Reset button released


@dataclass
class ControlGoal:
    """Control goal."""

    event_type: EventType
    arm: str
    vr_reference_transform: Optional[np.ndarray] = None
    vr_target_transform: Optional[np.ndarray] = None
    gripper_closed: Optional[bool] = None


class ArmState:
    """State tracking for a single robot arm."""

    def __init__(self, arm_name: str):
        self.arm_name = arm_name
        self.initial_transform = xyzrpy2transform(0.19, 0.0, 0.2, 0, 0, 0)
        self.origin_transform = None
        self.target_transform = None
        self.gripper_closed = True


class APIHandler(http.server.BaseHTTPRequestHandler):
    """HTTP request handler for the teleoperation API."""

    def __init__(self, *args, **kwargs):
        # Set CORS headers for all requests
        super().__init__(*args, **kwargs)

    def end_headers(self):
        """Add CORS headers to all responses."""
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        try:
            super().end_headers()
        except (
            BrokenPipeError,
            ConnectionResetError,
            ConnectionAbortedError,
            ssl.SSLError,
        ):
            # Client disconnected or SSL error - ignore silently
            pass

    def do_OPTIONS(self):
        """Handle preflight CORS requests."""
        self.send_response(200)
        self.end_headers()

    def log_message(self, format, *args):
        """Override to reduce HTTP request logging noise."""
        pass  # Disable default HTTP logging

    def do_GET(self):
        """Handle GET requests."""
        if self.path == "/" or self.path == "/index.html":
            # Serve main page from web-ui directory
            self.serve_file("web-ui/index.html", "text/html")
        elif self.path.endswith(".css"):
            # Serve CSS files from web-ui directory
            self.serve_file(f"web-ui{self.path}", "text/css")
        elif self.path.endswith(".js"):
            # Serve JS files from web-ui directory
            self.serve_file(f"web-ui{self.path}", "application/javascript")
        elif self.path.endswith(".ico"):
            self.serve_file(self.path[1:], "image/x-icon")
        elif self.path.endswith((".jpg", ".jpeg")):
            # Serve image files from web-ui directory
            self.serve_file(f"web-ui{self.path}", "image/jpeg")
        elif self.path.endswith(".png"):
            # Serve image files from web-ui directory
            self.serve_file(f"web-ui{self.path}", "image/png")
        elif self.path.endswith(".gif"):
            # Serve image files from web-ui directory
            self.serve_file(f"web-ui{self.path}", "image/gif")
        else:
            self.send_error(404, "Not found")

    def serve_file(self, filename, content_type):
        """Serve a static file from the project directory."""
        from telegrip.utils import get_absolute_path

        try:
            # Convert relative path to absolute path in project directory
            abs_path = get_absolute_path(filename)

            with open(abs_path, "rb") as f:
                file_content = f.read()

            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", len(file_content))
            self.end_headers()
            self.wfile.write(file_content)

        except FileNotFoundError:
            self.send_error(404, f"File {filename} not found")
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            # Client disconnected - log quietly and continue
            logger.debug(f"Client disconnected while serving {filename}")
        except Exception as e:
            logger.error(f"Error serving file {filename}: {e}")
            try:
                self.send_error(500, "Internal server error")
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                # Client already disconnected, ignore
                pass


class HTTPSServer:
    """HTTPS server for the teleoperation API."""

    def __init__(self, config: TelegripConfig):
        self.config = config
        self.httpd = None
        self.server_thread = None
        self.system_ref = None  # Direct reference to the main system

    def set_system_ref(self, system_ref):
        """Set reference to the main teleoperation system."""
        self.system_ref = system_ref

    async def start(self):
        """Start the HTTPS server."""
        try:
            # Create server - directly use APIHandler class
            self.httpd = http.server.HTTPServer((self.config.host_ip, self.config.https_port), APIHandler)

            # Set API handler reference for command queuing
            self.httpd.api_handler = self.system_ref

            # Setup SSL
            context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            # Get absolute paths for SSL certificates
            cert_path, key_path = self.config.get_absolute_ssl_paths()
            context.load_cert_chain(cert_path, key_path)
            self.httpd.socket = context.wrap_socket(self.httpd.socket, server_side=True)

            # Start server in a separate thread
            self.server_thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
            self.server_thread.start()

            # Only log if INFO level or more verbose
            if getattr(logging, self.config.log_level.upper()) <= logging.INFO:
                logger.info(f"HTTPS server started on {self.config.host_ip}:{self.config.https_port}")

        except Exception as e:
            logger.error(f"Failed to start HTTPS server: {e}")
            raise

    async def stop(self):
        """Stop the HTTPS server."""
        if self.httpd:
            self.httpd.shutdown()
            if self.server_thread:
                self.server_thread.join(timeout=5)
            logger.info("HTTPS server stopped")


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

        # Simple frequency tracking
        self.msg_count = 0
        self.start_time = None

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
                    # Simple frequency tracking
                    self.msg_count += 1
                    if self.start_time is None:
                        self.start_time = time.time()
                    elif self.msg_count % 100 == 0:  # Log every 100 messages
                        elapsed = time.time() - self.start_time
                        freq = self.msg_count / elapsed
                        print(f"📊 Message frequency: {freq:.1f} Hz ({self.msg_count} msgs)")
                        self.msg_count = 0
                        self.start_time = None

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

        if data.get("resetEvent"):
            await self.handle_reset_button_release(hand)
            return

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

            logger.info(f"🤏 {hand.upper()} gripper {'OPENED' if trigger_active else 'CLOSED'}")

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

                logger.info(
                    f"🔒 {hand.upper()} grip activated - controlling {hand} arm (target reset to current position)"
                )

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
            goal = ControlGoal(
                event_type=EventType.GRIP_RELEASE,
                arm=hand,
            )
            await self.send_goal(goal)

            logger.info(f"🔓 {hand.upper()} grip released - arm control stopped")

    async def handle_trigger_release(self, hand: str):
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

            logger.info(f"🤏 {hand.upper()} gripper CLOSED (trigger released)")

    async def handle_reset_button_release(self, hand: str):
        """Handle X button release for a controller."""
        goal = ControlGoal(
            event_type=EventType.RESET_BUTTON_RELEASE,
            arm=hand,
        )
        await self.send_goal(goal)

        logger.info(f"🔓 {hand.upper()} reset button released - going to initial position")


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
        vr_reference_transform = convert_to_robot_convention(
            last_grip_active_goal.vr_reference_transform  # type: ignore
        )
        vr_target_transform = convert_to_robot_convention(last_grip_active_goal.vr_target_transform)  # type: ignore
        relative_transform = np.linalg.inv(vr_reference_transform) @ vr_target_transform
        arm_state.target_transform = arm_state.origin_transform @ relative_transform

    return arm_state


def update_robot(left_arm: ArmState, right_arm: ArmState, robot_interface: RobotInterface, robot_enabled: bool = False):
    """Update robot with current control goals."""
    start_time_total = time.time()
    # Measure all IK time together
    start_time_ik = time.time()

    # Left arm IK
    if left_arm.target_transform is not None:
        ik_solution = robot_interface.solve_ik("left", left_arm.target_transform)
        current_gripper = 0.0 if left_arm.gripper_closed else 0.07
        robot_interface.update_arm_angles("left", np.concatenate([ik_solution, [current_gripper]]))

    # Right arm IK
    if right_arm.target_transform is not None:
        ik_solution = robot_interface.solve_ik("right", right_arm.target_transform)
        current_gripper = 0.0 if right_arm.gripper_closed else 0.07
        robot_interface.update_arm_angles("right", np.concatenate([ik_solution, [current_gripper]]))

    ik_time = time.time() - start_time_ik

    # Send commands
    start_time_send = time.time()
    if robot_enabled:
        robot_interface.send_command()
    send_time = time.time() - start_time_send

    total_time = time.time() - start_time_total
    overhead_time = total_time - ik_time - send_time

    # Print all at once to minimize timing impact
    print(
        f"IK: {ik_time*1000:.1f}ms, CAN: {send_time*1000:.1f}ms, "
        f"Overhead: {overhead_time*1000:.1f}ms, Total: {total_time*1000:.1f}ms"
    )


async def control_loop(command_queue: asyncio.Queue, config: TelegripConfig, robot_enabled: bool = True):
    """Control loop for the teleoperation system."""
    left_arm = ArmState("left")
    right_arm = ArmState("right")
    robot_interface = RobotInterface(config)
    robot_interface.setup_kinematics()
    if robot_enabled:
        robot_interface.connect()
        robot_interface.return_to_initial_position()

    while True:
        iteration_start = time.time()
        commands_start = time.time()
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
        commands_time = time.time() - commands_start

        # Simulates blocking robot communication
        robot_start = time.time()
        update_robot(left_arm, right_arm, robot_interface, robot_enabled)
        robot_time = time.time() - robot_start

        sleep_start = time.time()
        await asyncio.sleep(0.005)
        sleep_time = time.time() - sleep_start

        total_time = time.time() - iteration_start
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


async def main():
    command_queue = asyncio.Queue()
    https_server = HTTPSServer(config)
    vr_server = VRWebSocketServer(command_queue, config)

    await https_server.start()
    await vr_server.start()

    control_loop_task = asyncio.create_task(control_loop(command_queue, config))

    await asyncio.sleep(10000)
    await vr_server.stop()
    await https_server.stop()
    control_loop_task.cancel()


if __name__ == "__main__":
    asyncio.run(main())
