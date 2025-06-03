"""
VR WebSocket server for receiving controller data.
Adapted from the original vr_robot_teleop.py script.
"""

import asyncio
import json
import ssl
import websockets
import numpy as np
import math
import logging
from typing import Dict, Optional, Set
from scipy.spatial.transform import Rotation as R

from .base import BaseInputProvider, ControlGoal, ControlMode
from ..config import TeleopConfig
from ..core.kinematics import compute_relative_position

logger = logging.getLogger(__name__)


class VRControllerState:
    """State tracking for a single VR controller."""
    
    def __init__(self, hand: str):
        self.hand = hand
        self.grip_active = False
        self.trigger_active = False
        
        # Position tracking
        self.origin_position = None
        self.current_position = None
        
        # Rotation tracking
        self.origin_rotation = None
        self.z_axis_rotation = 0.0
        self.origin_wrist_angle = 0.0
        
    def reset_grip(self):
        """Reset grip-related state."""
        self.grip_active = False
        self.origin_position = None
        self.origin_rotation = None
        self.z_axis_rotation = 0.0
        self.origin_wrist_angle = 0.0


class VRWebSocketServer(BaseInputProvider):
    """WebSocket server for VR controller input."""
    
    def __init__(self, command_queue: asyncio.Queue, config: TeleopConfig):
        super().__init__(command_queue)
        self.config = config
        self.clients: Set = set()
        self.server = None
        
        # Controller states
        self.left_controller = VRControllerState("left")
        self.right_controller = VRControllerState("right")
        
        # Robot state tracking (for relative position calculation)
        self.left_arm_origin_position = None
        self.right_arm_origin_position = None
    
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
            ssl_context.load_cert_chain(certfile=self.config.certfile, keyfile=self.config.keyfile)
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
            self.server = await websockets.serve(
                self.websocket_handler, 
                host, 
                port, 
                ssl=ssl_context
            )
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
            await self.handle_grip_release('left')
            await self.handle_grip_release('right')
            logger.info(f"VR client {client_address} cleanup complete")
    
    async def process_controller_data(self, data: Dict):
        """Process incoming VR controller data."""
        
        # Handle new dual controller format
        if 'leftController' in data and 'rightController' in data:
            left_data = data['leftController']
            right_data = data['rightController']
            
            # Process left controller
            if left_data.get('position') and (left_data.get('gripActive', False) or left_data.get('trigger', 0) > 0.5):
                await self.process_single_controller('left', left_data)
            elif not left_data.get('gripActive', False) and self.left_controller.grip_active:
                await self.handle_grip_release('left')
            
            # Process right controller
            if right_data.get('position') and (right_data.get('gripActive', False) or right_data.get('trigger', 0) > 0.5):
                await self.process_single_controller('right', right_data)
            elif not right_data.get('gripActive', False) and self.right_controller.grip_active:
                await self.handle_grip_release('right')
                
            return
        
        # Handle legacy single controller format
        hand = data.get('hand')
        
        # Handle explicit release messages
        if data.get('gripReleased'):
            await self.handle_grip_release(hand)
            return
        
        if data.get('triggerReleased'):
            await self.handle_trigger_release(hand)
            return
            
        # Process single controller data
        if hand and data.get('position') and (data.get('gripActive', False) or data.get('trigger', 0) > 0.5):
            await self.process_single_controller(hand, data)
    
    async def process_single_controller(self, hand: str, data: Dict):
        """Process data for a single controller."""
        position = data.get('position', {})
        rotation = data.get('rotation', {})
        grip_active = data.get('gripActive', False)
        trigger = data.get('trigger', 0)
        
        controller = self.left_controller if hand == 'left' else self.right_controller
        
        # Handle trigger for gripper control
        trigger_active = trigger > 0.5
        if trigger_active != controller.trigger_active:
            controller.trigger_active = trigger_active
            
            # Send gripper control goal
            gripper_goal = ControlGoal(
                arm=hand,
                mode=ControlMode.IDLE,
                gripper_closed=trigger_active,
                metadata={"source": "vr_trigger"}
            )
            await self.send_goal(gripper_goal)
            
            logger.info(f"🤏 {hand.upper()} gripper {'CLOSED' if trigger_active else 'OPENED'}")
        
        # Handle grip button for arm movement control
        if grip_active:
            if not controller.grip_active:
                # Grip just activated - set origin
                controller.grip_active = True
                controller.origin_position = position.copy()
                controller.origin_rotation = self.euler_to_quaternion(rotation) if rotation else None
                controller.z_axis_rotation = 0.0
                
                # TODO: Get current robot end effector position for origin
                # This would require communication with the robot interface
                logger.info(f"🔒 {hand.upper()} grip activated - controlling {hand} arm")
            
            # Compute target position
            if controller.origin_position:
                relative_delta = compute_relative_position(
                    position, 
                    controller.origin_position, 
                    self.config.vr_to_robot_scale
                )
                
                # Calculate Z-axis rotation for wrist_roll control
                if controller.origin_rotation is not None and rotation:
                    current_quat = self.euler_to_quaternion(rotation)
                    controller.z_axis_rotation = self.calculate_z_axis_rotation(
                        current_quat, controller.origin_rotation
                    )
                
                # Create position control goal
                # Note: We send relative position here, the control loop will handle
                # adding it to the robot's current position
                goal = ControlGoal(
                    arm=hand,
                    mode=ControlMode.POSITION_CONTROL,
                    target_position=relative_delta,  # Relative position delta
                    wrist_roll_deg=controller.z_axis_rotation,
                    metadata={
                        "source": "vr_grip",
                        "relative_position": True,
                        "origin_position": controller.origin_position.copy()
                    }
                )
                await self.send_goal(goal)
    
    async def handle_grip_release(self, hand: str):
        """Handle grip release for a controller."""
        if hand == 'left':
            controller = self.left_controller
        elif hand == 'right':
            controller = self.right_controller
        else:
            return
        
        if controller.grip_active:
            controller.reset_grip()
            
            # Send idle goal to stop arm control
            goal = ControlGoal(
                arm=hand,
                mode=ControlMode.IDLE,
                metadata={"source": "vr_grip_release"}
            )
            await self.send_goal(goal)
            
            logger.info(f"🔓 {hand.upper()} grip released - arm control stopped")
    
    async def handle_trigger_release(self, hand: str):
        """Handle trigger release for a controller."""
        controller = self.left_controller if hand == 'left' else self.right_controller
        
        if controller.trigger_active:
            controller.trigger_active = False
            
            # Send gripper open goal
            goal = ControlGoal(
                arm=hand,
                mode=ControlMode.IDLE,
                gripper_closed=False,
                metadata={"source": "vr_trigger_release"}
            )
            await self.send_goal(goal)
            
            logger.info(f"👐 {hand.upper()} gripper OPENED (trigger released)")
    
    def euler_to_quaternion(self, euler_deg: Dict[str, float]) -> np.ndarray:
        """Convert Euler angles in degrees to quaternion [x, y, z, w]."""
        euler_rad = [math.radians(euler_deg['x']), math.radians(euler_deg['y']), math.radians(euler_deg['z'])]
        rotation = R.from_euler('xyz', euler_rad)
        return rotation.as_quat()
    
    def calculate_z_axis_rotation(self, current_quat: np.ndarray, initial_quat: np.ndarray) -> float:
        """Calculate rotation around the local Z-axis (forward direction) in degrees."""
        try:
            current_rot = R.from_quat(current_quat)
            initial_rot = R.from_quat(initial_quat)
            
            relative_rot = current_rot * initial_rot.inv()
            forward_direction = current_rot.apply([0, 0, 1])
            
            angle_rad = relative_rot.magnitude()
            if angle_rad < 0.0001:
                return 0.0
            
            rotation_axis = relative_rot.as_rotvec() / angle_rad if angle_rad > 0 else np.array([0, 0, 1])
            projected_component = np.dot(rotation_axis, forward_direction)
            forward_rotation_rad = angle_rad * projected_component
            
            degrees = math.degrees(forward_rotation_rad)
            
            # Normalize to -180 to +180 range
            while degrees > 180:
                degrees -= 360
            while degrees < -180:
                degrees += 360
            
            return degrees
            
        except Exception as e:
            logger.warning(f"Error calculating Z-axis rotation: {e}")
            return 0.0 