"""
Configuration module for the unified teleoperation system.
Contains all constants, port numbers, SSL paths, joint mappings, and other settings.
"""

import os
from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional
import numpy as np

# --- Network Configuration ---
HTTPS_PORT = 8443
WEBSOCKET_PORT = 8442
HOST_IP = "0.0.0.0"

# --- SSL Configuration ---
CERTFILE = "cert.pem"
KEYFILE = "key.pem"

# --- Robot Configuration ---
VR_TO_ROBOT_SCALE = 1.0  # Scale factor for VR movements
SEND_INTERVAL = 0.05     # Send robot commands every 50ms (20Hz)
POSITION_SMOOTHING = 0.1 # Smoothing factor for position updates

# --- Joint Configuration ---
JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
NUM_JOINTS = len(JOINT_NAMES)
NUM_IK_JOINTS = 3  # Use only first 3 joints for IK (Rotation, Pitch, Elbow)
WRIST_ROLL_INDEX = 4
GRIPPER_INDEX = 5

# Motor configuration for SO100
COMMON_MOTORS = {
    "shoulder_pan": [1, "sts3215"],
    "shoulder_lift": [2, "sts3215"], 
    "elbow_flex": [3, "sts3215"],
    "wrist_flex": [4, "sts3215"],
    "wrist_roll": [5, "sts3215"],
    "gripper": [6, "sts3215"],
}

# URDF joint name mapping
URDF_TO_INTERNAL_NAME_MAP = {
    "Rotation": "shoulder_pan",
    "Pitch": "shoulder_lift",
    "Elbow": "elbow_flex",
    "Wrist_Pitch": "wrist_flex",
    "Wrist_Roll": "wrist_roll",
    "Jaw": "gripper",
}

# --- Gripper Control ---
GRIPPER_OPEN_ANGLE = 0.0    # Degrees - gripper open position
GRIPPER_CLOSED_ANGLE = 45.0 # Degrees - gripper closed position

# --- PyBullet Configuration ---
URDF_PATH = "URDF/SO_5DOF_ARM100_8j/urdf/so100.urdf"
END_EFFECTOR_LINK_NAME = "Wrist_Pitch_Roll"

# --- Keyboard Control ---
POS_STEP = 0.01  # meters
ANGLE_STEP = 5.0 # degrees
GRIPPER_STEP = 10.0 # degrees

# --- Device Ports ---
DEFAULT_FOLLOWER_PORTS = {
    "left": "/dev/ttySO100red",
    "right": "/dev/ttySO100blue"
}

@dataclass
class TeleopConfig:
    """Main configuration class for the teleoperation system."""
    
    # Network settings
    https_port: int = HTTPS_PORT
    websocket_port: int = WEBSOCKET_PORT
    host_ip: str = HOST_IP
    
    # SSL settings
    certfile: str = CERTFILE
    keyfile: str = KEYFILE
    
    # Robot settings
    vr_to_robot_scale: float = VR_TO_ROBOT_SCALE
    send_interval: float = SEND_INTERVAL
    position_smoothing: float = POSITION_SMOOTHING
    
    # Device ports
    follower_ports: Dict[str, str] = None
    
    # Control flags
    enable_pybullet: bool = True
    enable_robot: bool = True
    enable_vr: bool = True
    enable_keyboard: bool = True
    
    # Paths
    urdf_path: str = URDF_PATH
    webapp_dir: str = "webapp"
    
    def __post_init__(self):
        if self.follower_ports is None:
            self.follower_ports = DEFAULT_FOLLOWER_PORTS.copy()
    
    @property
    def ssl_files_exist(self) -> bool:
        """Check if SSL certificate files exist."""
        return os.path.exists(self.certfile) and os.path.exists(self.keyfile)
    
    def ensure_ssl_certificates(self) -> bool:
        """Ensure SSL certificates exist, generating them if necessary."""
        from .utils import ensure_ssl_certificates
        return ensure_ssl_certificates(self.certfile, self.keyfile)
    
    @property
    def urdf_exists(self) -> bool:
        """Check if URDF file exists."""
        return os.path.exists(self.urdf_path)
    
    @property
    def webapp_exists(self) -> bool:
        """Check if webapp directory exists."""
        return os.path.exists(self.webapp_dir)

# Global configuration instance
config = TeleopConfig() 