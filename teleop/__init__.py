"""
Unified Teleoperation Control System for SO100 Robot

This package provides a consolidated entry point for controlling the SO100 robot
via multiple input methods (VR controllers, keyboard, etc.) with shared
inverse kinematics, safety logic, and visualization.
"""

__version__ = "0.5.0"
__author__ = "Emil Rofors"

from .config import TeleopConfig
from .core.robot_interface import RobotInterface
from .control_loop import ControlLoop

__all__ = [
    "TeleopConfig",
    "RobotInterface", 
    "ControlLoop",
] 