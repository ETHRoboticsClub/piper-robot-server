"""
telegrip - Unified Teleoperation Control System for SO100 robot arms

A unified system for controlling SO100 robot arms through various input methods 
including VR controllers, keyboard, or programmatic control. Features shared 
inverse kinematics, real-time visualization, and comprehensive robot control.
"""

__version__ = "0.9.0"
__author__ = "Emil Rofors"

from .config import TelegripConfig
from .core.robot_interface import RobotInterface
from .control_loop import ControlLoop
from .main import TelegripSystem

__all__ = [
    "TelegripConfig",
    "TelegripSystem",
    "RobotInterface", 
    "ControlLoop",
] 