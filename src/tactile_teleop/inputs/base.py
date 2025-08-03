"""
Base classes and data structures for input providers.
"""

import asyncio
from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, Literal, Optional

import numpy as np


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


class BaseInputProvider(ABC):
    """Abstract base class for input providers."""

    def __init__(self, command_queue: asyncio.Queue):
        self.command_queue = command_queue
        self.is_running = False

    @abstractmethod
    async def start(self):
        """Start the input provider."""
        pass

    @abstractmethod
    async def stop(self):
        """Stop the input provider."""
        pass

    async def send_goal(self, goal: ControlGoal):
        """Send a control goal to the command queue."""
        try:
            await self.command_queue.put(goal)
        except Exception as e:
            # Handle queue full or other errors
            pass
