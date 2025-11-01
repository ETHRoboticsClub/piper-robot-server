"""Lightweight interfaces for interacting with Piper robot arms."""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Mapping, MutableMapping, Optional

from piper_teleop.robot_server.core.piper_sdk_interface import PiperSDKInterface

logger = logging.getLogger(__name__)

JOINT_NAMES: tuple[str, ...] = tuple(f"joint_{i}.pos" for i in range(7))


@dataclass
class ArmState:
    """State tracking for a Piper arm."""

    name: str
    port: str
    enabled: bool = True
    dry_run: bool = False
    sdk: Optional[PiperSDKInterface] = None
    connected: bool = False
    last_observation: MutableMapping[str, float] = field(default_factory=dict)
    last_action: MutableMapping[str, float] = field(
        default_factory=lambda: {joint: 0.0 for joint in JOINT_NAMES}
    )
    last_timestamp: float = field(default_factory=time.time)


class PiperArm:
    """Minimal wrapper around a Piper arm."""

    def __init__(self, state: ArmState):
        self._state = state

    @property
    def name(self) -> str:
        return self._state.name

    @property
    def joint_names(self) -> tuple[str, ...]:
        return JOINT_NAMES

    @property
    def connected(self) -> bool:
        return self._state.connected

    def connect(self) -> None:
        if not self._state.enabled:
            logger.info("Arm %s disabled via configuration; skipping connection.", self._state.name)
            return
        if self._state.connected:
            return
        if self._state.dry_run:
            logger.info("Arm %s running in dry-run mode.", self._state.name)
            self._state.connected = True
            return
        logger.info("Connecting Piper arm %s on port %s", self._state.name, self._state.port)
        self._state.sdk = PiperSDKInterface(port=self._state.port)
        self._state.connected = True

    def disconnect(self) -> None:
        if not self._state.connected:
            return
        if self._state.sdk is not None:
            self._state.sdk.disconnect()
        self._state.connected = False

    def get_observation(self) -> Mapping[str, float]:
        """Return latest joint observation (radians)."""
        if not self._state.connected:
            raise RuntimeError(f"Arm {self._state.name} is not connected.")
        if self._state.dry_run or self._state.sdk is None:
            self._state.last_timestamp = time.time()
            return dict(self._state.last_action)
        try:
            obs = self._state.sdk.get_status()
        except Exception as exc:  # pragma: no cover - hardware specific
            logger.error("Failed to read observation for arm %s: %s", self._state.name, exc)
            return dict(self._state.last_observation) or dict(self._state.last_action)
        else:
            self._state.last_observation.update(obs)
            self._state.last_timestamp = time.time()
            return dict(self._state.last_observation)

    def last_action(self) -> Mapping[str, float]:
        return dict(self._state.last_action)

    def send_action(self, action: Mapping[str, float]) -> None:
        """Send a joint position command (radians) to the arm."""
        if not self._state.connected:
            raise RuntimeError(f"Arm {self._state.name} is not connected.")

        joint_targets = [float(action.get(joint, self._state.last_action[joint])) for joint in JOINT_NAMES]
        self._state.last_action.update({joint: value for joint, value in zip(JOINT_NAMES, joint_targets)})

        if self._state.dry_run or self._state.sdk is None:
            return
        try:
            self._state.sdk.set_joint_positions(joint_targets)
        except Exception as exc:  # pragma: no cover - hardware specific
            logger.error("Failed to send action to arm %s: %s", self._state.name, exc)

    def get_end_effector_pose(self) -> Mapping[str, float]:
        if not self._state.connected:
            raise RuntimeError(f"Arm {self._state.name} is not connected.")
        if self._state.dry_run or self._state.sdk is None:
            return {"x": 0.0, "y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}
        try:
            return self._state.sdk.get_end_effector_pose()
        except Exception as exc:  # pragma: no cover - hardware specific
            logger.error("Failed to query end-effector pose for arm %s: %s", self._state.name, exc)
            return {"x": 0.0, "y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}


class PiperRobot:
    """Manages a collection of Piper arms."""

    def __init__(self, arms: Iterable[PiperArm]):
        self._arms = list(arms)

    def connect(self) -> None:
        for arm in self._arms:
            arm.connect()

    def disconnect(self) -> None:
        for arm in self._arms:
            arm.disconnect()

    @property
    def arms(self) -> List[PiperArm]:
        return self._arms

    def observations(self) -> Dict[str, Mapping[str, float]]:
        return {f"{arm.name}_arm": arm.get_observation() for arm in self._arms}

    def send_actions(self, action_dict: Mapping[str, Mapping[str, float]]) -> None:
        for arm in self._arms:
            arm_action = action_dict.get(f"{arm.name}_arm")
            if arm_action is None:
                logger.warning("Missing action for arm %s; skipping.", arm.name)
                continue
            arm.send_action(arm_action)
