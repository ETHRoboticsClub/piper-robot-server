# Implementation of Piper robot for LeRobot

from dataclasses import dataclass
from typing import Any

import numpy as np

#
from .geometry import xyzrpy2transform
from .piper_sdk_interface import PiperSDKInterface


@dataclass
class PiperConfig:
    port: str


class Piper:
    config_class = PiperConfig
    name = "piper"

    def __init__(self, config: PiperConfig):
        self.sdk = PiperSDKInterface(port=config.port)

    @property
    def _motors_ft(self) -> dict[str, type]:
        return {f"joint_{i}.pos": float for i in range(7)}

    @property
    def observation_features(self) -> dict:
        return {**self._motors_ft}

    @property
    def action_features(self) -> dict:
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        # Assume always connected after SDK init
        return self.sdk.get_connection_status()

    def connect(self, calibrate: bool = True) -> None:
        # Already connected in SDK init
        self.configure()

    def disconnect(self) -> None:
        self.sdk.disconnect()

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def get_observation(self) -> dict[str, Any]:
        obs_dict = self.sdk.get_status()
        return obs_dict

    def get_end_effector_transform(self) -> np.ndarray:

        raw_pose = self.sdk.get_end_effector_pose()
        raw_transform = xyzrpy2transform(
            raw_pose["x"], raw_pose["y"], raw_pose["z"], raw_pose["roll"], raw_pose["pitch"], raw_pose["yaw"]
        )
        link6_to_gripper_transform = xyzrpy2transform(0.0, 0.0, 0.13, 0.0, -1.57, 0.0)
        gripper_transform = raw_transform @ link6_to_gripper_transform
        return gripper_transform

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        # map the action from the leader to joints for the follower
        positions = [
            action.get("joint_0.pos"),
            action.get("joint_1.pos"),
            action.get("joint_2.pos"),
            action.get("joint_3.pos"),
            action.get("joint_4.pos"),
            action.get("joint_5.pos"),
            action.get("joint_6.pos"),
        ]

        self.sdk.set_joint_positions(positions)
        return action
