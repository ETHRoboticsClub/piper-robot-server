# Implementation of Piper robot for LeRobot

from dataclasses import dataclass, field
from typing import Any

import numpy as np
from lerobot.cameras import CameraConfig, make_cameras_from_configs
from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.robots import Robot, RobotConfig

from .geometry import xyzrpy2transform
from .piper_sdk_interface import PiperSDKInterface


@RobotConfig.register_subclass("piper")
@dataclass
class PiperConfig(RobotConfig):
    port: str
    cameras: dict[str, CameraConfig] = field(
        default_factory=lambda: {
            "cam_1": OpenCVCameraConfig(
                index_or_path=0,
                fps=30,
                width=640,
                height=480,
            ),
        }
    )


class Piper(Robot):
    config_class = PiperConfig
    name = "piper"

    def __init__(self, config: PiperConfig):
        super().__init__(config)
        self.sdk = PiperSDKInterface(port=config.port)
        self.cameras = make_cameras_from_configs(config.cameras)

    @property
    def _motors_ft(self) -> dict[str, type]:
        return {f"joint_{i}.pos": float for i in range(7)}

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {cam: (self.cameras[cam].height, self.cameras[cam].width, 3) for cam in self.cameras}

    @property
    def observation_features(self) -> dict:
        return {**self._motors_ft, **self._cameras_ft}

    @property
    def action_features(self) -> dict:
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        # Assume always connected after SDK init
        return self.sdk.get_connection_status()

    def connect(self, calibrate: bool = True) -> None:
        # Already connected in SDK init
        for cam in self.cameras.values():
            cam.connect()
        self.configure()

    def disconnect(self) -> None:
        self.sdk.disconnect()
        for cam in self.cameras.values():
            cam.disconnect()

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def get_observation(self) -> dict[str, Any]:
        obs_dict = self.sdk.get_status()

        for cam_key, cam in self.cameras.items():
            obs_dict[cam_key] = cam.async_read()
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
