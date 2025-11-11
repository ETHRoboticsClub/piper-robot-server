from dataclasses import dataclass, field
from typing import Any

import numpy as np
from lerobot.cameras import CameraConfig, make_cameras_from_configs
from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.robots import Robot, RobotConfig

from piper_teleop.config import TelegripConfig
from piper_teleop.robot_server.core import RobotInterface

config_global = TelegripConfig()


def _default_cameras():
    return {
        c.name: OpenCVCameraConfig(
            index_or_path=c.cam_index,
            fps=30,
            width=c.frame_width,
            height=c.frame_height,
        )
        for c in config_global.camera_configs
    }


@RobotConfig.register_subclass("piper")
@dataclass
class LerobotPiperConfig(RobotConfig):
    no_robot: bool = False
    cameras: dict[str, CameraConfig] = field(default_factory=_default_cameras)


class LerobotPiper(Robot):

    config_class = LerobotPiperConfig
    name = "piper"

    def __init__(self, config: LerobotPiperConfig, dof_arm=7):
        super().__init__(config)
        self.config = config
        self.robot = RobotInterface(config_global)
        self.cameras = make_cameras_from_configs(config.cameras)
        self.joints = [f"L.joint_{i}" for i in range(dof_arm)] + [f"R.joint_{i}" for i in range(dof_arm)]
        self.dof_arm = dof_arm

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {cam: (self.cameras[cam].height, self.cameras[cam].width, 3) for cam in self.cameras}

    def action_features(self) -> dict:
        return {j + ".pos": float for j in self.joints}

    def observation_features(self) -> dict:
        action_features = self.action_features()
        return {**action_features, **self._cameras_ft}

    def is_connected(self) -> bool:
        if self.config.no_robot:
            return True
        robot_connected = self.robot.is_connected
        cameras_connected = all(cam.is_connected for cam in self.cameras.values())
        return robot_connected and cameras_connected

    def connect(self, calibrate: bool = True) -> None:
        if not self.config.no_robot:
            for cam in self.cameras.values():
                cam.connect()
            self.robot.connect()
        else:
            self.robot.setup_kinematics()

    def disconnect(self) -> None:
        if not self.config.no_robot:
            self.robot.disconnect()
            for cam in self.cameras.values():
                cam.disconnect()

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        """Apply runtime configuration to the robot (no-op for Piper)."""
        pass

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise ConnectionError(f"{self} is not connected.")

        # Read arm position
        robot_obs = self.robot.get_observation()
        state = np.array(
            [robot_obs["left"][f"joint_{i}.pos"] for i in range(self.dof_arm)]
            + [robot_obs["right"][f"joint_{i}.pos"] for i in range(self.dof_arm)],
            dtype=np.float32,
        )
        obs_dict = dict(zip(self.joints, state))

        # Only read cameras if not in no_robot mode
        if not self.config.no_robot:
            for cam_key, cam in self.cameras.items():
                obs_dict[cam_key] = cam.async_read()

        return obs_dict

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        dict_left = dict()
        dict_right = dict()
        for name in action:
            if name[:2] == "L.":
                dict_left[name.replace("L.", "") + ".pos"] = action[name]
            elif name[:2] == "R.":
                dict_right[name.replace("R.", "") + ".pos"] = action[name]
            else:
                raise Exception(f"wrong name: {name}")

        if self.config.no_robot:
            q_1 = [dict_left[k] for k in sorted(dict_left)[:6]]
            q_2 = [dict_right[k] for k in sorted(dict_right)[:6]]
            self.robot.ik_solver.vis.display(np.array(q_1 + q_2))
        else:
            self.robot.left_robot.send_action({key: float(value) for key, value in dict_left.items()})
            self.robot.right_robot.send_action({key: float(value) for key, value in dict_right.items()})

        return action
