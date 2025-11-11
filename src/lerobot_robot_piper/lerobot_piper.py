import cv2
from dataclasses import dataclass, field
from typing import Any, Optional

import numpy as np
from lerobot.cameras import CameraConfig, make_cameras_from_configs
from lerobot.cameras.opencv import OpenCVCameraConfig, OpenCVCamera
from lerobot.robots import Robot, RobotConfig

from piper_teleop.config import TelegripConfig
from piper_teleop.robot_server.camera import CameraType
from piper_teleop.robot_server.camera.stereo_camera import crop_stereo_image
from piper_teleop.robot_server.core import RobotInterface

import piper_teleop.robot_server.camera as  piper_camera

config_global = TelegripConfig()


def _default_cameras():
    cameras = {
        c.name: OpenCVCameraConfig(
            index_or_path=c.cam_index,
            fps=30,
            width=c.frame_width if not c.type == CameraType.STEREO else c.capture_frame_width,
            height=c.frame_height if not c.type == CameraType.STEREO else c.capture_frame_height,
        )
        for c in config_global.camera_configs
    }
    return cameras

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
        self.stereo_config: Optional[piper_camera.CameraConfig] = self._get_stereo_cam_settings()
        self.robot = RobotInterface(config_global)
        self.cameras = make_cameras_from_configs(config.cameras)
        self.joints = [f"L.joint_{i}" for i in range(dof_arm)] + [f"R.joint_{i}" for i in range(dof_arm)]
        self.dof_arm = dof_arm

    def _get_stereo_cam_settings(self):
        stereo_cams = [c for c in config_global.camera_configs if c.type == CameraType.STEREO]
        assert len(stereo_cams) <= 1
        if len(stereo_cams) == 1:
            return stereo_cams[0]
        else:
            return None

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
            for name, cam in self.cameras.items():
                cam.connect()
                if 'stereo' in name:
                    cam: OpenCVCamera = cam
                    cam.videocapture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
                cam.videocapture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
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
                frame_raw = cam.async_read()
                if 'stereo' in cam_key:
                    # Edge Case cropping
                    frame, frame_rgb_right, cropped_left, cropped_right = crop_stereo_image(frame=frame_raw,
                                      frame_width=self.stereo_config.frame_width,
                                      frame_height=self.stereo_config.frame_height,
                                      edge_crop=self.stereo_config.edge_crop)
                else:
                    frame = frame_raw
                obs_dict[cam_key] = frame
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
