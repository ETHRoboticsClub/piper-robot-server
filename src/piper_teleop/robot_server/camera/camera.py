import abc
import logging

import cv2

# Replace line 5:
from piper_teleop.robot_server.camera.camera_config import CameraConfig, CameraMode


class Camera(abc.ABC):
    """A base class for cameras."""

    logger: logging.Logger

    name: str
    mode: CameraMode
    fps: int
    frame_width: int
    frame_height: int
    cam_index: int
    edge_crop: int
    capture_api: int

    def __init__(self, camera_config: CameraConfig):
        self.name = camera_config.name
        self.logger = logging.getLogger(self.name)
        self.mode = camera_config.mode
        self.fps = camera_config.fps
        self.frame_width = camera_config.frame_width
        self.frame_height = camera_config.frame_height
        self.cam_index = camera_config.cam_index
        self.edge_crop = camera_config.edge_crop
        self.capture_api = camera_config.capture_api
        self.capture_frame_width = camera_config.capture_frame_width
        self.capture_frame_height = camera_config.capture_frame_height

    @abc.abstractmethod
    def get_cropped_width(self) -> int:
        raise NotImplementedError

    @abc.abstractmethod
    def is_connected(self):
        raise NotImplementedError

    @abc.abstractmethod
    def init_camera(self):
        raise NotImplementedError

    @abc.abstractmethod
    async def capture_frame(self):
        raise NotImplementedError

    @abc.abstractmethod
    def stop_camera(self):
        raise NotImplementedError
