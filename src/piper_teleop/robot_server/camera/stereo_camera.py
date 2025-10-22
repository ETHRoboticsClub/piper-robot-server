import cv2
import asyncio
from typing import Tuple

import numpy as np

from piper_teleop.robot_server.camera.camera import Camera
from piper_teleop.robot_server.camera.camera_config import CameraConfig


class StereoCamera(Camera):
    """Stereo camera class."""

    capture: cv2.VideoCapture

    def __init__(self, config: CameraConfig):
        super().__init__(config)
        self.capture = None

    def get_single_width(self) -> int:
        return self.frame_width // 2

    def get_cropped_width(self) -> int:
        return self.get_single_width() - self.edge_crop

    def is_connected(self) -> bool:
        """Returns True if the stereo camera is connected and False otherwise."""
        return self.capture is not None and self.capture.isOpened()

    def init_camera(self) -> None:
        """Initialises the stereo camera."""
        if self.capture is not None:  # already initialised
            return
        self.capture = cv2.VideoCapture(index=self.cam_index, apiPreference=self.capture_api)
        if not self.capture.isOpened():  # failed to open camera
            raise RuntimeError(f"failed to open camera {self.name} at index {self.cam_index}")

        # set camera properties
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        self.capture.set(cv2.CAP_PROP_FPS, self.fps)
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.logger.info(f"stereo camera initialised at index {self.cam_index}")

    async def capture_frame(self) -> Tuple[np.ndarray, np.ndarray]:
        """Captures a frame from the stereo camera and splits it into left and right frames."""
        loop = asyncio.get_event_loop()
        ret, frame = await loop.run_in_executor(None, self.capture.read)
        if not ret or frame is None:
            self.logger.warning("cannot receive frame (stream end?). Retrying...")
            await asyncio.sleep(0.1)
            return None, None

        # split frame into left and right columns
        mid = frame.shape[1] // 2
        frame_left = frame[:, :mid]
        frame_right = frame[:, mid:]

        # convert BGR to RGB for visualisation
        frame_rgb_left = cv2.cvtColor(frame_left, cv2.COLOR_BGR2RGB)
        frame_rgb_right = cv2.cvtColor(frame_right, cv2.COLOR_BGR2RGB)

        # ensure that the frames are correct size
        if frame_rgb_left.shape[:2] != (self.frame_height, self.frame_width):
            frame_rgb_left = cv2.resize(frame_rgb_left, (self.frame_width, self.frame_height))
        if frame_rgb_right.shape[:2] != (self.frame_height, self.frame_width):
            frame_rgb_right = cv2.resize(frame_rgb_right, (self.frame_width, self.frame_height))

        # crop the outer edges to remove monocular zones
        cropped_left, cropped_right = frame_left[:, self.edge_crop :], frame_right[:, : self.edge_crop]
        return cropped_left, cropped_right

    def _rectify_frames(self, frame_left, frame_right):
        """Apply calibration rectification to stereo frames."""
        # Apply rectification mapping using precomputed maps
        rect_left = cv2.remap(frame_left, self.map_left[0], self.map_left[1], cv2.INTER_LINEAR)
        rect_right = cv2.remap(frame_right, self.map_right[0], self.map_right[1], cv2.INTER_LINEAR)
        return rect_left, rect_right

    def _load_calibration_data(self, calibration_file: str):
        """Load calibration data from file."""
        self.logger.info(f"Loading calibration data from {calibration_file}...")

        try:
            if calibration_file.endswith(".pkl"):
                with open(calibration_file, "rb") as f:
                    calib_data = pickle.load(f)
            else:
                raise ValueError("Calibration file must be .pkl")

            # These are the key rectification maps for fast processing
            self.map_left = calib_data["map_left"]
            self.map_right = calib_data["map_right"]
            self.frame_width = calib_data["frame_width"]
            self.frame_height = calib_data["frame_height"]

            self.logger.info("✓ Calibration data loaded successfully")
            self.logger.info(f"  Frame size: {self.frame_width}x{self.frame_height}")

        except Exception as e:
            self.logger.error(f"Error loading calibration data: {e}")
            raise RuntimeError(f"Failed to load calibration data: {e}")

    def stop_camera(self) -> None:
        """Releases the capture and stops the camera."""
        if self.capture is not None:
            self.capture.release()
            self.capture = None
            self.logger.info(f"stereo camera at index {self.cam_index} stopped")
