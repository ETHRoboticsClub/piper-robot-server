import cv2
import asyncio
import numpy as np

from piper_teleop.robot_server.camera import Camera, CameraConfig


class MonocularCamera(Camera):
    """Monocular camera class."""

    capture: cv2.VideoCapture

    def __init__(self, config: CameraConfig):
        super().__init__(config)
        self.capture = None

    def get_cropped_width(self) -> int:
        return self.frame_width - self.edge_crop

    def is_connected(self) -> bool:
        """Returns True if the monocular camera is connected and False otherwise."""
        return self.capture is not None and self.capture.isOpened()

    def init_camera(self) -> None:
        """Initialises the monocular camera."""
        if self.capture is not None:
            return  # already initialised

        self.capture = cv2.VideoCapture(index=self.cam_index, apiPreference=self.capture_api)
        if not self.capture.isOpened():  # failed to open camera
            raise RuntimeError(f'failed to open camera "{self.name}" at index {self.cam_index}')

        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        self.capture.set(cv2.CAP_PROP_FPS, self.fps)
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.logger.info(f"monocular camera initialised at index {self.cam_index}")

    async def capture_frame(self) -> np.ndarray:
        """Captures a frame from the monocular camera."""
        loop = asyncio.get_event_loop()
        ret, frame = await loop.run_in_executor(None, self.capture.read)

        if not ret or frame is None:
            self.logger.warning("cannot receive frame (stream end?). retrying...")
            await asyncio.sleep(0.1)
            return None, None

        # convert BGR to RGB
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # ensure that the frame is correct size
        if frame_rgb.shape[:2] != (self.frame_height, self.frame_width):
            frame_rgb = cv2.resize(frame_rgb, (self.frame_width, self.frame_height))

        return frame_rgb

    def stop_camera(self) -> None:
        """Releases the capture and stops the camera."""
        if self.capture is not None:
            self.capture.release()
            self.capture = None
            self.logger.info(f"monocular camera at index {self.cam_index} stopped")
