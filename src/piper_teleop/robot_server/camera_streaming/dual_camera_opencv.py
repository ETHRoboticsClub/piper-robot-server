import asyncio
import logging
import pickle

import cv2

from piper_teleop.robot_server.camera_streaming.base_camera import BaseCamera

logger = logging.getLogger(__name__)


class DualCameraOpenCV(BaseCamera):
    def __init__(self, camera_config: dict):
        # Camera indices and backend
        self.cam_index_left = camera_config["cam_index_left"]
        self.cam_index_right = camera_config["cam_index_right"]
        self.cap_backend = camera_config["cap_backend"]
        self.calibration_file = camera_config["calibration_file"]
        self.cap_left = None
        self.cap_right = None

        # Edge cropping configuration
        self.edge_crop_pixels = camera_config["edge_crop_pixels"]

        # Load calibration data
        self._load_calibration_data(self.calibration_file)

        # Calculate cropped dimensions
        self.cropped_width = self.frame_width - self.edge_crop_pixels

    def _rectify_frames(self, frame_left, frame_right):
        """Apply calibration rectification to stereo frames."""
        # Apply rectification mapping using precomputed maps
        rect_left = cv2.remap(frame_left, self.map_left[0], self.map_left[1], cv2.INTER_LINEAR)
        rect_right = cv2.remap(frame_right, self.map_right[0], self.map_right[1], cv2.INTER_LINEAR)
        return rect_left, rect_right

    def _load_calibration_data(self, calibration_file: str):
        """Load calibration data from file."""
        logger.info(f"Loading calibration data from {calibration_file}...")

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

            logger.info("✓ Calibration data loaded successfully")
            logger.info(f"  Frame size: {self.frame_width}x{self.frame_height}")

        except Exception as e:
            logger.error(f"Error loading calibration data: {e}")
            raise RuntimeError(f"Failed to load calibration data: {e}")

    def init_camera(self):
        """Initialize camera capture - called in the correct async context."""
        if self.cap_left is not None and self.cap_right is not None:
            return  # Already initialized

        # Left camera capture
        self.cap_left = cv2.VideoCapture(index=self.cam_index_left, apiPreference=self.cap_backend)
        self.cap_left.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap_left.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        self.cap_left.set(cv2.CAP_PROP_FPS, 30)
        self.cap_left.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not self.cap_left.isOpened():
            raise RuntimeError(f"Failed to open left camera at index {self.cam_index_left}")

        # Right camera capture
        self.cap_right = cv2.VideoCapture(index=self.cam_index_right, apiPreference=self.cap_backend)
        self.cap_right.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap_right.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        self.cap_right.set(cv2.CAP_PROP_FPS, 30)
        self.cap_right.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not self.cap_right.isOpened():
            raise RuntimeError(f"Failed to open right camera at index {self.cam_index_right}")

        logger.info(f"Cameras initialized at {self.frame_width}x{self.frame_height}")
        debug_info = (
            f"Camera loop starting, "
            f"cap_left.isOpened()={self.cap_left.isOpened()}, "
            f"cap_right.isOpened()={self.cap_right.isOpened()}"
        )
        logger.debug(debug_info)

    async def capture_frame(self):
        """Capture a frame from the left and right cameras."""
        loop = asyncio.get_event_loop()
        ret_left, frame_left = await loop.run_in_executor(None, self.cap_left.read)
        ret_right, frame_right = await loop.run_in_executor(None, self.cap_right.read)
        if not ret_left or not ret_right:
            logger.warning("Can't receive frame (stream end?). Retrying...")
            await asyncio.sleep(0.1)
            return None, None

        # rect_left, rect_right = self._rectify_frames(frame_left, frame_right)
        rect_left = frame_left
        rect_right = frame_right

        # Convert BGR to RGB
        frame_rgb_left = cv2.cvtColor(rect_left, cv2.COLOR_BGR2RGB)
        frame_rgb_right = cv2.cvtColor(rect_right, cv2.COLOR_BGR2RGB)

        # Ensure frames are correct size
        if frame_rgb_left.shape[:2] != (
            self.frame_height,
            self.frame_width,
        ):
            frame_rgb_left = cv2.resize(frame_rgb_left, (self.frame_width, self.frame_height))
        if frame_rgb_right.shape[:2] != (
            self.frame_height,
            self.frame_width,
        ):
            frame_rgb_right = cv2.resize(frame_rgb_right, (self.frame_width, self.frame_height))

        return frame_rgb_left, frame_rgb_right

    def stop_camera(self):
        if self.cap_left:
            self.cap_left.release()
        if self.cap_right:
            self.cap_right.release()
        logger.info("Stopped Camera Stream")
