import asyncio
import json
import logging
import os
import pickle
from typing import Optional

import cv2
import numpy as np
from dotenv import load_dotenv
from livekit import rtc

from tactile_teleop.livekit_auth import generate_token

# Load environment variables from the project root
load_dotenv()

LIVEKIT_URL = os.getenv("LIVEKIT_URL")

WIDTH = 640
HEIGHT = 480
DEFAULT_CAM_INDEX = int(os.getenv("CAMERA_INDEX", 6))


class CameraStreamer:
    def __init__(
        self,
        cam_indices: dict[str, int],
        calibration_file: str,
        cap_backend: int = cv2.CAP_V4L2,
        cam_name="robot0-birds-eye",
        edge_crop_pixels: int = 50,  # New parameter for edge cropping
    ):
        self.logger = logging.getLogger(__name__)

        self.is_running = False
        self._publisher_task: Optional[asyncio.Task] = None
        self.cam_loop_task: Optional[asyncio.Task] = None

        # Camera indices and backend
        self.cam_index_left = cam_indices["left"]
        self.cam_index_right = cam_indices["right"]
        self.cap_backend = cap_backend
        self.calibration_file = calibration_file
        self.cap_left = None
        self.cap_right = None

        # Edge cropping configuration
        self.edge_crop_pixels = edge_crop_pixels

        # Load calibration data
        self.load_calibration_data()

        # Calculate cropped dimensions
        self.cropped_width = self.frame_width - self.edge_crop_pixels

        # Video source - use cropped frame dimensions (side-by-side)
        self.source = rtc.VideoSource(self.cropped_width * 2, self.frame_height)
        self.track = rtc.LocalVideoTrack.create_video_track(cam_name, self.source)
        self.room: Optional[rtc.Room] = None

        # Track publish options
        self.options = rtc.TrackPublishOptions(
            source=rtc.TrackSource.SOURCE_CAMERA,
            simulcast=False,
            video_encoding=rtc.VideoEncoding(
                max_framerate=30,
                max_bitrate=3_000_000,
            ),
            video_codec=rtc.VideoCodec.H264,
        )

    def load_calibration_data(self):
        """Load calibration data from file."""
        self.logger.info(f"Loading calibration data from {self.calibration_file}...")

        try:
            if self.calibration_file.endswith(".pkl"):
                with open(self.calibration_file, "rb") as f:
                    calib_data = pickle.load(f)
            elif self.calibration_file.endswith(".json"):
                with open(self.calibration_file, "r") as f:
                    json_data = json.load(f)
                # Convert lists back to numpy arrays
                calib_data = {}
                for key, value in json_data.items():
                    if isinstance(value, list) and key not in ["checkerboard_size"]:
                        calib_data[key] = np.array(value)
                    else:
                        calib_data[key] = value
            else:
                raise ValueError("Calibration file must be .pkl or .json")

            # Extract calibration parameters
            self.camera_matrix_left = calib_data["camera_matrix_left"]
            self.dist_coeffs_left = calib_data["dist_coeffs_left"]
            self.camera_matrix_right = calib_data["camera_matrix_right"]
            self.dist_coeffs_right = calib_data["dist_coeffs_right"]

            self.rotation_matrix = calib_data["rotation_matrix"]
            self.translation_vector = calib_data["translation_vector"]

            self.rect_left = calib_data["rect_left"]
            self.rect_right = calib_data["rect_right"]
            self.proj_left_scaled = calib_data["proj_left_scaled"]
            self.proj_right_scaled = calib_data["proj_right_scaled"]
            self.Q = calib_data["Q"]

            # These are the key rectification maps for fast processing
            self.map_left = calib_data["map_left"]
            self.map_right = calib_data["map_right"]

            self.frame_width = calib_data["frame_width"]
            self.frame_height = calib_data["frame_height"]
            self.physical_baseline_mm = calib_data["physical_baseline_mm"]
            self.target_ipd_mm = calib_data["target_ipd_mm"]
            self.baseline_scale_factor = calib_data["baseline_scale_factor"]

            self.logger.info("✓ Calibration data loaded successfully")
            self.logger.info(f"  Frame size: {self.frame_width}x{self.frame_height}")
            baseline_info = f"  Baseline scaling: {self.physical_baseline_mm}mm → " f"{self.target_ipd_mm}mm"
            self.logger.info(baseline_info)

        except Exception as e:
            self.logger.error(f"Error loading calibration data: {e}")
            raise RuntimeError(f"Failed to load calibration data: {e}")

    def _init_camera(self):
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

        self.logger.info(f"Cameras initialized at {self.frame_width}x{self.frame_height}")
        self.logger.info(f"Edge cropping enabled: {self.edge_crop_pixels} pixels from " f"outer edges")

    def rectify_frames(self, frame_left, frame_right):
        """Apply calibration rectification to stereo frames."""
        # Apply rectification mapping using precomputed maps
        rect_left = cv2.remap(frame_left, self.map_left[0], self.map_left[1], cv2.INTER_LINEAR)
        rect_right = cv2.remap(frame_right, self.map_right[0], self.map_right[1], cv2.INTER_LINEAR)
        return rect_left, rect_right

    def crop_stereo_edges(self, frame_left, frame_right):
        """
        Crop the outer edges of stereo frames to remove monocular zones.
        Removes left edge of left frame and right edge of right frame.
        """
        # Crop left edge from left frame (remove leftmost pixels)
        cropped_left = frame_left[:, self.edge_crop_pixels :]

        # Crop right edge from right frame (remove rightmost pixels)
        cropped_right = frame_right[:, : -self.edge_crop_pixels]

        return cropped_left, cropped_right

    def _start_camera_loop(self):
        self.is_running = True
        self.cam_loop_task = asyncio.create_task(self._camera_loop())
        camera_info = f"Started camera capture loop for cameras " f"{self.cam_index_left} and {self.cam_index_right}"
        self.logger.info(camera_info)

    async def _stop_camera_loop(self):
        self.is_running = False
        if self.cam_loop_task:
            self.cam_loop_task.cancel()
            try:
                await self.cam_loop_task  # wait for the loop to finish
            except asyncio.CancelledError:
                pass
        if self.cap_left:
            self.cap_left.release()
        if self.cap_right:
            self.cap_right.release()
        self.logger.info("Stopped Camera Stream")

    async def _camera_loop(self):
        """Continuous loop to capture, rectify, and stream camera frames."""
        # Initialize cameras in the correct async context
        self._init_camera()
        debug_info = (
            f"Camera loop starting, is_running={self.is_running}, "
            f"cap_left.isOpened()={self.cap_left.isOpened()}, "
            f"cap_right.isOpened()={self.cap_right.isOpened()}"
        )
        self.logger.debug(debug_info)

        while self.is_running:
            try:
                loop = asyncio.get_event_loop()
                # Read from both cameras
                ret_left, frame_left = await loop.run_in_executor(None, self.cap_left.read)
                ret_right, frame_right = await loop.run_in_executor(None, self.cap_right.read)

                if not ret_left or not ret_right:
                    self.logger.warning("Can't receive frame (stream end?). Retrying...")
                    await asyncio.sleep(0.1)
                    continue
            except Exception as e:
                self.logger.error(f"Error reading from cameras: {e}")
                await asyncio.sleep(0.1)
                continue

            try:
                # Apply calibration rectification
                rect_left, rect_right = self.rectify_frames(frame_left, frame_right)

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

                # Crop outer edges to remove monocular zones
                cropped_left, cropped_right = self.crop_stereo_edges(frame_rgb_left, frame_rgb_right)

                # Concatenate cropped left and right frames width-wise
                concat_frame = cv2.hconcat([cropped_left, cropped_right])

                # Push concatenated frame to livekit track
                frame_bytes = concat_frame.tobytes()
                video_frame = rtc.VideoFrame(
                    self.cropped_width * 2,
                    self.frame_height,
                    rtc.VideoBufferType.RGB24,
                    frame_bytes,
                )
                self.source.capture_frame(video_frame)
            except Exception as e:
                self.logger.error(f"Error processing frame: {e}")
                continue

    async def _publish_track(self, room: rtc.Room):
        """Publish the video track to livekit room"""
        try:
            self.logger.debug(f"(CameraStreamer) Publishing track {self.track.name} to room {room.name}")
            await room.local_participant.publish_track(self.track, self.options)
            self.logger.info("(CameraStreamer) Published video track successfully")
        except Exception as e:
            self.logger.error(f"Failed to publish video track: {e}", exc_info=True)
            raise

    async def _run_camera_publisher(self, room_name: str, participant_name: str):
        self.logger.info("=== STARTING CAMERA VIDEO STREAMER ===")

        # Check environment variables
        if not LIVEKIT_URL:
            self.logger.error("LIVEKIT_URL environment variables must be set")
            return

        try:
            lk_token = generate_token(room_name=room_name, participant_identity=participant_name, canPublish=True)
            self.logger.info(f"(CameraStreamer) Token generated successfully. Length: {len(lk_token)}")
            self.logger.debug(f"(CameraStreamer) Token preview: {lk_token[:50]}...")
        except Exception as e:
            self.logger.error(f"Failed to generate token: {e}", exc_info=True)
            return

        self.room = rtc.Room()

        # Add event handlers
        @self.room.on("connected")
        def on_connected():
            self.logger.info("(CameraStreamer) ✅ Connected to LiveKit room")

        @self.room.on("disconnected")
        def on_disconnected(reason):
            self.logger.warning(f"(CameraStreamer) ❌ Room disconnected: {reason}")

        @self.room.on("connection_state_changed")
        def on_connection_state_changed(state):
            self.logger.info(f"(CameraStreamer) 🔄 Connection state changed: {state}")

        @self.room.on("participant_connected")
        def on_participant_connected(participant: rtc.RemoteParticipant):
            self.logger.info(f"(CameraStreamer) 👤 Participant connected {participant.sid}, {participant.identity}")

        @self.room.on("track_subscribed")
        def on_track_subscribed(
            track: rtc.Track, publication: rtc.RemoteTrackPublication, participant: rtc.RemoteParticipant
        ):
            self.logger.info(f"(CameraStreamer) 📺 Track subscribed: {publication.sid}")

        try:
            self.logger.info(f"(CameraStreamer) 🔌 Connecting to LiveKit, URL: {LIVEKIT_URL}")

            connection_start = asyncio.get_event_loop().time()

            await self.room.connect(LIVEKIT_URL, lk_token)

            connection_time = asyncio.get_event_loop().time() - connection_start
            self.logger.info(
                f"(CameraStreamer) ✅ Connected to LiveKit room {room_name} as {participant_name} (took {connection_time:.2f}s)"
            )
            self.logger.info(f"(CameraStreamer) 📊 Remote participants: {len(self.room.remote_participants)}")
            await self._publish_track(self.room)

            # Start camera loop for actual video streaming
            self.logger.info("📹 Starting camera loop in another thread...")
            self._start_camera_loop()

            while True:
                await asyncio.sleep(5)  # Less frequent logging
                self.logger.info(
                    f"(CameraStreamer) 💓 Connection alive - Remote participants: {len(self.room.remote_participants)}"
                )

        except KeyboardInterrupt:
            self.logger.info("(CameraStreamer) ⌨️  KeyboardInterrupt, shutting down")
        except Exception as e:
            self.logger.error(f"💥 Error in camera publisher: {e}", exc_info=True)
            # Additional debugging info
            self.logger.error(f"🔍 Exception type: {type(e).__name__}")
            if hasattr(e, "args") and e.args:
                self.logger.error(f"🔍 Exception args: {e.args}")
        finally:
            self.logger.info("(CameraStreamer) 🧹 Cleaning up...")
            await self._stop_camera_loop()
            if self.room:
                await self.room.disconnect()
            self.logger.info("(CameraStreamer) 🏁 Camera publisher shutdown complete")

    async def start(self, room_name: str, participant_name: str):
        """Start the camera streamer and wait for it to complete."""
        if self._publisher_task and not self._publisher_task.done():
            self.logger.info("Camera streamer already running")
            return

        self.logger.info("Starting camera streamer...")
        self._publisher_task = asyncio.create_task(self._run_camera_publisher(room_name, participant_name))
        self.logger.info("Camera streamer task started")

        # Wait for the task to complete (runs indefinitely)
        await self._publisher_task

    async def stop(self, timeout: float = 5.0):
        """Stop the camera streamer task."""
        if not self._publisher_task:
            return

        self.logger.info("Stopping camera streamer task...")

        # Cancel the publisher task
        self._publisher_task.cancel()

        try:
            await asyncio.wait_for(self._publisher_task, timeout=timeout)
        except asyncio.CancelledError:
            self.logger.info("Camera streamer task cancelled")
        except asyncio.TimeoutError:
            self.logger.warning(f"Camera streamer task did not stop within {timeout}s")
        except Exception as e:
            self.logger.error(f"Error stopping camera streamer task: {e}")

        self._publisher_task = None
        self.logger.info("Camera streamer stopped")
