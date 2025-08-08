import asyncio
import logging
import os
from typing import Optional

import cv2
from dotenv import load_dotenv
from livekit import rtc

from tactile_teleop.livekit_auth import generate_token
from tactile_teleop.robot_server.camera_streaming import DualCameraOpenCV

# Load environment variables from the project root
load_dotenv()

LIVEKIT_URL = os.getenv("LIVEKIT_URL")


class CameraStreamer:
    def __init__(
        self,
        camera_config: dict,
    ):
        self.logger = logging.getLogger(__name__)

        self.is_running = False
        self._publisher_task: Optional[asyncio.Task] = None
        self.cam_loop_task: Optional[asyncio.Task] = None

        if camera_config["type"] == "dual_camera_opencv":
            self.camera = DualCameraOpenCV(camera_config)
        else:
            raise ValueError(f"Unsupported camera type: {camera_config['type']}")

        # Video source - use cropped frame dimensions (side-by-side)
        self.source = rtc.VideoSource(self.camera.cropped_width * 2, self.camera.frame_height)
        self.track = rtc.LocalVideoTrack.create_video_track("robot0-birds-eye", self.source)
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
        # Necessary for cropping out monocular zones
        self.edge_crop_pixels = self.camera.edge_crop_pixels

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
        camera_info = f"Started camera capture loop for camera {type(self.camera)}"
        self.logger.info(camera_info)

    async def _stop_camera_loop(self):
        self.is_running = False
        if self.cam_loop_task:
            self.cam_loop_task.cancel()
            try:
                await self.cam_loop_task  # wait for the loop to finish
            except asyncio.CancelledError:
                pass
        self.camera.stop_camera()

    async def _camera_loop(self):
        """Continuous loop to capture, rectify, and stream camera frames."""
        # Initialize cameras in the correct async context
        self.camera.init_camera()

        while self.is_running:
            try:
                frame_left, frame_right = await self.camera.capture_frame()
            except Exception as e:
                self.logger.error(f"Error reading from cameras: {e}")
                await asyncio.sleep(0.1)
                continue
            try:
                # Crop outer edges to remove monocular zones
                cropped_left, cropped_right = self.crop_stereo_edges(frame_left, frame_right)

                # Concatenate cropped left and right frames width-wise
                concat_frame = cv2.hconcat([cropped_left, cropped_right])

                # Push concatenated frame to livekit track
                frame_bytes = concat_frame.tobytes()
                video_frame = rtc.VideoFrame(
                    self.camera.cropped_width * 2,
                    self.camera.frame_height,
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
