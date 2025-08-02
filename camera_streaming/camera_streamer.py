import asyncio
import logging
import os
import time
from typing import Optional
import multiprocessing as mp
import cv2

from dotenv import load_dotenv
from livekit import rtc

from telegrip.livekit_auth import generate_token

# Load environment variables from the project root
load_dotenv()

LIVEKIT_URL = os.getenv("LIVEKIT_URL")

WIDTH = 640
HEIGHT = 480
DEFAULT_CAM_INDEX = int(os.getenv("CAMERA_INDEX", 6))


class CameraStreamer:
    def __init__(self, cam_index: int, cap_backend: int = cv2.CAP_V4L2, cam_name="robot0-birds-eye"):
        self.logger = logging.getLogger(__name__)
        
        self._proc: Optional[mp.Process] = None
        self.is_running = False
        self.cam_loop_task: Optional[asyncio.Task] = None

        # Camera index and backend
        self.cam_index = cam_index
        self.cap_backend = cap_backend

        # Video source
        self.source = rtc.VideoSource(WIDTH, HEIGHT)
        self.track = rtc.LocalVideoTrack.create_video_track(cam_name, self.source)

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

        # Camera capture
        self.cap = cv2.VideoCapture(index=self.cam_index, apiPreference=self.cap_backend)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # reduce buffering to 1 frame

        # LiveKit room placeholder (set when start() is called)
        self.room = None

        # Check if camera opened successfully
        if not self.cap.isOpened():
            raise RuntimeError(f"Failed to open camera at index {self.cam_index}")

    def _start_camera_loop(self):
        self.is_running = True
        self.cam_loop_task = asyncio.create_task(self._camera_loop())
        self.logger.info(f"Started camera capture loop for camera {self.cam_index}")

    async def _stop_camera_loop(self):
        self.is_running = False
        if self.cam_loop_task:
            self.cam_loop_task.cancel()
            try:
                await self.cam_loop_task # wait for the loop to finish
            except asyncio.CancelledError:
                pass
        if self.cap:
            self.cap.release()
        self.logger.info("Stopped Camera Stream")

    async def _camera_loop(self):
        """Continuous loop to capture the camera frames and push them to the livekit track"""
        while self.is_running:
            ret, frame = self.cap.read()  # blocking call (synchronous)

            if not ret:
                self.logger.warning("Can't receive frame (stream end?). Retrying...")
                time.sleep(0.1)  # Small delay before retry
                continue

            try:
                # Convert BGR to RGB and resize if needed
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                if frame_rgb.shape[:2] != (HEIGHT, WIDTH):
                    frame_rgb = cv2.resize(frame_rgb, (WIDTH, HEIGHT))

                # Push frame to livekit track
                frame_bytes = frame_rgb.tobytes()
                video_frame = rtc.VideoFrame(WIDTH, HEIGHT, rtc.VideoBufferType.RGB24, frame_bytes)
                self.source.capture_frame(video_frame)
            except Exception as e:
                self.logger.error(f"Error processing frame: {e}")
                continue

    async def _publish_track(self, room: rtc.Room):
        """Publish the video track to livekit room"""
        await room.local_participant.publish_track(self.track, self.options)
        self.logger.info(f"Published video track to room {room.name}")
        
    async def _run_camera_publisher(self, room_name: str, participant_name: str):
        self.logger.info("=== STARTING CAMERA VIDEO STREAMER ===")

        # Check environment variables
        if not LIVEKIT_URL:
            self.logger.error("LIVEKIT_URL environment variables must be set")
            return

        self.room = rtc.Room()
        lk_token = generate_token(room_name=room_name, participant_identity=participant_name)

        @self.room.on("participant_connected")
        def on_participant_connected(participant: rtc.RemoteParticipant):
            self.logger.info(f"Participant connected {participant.sid}, {participant.identity}")

        # track_subscribed is emitted whenever the local participant is subscribed to a new track
        @self.room.on("track_subscribed")
        def on_track_subscribed(
            track: rtc.Track, publication: rtc.RemoteTrackPublication, participant: rtc.RemoteParticipant
        ):
            self.logger.info("track subscribed: %s", publication.sid)

        self._start_camera_loop() # keeps sending frames to livekit

        try:
            await self.room.connect(LIVEKIT_URL, lk_token)
            await self._publish_track(self.room)

            while True:
                await asyncio.sleep(1)  # livekit room stay alive signal
        except KeyboardInterrupt:
            self.logger.info("KeyboardInterrupt, shutting down")
        finally:
            await self._stop_camera_loop()
            await self.room.disconnect()

    def _publisher_entry(self, room_name: str, participant_name: str):
        """Synchronous entry point, running camera publisher in asyncio event loop"""
        asyncio.run(self._run_camera_publisher(room_name, participant_name))

    def start(self, room_name: str, participant_name: str):
        if self._proc and self._proc.is_alive():
            self.logger.info("Camera streamer already running")
            return
        
        self._proc = mp.Process(
            target=self._publisher_entry,
            args=(room_name, participant_name),
            daemon=True,
        )
        self._proc.start()
        self.logger.info("Camera streamer started (pid=%s)", self._proc.pid)

    def stop(self, timeout: float = 5.0):
        """Terminate the camera streamer process if running."""
        if not self._proc:
            return
        self.logger.info("Stopping camera streamer (pid=%s)...", self._proc.pid)
        if self._proc.is_alive():
            self._proc.terminate()
            self._proc.join(timeout)
            if self._proc.is_alive():
                self.logger.warning("Camera streamer did not stop within %.1fs, killing", timeout)
                self._proc.kill()
                self._proc.join()
        self._proc = None