import asyncio
import logging
import os
from typing import Optional

import cv2
import numpy as np
from dotenv import load_dotenv
from livekit import rtc
from tactile_teleop import TactileAPI

from piper_teleop.livekit_auth import generate_token
from piper_teleop.robot_server.camera_streaming import DualCameraOpenCV

# Load environment variables from the project root
load_dotenv()

LIVEKIT_URL = os.getenv("LIVEKIT_URL")


class CameraStreamer:
    def __init__(
        self,
        camera_config: dict,
    ):
        self.logger = logging.getLogger(__name__)
        self.api = TactileAPI()
        self.cam_loop_task: Optional[asyncio.Task] = None

        if camera_config["type"] == "dual_camera_opencv":
            self.camera = DualCameraOpenCV(camera_config)
        else:
            raise ValueError(f"Unsupported camera type: {camera_config['type']}")

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
        await self.api.connect_camera_streamer()

        while True:
            try:
                frame_left, frame_right = await self.camera.capture_frame()
            except Exception as e:
                self.logger.error(f"Error reading from cameras: {e}")
                await asyncio.sleep(0.1)
                continue
            try:
                # Crop outer edges to remove monocular zones
                cropped_left, cropped_right = self.crop_stereo_edges(frame_left, frame_right)
                await self.api.send_stereo_frame(cv2.hconcat([cropped_left, cropped_right]))

            except Exception as e:
                self.logger.error(f"Error processing frame: {e}")
                continue

    async def start(self, room_name: str, participant_name: str):
        """Start the camera streamer and wait for it to complete."""
        if self.cam_loop_task and not self.cam_loop_task.done():
            self.logger.info("Camera streamer already running")
            return

        self.logger.info("Starting camera streamer...")
        self.is_running = True
        self.cam_loop_task = asyncio.create_task(self._camera_loop())
        self.logger.info("Camera streamer task started")

        # Wait for the task to complete (runs indefinitely)
        await self.cam_loop_task

    async def stop(self, timeout: float = 5.0):
        """Stop the camera streamer task."""
        if not self.cam_loop_task:
            return

        self.logger.info("Stopping camera streamer task...")

        # Cancel the publisher task
        self.cam_loop_task.cancel()

        try:
            await asyncio.wait_for(self.cam_loop_task, timeout=timeout)
        except asyncio.CancelledError:
            self.logger.info("Camera streamer task cancelled")
        except asyncio.TimeoutError:
            self.logger.warning(f"Camera streamer task did not stop within {timeout}s")
        except Exception as e:
            self.logger.error(f"Error stopping camera streamer task: {e}")

        self.cam_loop_task = None
        self.logger.info("Camera streamer stopped")
