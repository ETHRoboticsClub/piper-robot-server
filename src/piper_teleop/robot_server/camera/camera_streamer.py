import asyncio
import logging
import os
import numpy as np

import torch
from dotenv import load_dotenv
from tactile_teleop_sdk import TactileAPI

from piper_teleop.robot_server.camera.camera import Camera
from piper_teleop.robot_server.camera.stereo_camera import StereoCamera
from piper_teleop.robot_server.camera.monocular_camera import MonocularCamera
from piper_teleop.robot_server.camera.camera_config import CameraConfig, CameraType, CameraMode

# Load environment variables from the project root
load_dotenv()


class SharedCameraData:
    """A shared memory object which is used to store the frames of the individual cameras."""

    data: dict[str, torch.Tensor]

    def __init__(self, configs: list[CameraConfig]):
        torch.multiprocessing.set_start_method("spawn")

        self.data = {}
        for config in configs:
            i = configs.index(config)
            if config.mode == CameraMode.RECORDING or config.mode == CameraMode.HYBRID:
                width = config.frame_width
                height = config.frame_height
                self.data[config.name] = torch.empty((width, height)).share_memory()

    def copy(self, name: str, new_data: torch.Tensor) -> None:
        if name not in self.data:
            raise ValueError(f"camera {name} is not in recording mode")
        self.data[name].copy_(new_data)

    def get(self, name: str) -> torch.Tensor:
        if name not in self.data:
            raise ValueError(f"camera {name} is not in recording mode")
        return self.data[name]


class CameraStreamer:
    logger: logging.Logger
    api: TactileAPI

    cameras: list[Camera]
    tasks: list[asyncio.Task]
    shared_data: SharedCameraData
    is_running: bool

    def __init__(self, configs: list[CameraConfig], shared_data: SharedCameraData):
        self.logger = logging.getLogger(__name__)
        self.api = TactileAPI(api_key=os.getenv("TACTILE_API_KEY"))
        self.cameras = []
        self.shared_data = shared_data
        self.tasks = []
        self.is_running = False

        for config in configs:
            if config.type == CameraType.STEREO:
                self.cameras.append(StereoCamera(config))
            elif config.type == CameraType.MONOCULAR:
                self.cameras.append(MonocularCamera(config))
            else:
                raise ValueError(f"unsupported camera type: {config.type.value}")

    async def _run_camera(self, camera: Camera):
        """Run a camera to capture frames and stream them to the Tactile API or save them to shared memory."""
        camera.init_camera()

        if camera.mode == CameraMode.STREAMING or camera.mode == CameraMode.HYBRID:
            await self.api.connect_camera_streamer(camera.frame_height, camera.get_cropped_width())

        while self.is_running:
            try:
                if isinstance(camera, StereoCamera):
                    left_frame, right_frame = await camera.capture_frame()
                    if left_frame is None or right_frame is None:
                        continue
                    if camera.mode == CameraMode.STREAMING or camera.mode == CameraMode.HYBRID:
                        await self.api.send_stereo_frame(left_frame, right_frame)
                    if camera.mode == CameraMode.RECORDING or camera.mode == CameraMode.HYBRID:
                        self.shared_data.copy(
                            camera.name, torch.from_numpy(np.concatenate([left_frame, right_frame], dim=1))
                        )
                elif isinstance(camera, MonocularCamera):
                    frame = await camera.capture_frame()
                    if frame is None:
                        continue
                    if camera.mode == CameraMode.STREAMING or camera.mode == CameraMode.HYBRID:
                        await self.api.send_single_frame(frame)
                    if camera.mode == CameraMode.RECORDING or camera.mode == CameraMode.HYBRID:
                        self.shared_data.copy(camera.name, torch.from_numpy(frame))

            except Exception as e:
                self.logger.error(f'error streaming camera "{camera.name}": {e}')
                await asyncio.sleep(0.1)

    async def start(self, room_name: str, participant_name: str):
        """Starts the camera streamer and waits for all camera jobs to complete."""
        if self.is_running:
            self.logger.info("Camera streamer already running")
            return
        self.logger.info("Starting camera streamer...")
        self.is_running = True

        self.tasks = [asyncio.create_task(self._run_camera(camera)) for camera in self.cameras]
        self.logger.info("Camera streamer started.")

        await asyncio.gather(*self.tasks)

    async def stop(self, timeout: float = 5.0):
        """Stops the running cameras and waits for them to complete."""
        if not self.is_running:
            return

        self.logger.info("Stopping all cameras...")
        self.is_running = False

        for task in self.tasks:
            task.cancel()

        try:
            await asyncio.wait_for(asyncio.gather(*self.tasks), timeout=timeout)
        except asyncio.CancelledError:
            self.logger.info("Camera tasks cancelled")
        except asyncio.TimeoutError:
            self.logger.warning(f"Camera tasks did not stop within {timeout}s")
        except Exception as e:
            self.logger.error(f"Error stopping camera streamer tasks: {e}")

        for camera in self.cameras:
            camera.stop_camera()

        self.tasks = []
        self.logger.info("Camera streamer stopped.")
