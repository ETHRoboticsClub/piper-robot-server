from piper_teleop.robot_server.camera.camera import Camera
from piper_teleop.robot_server.camera.camera_config import CameraConfig, CameraMode, CameraType
from piper_teleop.robot_server.camera.camera_streamer import SharedCameraData
from piper_teleop.robot_server.camera.stereo_camera import StereoCamera
from piper_teleop.robot_server.camera.monocular_camera import MonocularCamera


class CameraRecorder:

    cameras: list[Camera]

    def __init__(self, configs: list[CameraConfig], shared_data: SharedCameraData):
        self.cameras = []

        for config in configs:
            if config.mode == CameraMode.RECORDING or config.mode == CameraMode.HYBRID:
                if config.type == CameraType.STEREO:
                    self.cameras.append(StereoCamera(config))
                elif config.type == CameraType.MONOCULAR:
                    self.cameras.append(MonocularCamera(config))

        self.shared_data = shared_data
