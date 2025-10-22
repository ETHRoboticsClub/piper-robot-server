from piper_teleop.robot_server.camera import (
    Camera,
    CameraConfig,
    SharedCameraData,
    CameraMode,
    CameraType,
    StereoCamera,
    MonocularCamera,
)


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
