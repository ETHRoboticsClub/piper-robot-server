from enum import Enum, auto

import cv2


class CameraType(Enum):
    MONOCULAR = auto()
    STEREO = auto()

    def __str__(self):
        return self.name.lower()


class CameraMode(Enum):
    STREAMING = "streaming"
    RECORDING = "recording"
    HYBRID = "hybrid"

    def __str__(self):
        return self.name.lower()


DEFAULT_CAMERA_TYPE = CameraType.MONOCULAR
DEFAULT_CAMERA_MODE = CameraMode.STREAMING
DEFAULT_CAMERA_FPS = 30
DEFAULT_CAMERA_FRAME_WIDTH = 640
DEFAULT_CAMERA_FRAME_HEIGHT = 480
DEFAULT_CAMERA_CAPTURE_API = cv2.CAP_V4L2
DEFAULT_CAMERA_CAM_INDEX = 0
DEFAULT_CAMERA_EDGE_CROP = 0


def type_from_str(name: str) -> CameraType:
    if name.lower() == "monocular":
        return CameraType.MONOCULAR
    elif name.lower() == "stereo":
        return CameraType.STEREO
    raise ValueError(f"Invalid camera type: {name}")


def mode_from_str(name: str) -> CameraMode:
    if name.lower() == "streaming":
        return CameraMode.STREAMING
    elif name.lower() == "recording":
        return CameraMode.RECORDING
    elif name.lower() == "hybrid":
        return CameraMode.HYBRID
    raise ValueError(f"Invalid camera mode: {name}")


class CameraConfig:
    """A configuration object for a single camera."""

    def __init__(
        self,
        name: str,
        type: CameraType = DEFAULT_CAMERA_TYPE,
        mode: CameraMode = DEFAULT_CAMERA_MODE,
        fps: int = DEFAULT_CAMERA_FPS,
        frame_width: int = DEFAULT_CAMERA_FRAME_WIDTH,
        frame_height: int = DEFAULT_CAMERA_FRAME_HEIGHT,
        capture_api: int = DEFAULT_CAMERA_CAPTURE_API,
        capture_frame_width: int = DEFAULT_CAMERA_FRAME_WIDTH,
        capture_frame_height: int = DEFAULT_CAMERA_FRAME_HEIGHT,
        cam_index: int = DEFAULT_CAMERA_CAM_INDEX,
        edge_crop: int = DEFAULT_CAMERA_EDGE_CROP,
    ):
        self.name = name
        self.type = type
        self.mode = mode
        self.fps = fps
        self.frame_width = frame_width
        self.frame_height = frame_height
        self.capture_api = capture_api
        self.capture_frame_width = capture_frame_width
        self.capture_frame_height = capture_frame_height
        self.cam_index = cam_index
        self.edge_crop = edge_crop

    def __str__(self):
        return f"CameraConfig(name={self.name}, type={self.type.value}, mode={self.mode.value}, frame_width={self.frame_width}, frame_height={self.frame_height}, capture_api={self.capture_api}), cam_indices={self.cam_index}"

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "type": self.type.__str__(),
            "mode": self.mode.__str__(),
            "fps": self.fps,
            "frame_width": self.frame_width,
            "frame_height": self.frame_height,
            "capture_frame_width": self.capture_frame_width,
            "capture_frame_height": self.capture_frame_height,
            "capture_api": self.capture_api,
            "cam_index": self.cam_index,
            "edge_crop": self.edge_crop,
        }


def from_config(config: dict[str, str]) -> list[CameraConfig]:
    configs = []
    for key, value in config.items():
        if value is None:
            continue
        type = type_from_str(value["type"]) if "type" in value else DEFAULT_CAMERA_TYPE
        mode = mode_from_str(value["mode"]) if "mode" in value else DEFAULT_CAMERA_MODE
        fps = int(value["fps"]) if "fps" in value else DEFAULT_CAMERA_FPS
        frame_width = int(value["frame_width"]) if "frame_width" in value else DEFAULT_CAMERA_FRAME_WIDTH
        frame_height = int(value["frame_height"]) if "frame_height" in value else DEFAULT_CAMERA_FRAME_HEIGHT
        capture_frame_width = (
            int(value["capture_frame_width"]) if "capture_frame_width" in value else DEFAULT_CAMERA_FRAME_WIDTH
        )
        capture_frame_height = (
            int(value["capture_frame_height"]) if "capture_frame_height" in value else DEFAULT_CAMERA_FRAME_HEIGHT
        )
        capture_api = int(value["capture_api"]) if "capture_api" in value else DEFAULT_CAMERA_CAPTURE_API
        cam_index = int(value["cam_index"]) if "cam_index" in value else DEFAULT_CAMERA_CAM_INDEX
        edge_crop = int(value["edge_crop"]) if "edge_crop" in value else DEFAULT_CAMERA_EDGE_CROP
        configs.append(
            CameraConfig(
                name=key,
                type=type,
                mode=mode,
                fps=fps,
                frame_width=frame_width,
                frame_height=frame_height,
                capture_api=capture_api,
                capture_frame_width=capture_frame_width,
                capture_frame_height=capture_frame_height,
                cam_index=cam_index,
                edge_crop=edge_crop,
            )
        )
    return configs
