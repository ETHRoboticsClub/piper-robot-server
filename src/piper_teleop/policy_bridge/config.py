"""Configuration dataclasses and defaults for the Piper policy bridge."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Sequence


@dataclass
class ArmConfig:
    """Configuration for a single Piper arm."""

    name: str
    port: str
    id: str
    enabled: bool = True


@dataclass
class PolicyClientConfig:
    """Websocket client settings for the upstream OpenPI policy server."""

    host: str = "127.0.0.1"
    port: int = 8000
    api_key: str | None = None
    timeout: float = 5.0


@dataclass
class CameraConfig:
    """Configure a camera feed that should be included in the observation."""

    enabled: bool = False
    device: str | int = 0
    backend: int | None = None
    width: int = 640
    height: int = 480
    fps: int = 30
    key: str = "observation/image"
    resize_width: int | None = None
    resize_height: int | None = None
    convert_to_rgb: bool = True


@dataclass
class ObservationConfig:
    """Selects which observation components to publish to the policy."""

    include_timestamp: bool = True
    include_joint_dict: bool = True
    include_joint_list: bool = True
    include_joint_names: bool = True
    include_ee_pose: bool = True
    cameras: Sequence[CameraConfig] = field(default_factory=tuple)


@dataclass
class ServerConfig:
    """Full configuration for the policy bridge."""

    arms: Sequence[ArmConfig] = field(
        default_factory=lambda: (
            ArmConfig(name="left", port="left_piper", id="left_follower", enabled=True),
            ArmConfig(name="right", port="right_piper", id="right_follower", enabled=True),
        )
    )
    policy: PolicyClientConfig = field(default_factory=PolicyClientConfig)
    control_rate_hz: float = 20.0
    chunk_horizon: int = 1
    dry_run: bool = False
    observations: ObservationConfig = field(default_factory=ObservationConfig)
    visualize: bool = False


def camera_configs_from_teleop(camera_type: str, key_prefix: str = "observation/stereo") -> tuple[CameraConfig, ...]:
    """Build camera configs from the teleop stereo video settings."""

    from piper_teleop.config import config as teleop_config  # imported lazily to avoid cycles

    teleop_cam_cfg = teleop_config.camera_config.get(camera_type)
    if teleop_cam_cfg is None:
        raise KeyError(f"Camera type '{camera_type}' not found in teleop configuration.")

    cam_type = teleop_cam_cfg.get("type")
    if cam_type != "dual_camera_opencv":
        raise ValueError(f"Camera type '{camera_type}' is not supported for policy bridge (found '{cam_type}').")

    width = int(teleop_cam_cfg.get("frame_width", 640))
    height = int(teleop_cam_cfg.get("frame_height", 480))
    fps = int(teleop_cam_cfg.get("fps", 30))
    backend = teleop_cam_cfg.get("cap_backend")
    convert_to_rgb = bool(teleop_cam_cfg.get("convert_to_rgb", True))

    left_device = teleop_cam_cfg.get("cam_index_left")
    right_device = teleop_cam_cfg.get("cam_index_right")
    if left_device is None or right_device is None:
        raise ValueError(f"Dual camera configuration '{camera_type}' is missing camera indices.")

    left_config = CameraConfig(
        enabled=True,
        device=left_device,
        backend=backend,
        width=width,
        height=height,
        fps=fps,
        key=f"{key_prefix}/left",
        convert_to_rgb=convert_to_rgb,
    )
    right_config = CameraConfig(
        enabled=True,
        device=right_device,
        backend=backend,
        width=width,
        height=height,
        fps=fps,
        key=f"{key_prefix}/right",
        convert_to_rgb=convert_to_rgb,
    )
    return (left_config, right_config)
