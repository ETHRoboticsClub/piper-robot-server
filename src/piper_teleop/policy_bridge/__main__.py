"""CLI entry point for the integrated Piper policy bridge."""

from __future__ import annotations

import argparse
import logging

from .config import (
    ArmConfig,
    CameraConfig,
    ObservationConfig,
    PolicyClientConfig,
    ServerConfig,
    camera_configs_from_teleop,
)
from .policy_loop import PolicyLoop
from .robot import ArmState, PiperArm, PiperRobot


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Piper policy bridge")
    parser.add_argument("--policy-host", default="127.0.0.1")
    parser.add_argument("--policy-port", type=int, default=8000)
    parser.add_argument("--api-key", default=None)
    parser.add_argument("--rate", type=float, default=20.0, help="Control loop rate (Hz)")
    parser.add_argument("--dry-run", action="store_true", help="Run without connecting to hardware")
    parser.add_argument("--log-level", default="INFO", choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    parser.add_argument("--no-timestamp", action="store_true", help="Exclude timestamp from observation payloads")
    parser.add_argument("--no-joint-dict", action="store_true", help="Exclude joint dictionaries from observations")
    parser.add_argument("--no-joint-list", action="store_true", help="Exclude joint arrays from observations")
    parser.add_argument("--no-joint-names", action="store_true", help="Exclude joint name lists from observations")
    parser.add_argument("--no-ee-pose", action="store_true", help="Exclude end-effector pose from observations")
    parser.add_argument(
        "--camera-device",
        default=None,
        help="Enable a camera stream by specifying the OpenCV device index or path (e.g. 0 or /dev/video0)",
    )
    parser.add_argument("--camera-width", type=int, default=640)
    parser.add_argument("--camera-height", type=int, default=480)
    parser.add_argument("--camera-fps", type=int, default=30)
    parser.add_argument("--camera-key", default="observation/image", help="Dictionary key for the camera frame")
    parser.add_argument("--camera-resize-width", type=int, default=None, help="Optional resize width for frames")
    parser.add_argument("--camera-resize-height", type=int, default=None, help="Optional resize height for frames")
    parser.add_argument(
        "--camera-bgr",
        action="store_true",
        help="Leave camera frames in BGR format instead of converting to RGB",
    )
    parser.add_argument(
        "--teleop-camera-type",
        default=None,
        help="Load camera settings from teleop config (e.g. dual_camera_opencv) to enable multiple cameras.",
    )
    parser.add_argument(
        "--teleop-camera-prefix",
        default="observation/camera",
        help="Dictionary key prefix for cameras loaded from teleop config.",
    )
    parser.add_argument(
        "--visualize",
        action="store_true",
        help="Launch a Meshcat visualization of the arm joint states (requires piper-robot-server dependencies).",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s - %(levelname)s - %(name)s - %(message)s",
    )

    camera_configs: list[CameraConfig] = []
    if args.teleop_camera_type:
        try:
            teleop_cameras = camera_configs_from_teleop(args.teleop_camera_type, args.teleop_camera_prefix)
        except (KeyError, ValueError) as exc:
            logging.error("Failed to load cameras from teleop config: %s", exc)
        else:
            camera_configs.extend(teleop_cameras)

    if args.camera_device is not None:
        try:
            camera_device: str | int = int(args.camera_device)
        except (TypeError, ValueError):
            camera_device = args.camera_device
        camera_configs.append(
            CameraConfig(
                enabled=True,
                device=camera_device,
                backend=None,
                width=args.camera_width,
                height=args.camera_height,
                fps=args.camera_fps,
                key=args.camera_key,
                resize_width=args.camera_resize_width,
                resize_height=args.camera_resize_height,
                convert_to_rgb=not args.camera_bgr,
            )
        )
    observation_config = ObservationConfig(
        include_timestamp=not args.no_timestamp,
        include_joint_dict=not args.no_joint_dict,
        include_joint_list=not args.no_joint_list,
        include_joint_names=not args.no_joint_names,
        include_ee_pose=not args.no_ee_pose,
        cameras=tuple(camera_configs),
    )

    config = ServerConfig(
        arms=(
            ArmConfig(name="left", port="left_piper", id="left_follower", enabled=True),
            ArmConfig(name="right", port="right_piper", id="right_follower", enabled=True),
        ),
        policy=PolicyClientConfig(host=args.policy_host, port=args.policy_port, api_key=args.api_key),
        control_rate_hz=args.rate,
        dry_run=args.dry_run,
        observations=observation_config,
        visualize=args.visualize,
    )

    arm_objects = [
        PiperArm(ArmState(name=cfg.name, port=cfg.port, enabled=cfg.enabled, dry_run=args.dry_run))
        for cfg in config.arms
        if cfg.enabled
    ]
    robot = PiperRobot(arm_objects)
    try:
        robot.connect()
    except ImportError as exc:
        logging.error("Failed to connect to Piper hardware: %s", exc)
        return
    except Exception as exc:  # pragma: no cover - safety net for hardware init
        logging.exception("Unexpected error while connecting to Piper hardware: %s", exc)
        return

    loop: PolicyLoop | None = None
    try:
        loop = PolicyLoop(config, robot)
        loop.run()
    except KeyboardInterrupt:
        logging.info("Shutting down Piper policy bridge.")
    except Exception as exc:  # pragma: no cover - safety net for runtime errors
        logging.exception("Policy loop terminated with an error: %s", exc)
    finally:
        if loop is not None:
            loop.close()
        robot.disconnect()


if __name__ == "__main__":  # pragma: no cover
    main()
