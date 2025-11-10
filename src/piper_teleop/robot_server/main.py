"""
The main entry point for the teleoperation system.
"""

import argparse
import asyncio
import datetime
import logging
import multiprocessing as mp
from importlib.metadata import version

from packaging.version import Version

from piper_teleop.config import TelegripConfig, config
from piper_teleop.robot_server.camera import CameraStreamer, SharedCameraData
from piper_teleop.robot_server.camera.camera_config import CameraMode
from piper_teleop.robot_server.control_loop import ControlLoop

logger = logging.getLogger(__name__)


def _camera_process_wrapper(
    config: TelegripConfig,
    shared_data: SharedCameraData,
) -> None:
    """Wrapper to run camera streamer in a separate process with asyncio"""

    async def run_camera():
        camera_streamer = CameraStreamer(
            configs=config.camera_configs,
            shared_data=shared_data,
        )
        await camera_streamer.start(config.livekit_room, config.camera_streamer_participant)

    asyncio.run(run_camera())


def _control_process_wrapper(config: TelegripConfig, shared_data: SharedCameraData) -> None:
    """Wrapper to run control process in a separate process with asyncio"""
    asyncio.run(
        _run_control_process(
            config=config,
            shared_data=shared_data,
        )
    )


async def _run_control_process(config: TelegripConfig, shared_data: SharedCameraData) -> None:
    """
    Run the controll process (receiving, processing and sending commands to the robot)
    """
    control_loop = ControlLoop(config=config, shared_data=shared_data)
    control_loop_task = asyncio.create_task(control_loop.run())

    await asyncio.gather(control_loop_task)


def main():
    parser = argparse.ArgumentParser(description="Robot Server - Tactile Robotics Teleoperation System")

    # Control flags
    parser.add_argument("--no-robot", action="store_true", help="Disable robot connection (visualization only)")
    parser.add_argument("--vis", action="store_true", help="Enable visualization")
    parser.add_argument(
        "--keyboard", action="store_true", default=False, help="Enable keyboard control (defaults to False)"
    )
    parser.add_argument("--record", action="store_true", help="Enable recording")
    parser.add_argument("--resume", action="store_true", help="Resume recording")
    parser.add_argument("--repo-id", type=str, default="default-piper", help="repo_id for dataset storage")
    parser.add_argument("--leader", action="store_true", help="Enable Leader-Follower setup")
    parser.add_argument("--policy", action="store_true", help="Enable policy control")
    parser.add_argument(
        "--log-level",
        default="info",
        choices=["debug", "info", "warning", "error", "critical"],
        help="Set logging level",
    )
    args = parser.parse_args()

    logging.basicConfig(
        level=getattr(logging, args.log_level.upper()), format="%(asctime)s - %(message)s", datefmt="%H:%M:%S"
    )

    config.enable_robot = not args.no_robot
    config.enable_visualization = args.vis
    config.record = args.record
    config.resume = args.resume
    config.repo_id = args.repo_id
    config.enable_keyboard = args.keyboard
    config.use_leader = args.leader
    config.use_policy = args.policy
    config.root = config.root / f'{datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")}'

    if config.record:
        num_recording_cams = 0
        for cam_config in config.camera_configs:
            if cam_config.mode == CameraMode.RECORDING or cam_config.mode == CameraMode.HYBRID:
                num_recording_cams += 1
        assert num_recording_cams > 0, "There must be at least one camera in recording or hybrid mode"
        assert Version(version("lerobot")) >= Version("0.4.0"), f"lerobot >= 0.4.0 required"

    shared_data = SharedCameraData(configs=config.camera_configs)

    logger.info("Initializing server components...")
    try:
        # running background (daemon) processes
        logger.info("Starting camera streamer and control loop in parallel...")
        logger.info(f"Camera participant: {config.camera_streamer_participant}")
        logger.info(f"Controller participant: {config.controllers_processing_participant}")

        # run camera task as process
        camera_process = mp.Process(
            target=_camera_process_wrapper,
            args=(
                config,
                shared_data,
            ),
        )
        camera_process.start()

        # run control loop as process
        control_process = mp.Process(
            target=_control_process_wrapper,
            args=(
                config,
                shared_data,
            ),
        )
        control_process.start()

        # Wait for processes to complete
        camera_process.join()
        control_process.join()

    except KeyboardInterrupt:
        logger.info("Keyboard interrupt received, shutting down...")

    finally:
        logger.info("Shutting down...")

        # Terminate and wait for processes to finish
        if "camera_process" in locals() and camera_process.is_alive():
            logger.info("Terminating camera process...")
            camera_process.terminate()
            camera_process.join(timeout=5)

        if "control_process" in locals() and control_process.is_alive():
            logger.info("Terminating control process...")
            control_process.terminate()
            control_process.join(timeout=5)

        logger.info("All processes stopped")


def main_cli():
    try:
        main()
    except KeyboardInterrupt:
        logging.info("👋 tactile teleop stopped")


if __name__ == "__main__":
    main_cli()
