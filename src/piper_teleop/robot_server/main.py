"""
The main entry point for the teleoperation system.
"""

import argparse
import asyncio
import datetime
import logging
import multiprocessing as mp
import torch

from piper_teleop.config import config
from piper_teleop.robot_server.camera_streaming.camera_streamer import CameraStreamer
from piper_teleop.robot_server.control_loop import ControlLoop

logger = logging.getLogger(__name__)


def _camera_process_wrapper(config, room_name: str, participant_name: str, camera_config: dict, shared_img):
    """Wrapper to run camera streamer in a separate process with asyncio"""

    async def run_camera():
        camera_streamer = CameraStreamer(
            camera_config=camera_config,
            record=config.record,
            shared_img=shared_img
        )
        await camera_streamer.start(room_name, participant_name)

    asyncio.run(run_camera())


def _control_process_wrapper(config, room_name: str, participant_name: str, robot_enabled: bool, visualize: bool, shared_img):
    """Wrapper to run control process in a separate process with asyncio"""
    asyncio.run(_run_control_process(config=config,
                                     room_name=room_name, participant_name=participant_name,
                                     robot_enabled=robot_enabled, visualize=visualize,
                                     shared_img=shared_img,
                                     ))


async def _run_control_process(config, room_name: str, participant_name: str, robot_enabled: bool, visualize: bool, shared_img) -> None:
    """
    Run the controll process (receiving, processing and sending commands to the robot)
    """
    control_loop = ControlLoop(config=config, robot_enabled=robot_enabled, visualize=visualize, shared_img=shared_img)
    control_loop_task = asyncio.create_task(control_loop.run())

    await asyncio.gather(control_loop_task)

def get_shared_memory_for_recording(config):
    torch.multiprocessing.set_start_method('spawn')
    W = config.camera_config['dual_camera_opencv']['frame_width']
    H = config.camera_config['dual_camera_opencv']['frame_height']
    shared = torch.empty((2, H, W, 3), dtype=torch.uint8).share_memory_()

    return shared

async def main():
    parser = argparse.ArgumentParser(description="Robot Server - Tactile Robotics Teleoperation System")

    # Control flags
    parser.add_argument("--no-robot", action="store_true", help="Disable robot connection (visualization only)")
    parser.add_argument("--vis", action="store_true", help="Enable visualization")
    parser.add_argument("--camera-type", default="dual_camera_opencv", help="Camera config")
    parser.add_argument("--record", action="store_true", help="Enable recording")
    parser.add_argument("--resume", action="store_true", help="Resume recording")
    parser.add_argument("--repo-id", type=str, default="default-piper", help="repo_id for dataset storage")
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

    robot_enabled = not args.no_robot
    visualize = args.vis
    config.record = args.record
    config.resume = args.resume
    config.repo_id = args.repo_id
    config.root = config.root / f'{datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")}'

    logger.info("Initializing server components...")

    shared_img = get_shared_memory_for_recording(config) if config.record else None

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
                config.livekit_room,
                config.camera_streamer_participant,
                config.camera_config[args.camera_type],
                shared_img,
            ),
        )
        camera_process.start()

        # run control loop as process
        control_process = mp.Process(
            target=_control_process_wrapper,
            args=(config,
                  config.livekit_room,
                  config.controllers_processing_participant,
                  robot_enabled,
                  visualize,
                  shared_img),
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
        asyncio.run(main())
    except KeyboardInterrupt:
        logging.info("👋 tactile teleop stopped")


if __name__ == "__main__":
    main_cli()
