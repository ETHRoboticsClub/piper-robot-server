"""
The main entry point for the teleoperation system.
"""

import argparse
import asyncio
import logging
import multiprocessing as mp

from camera_streaming.camera_streamer import CameraStreamer
from telegrip.config import config
from telegrip.control_loop import ControlLoop
from telegrip.inputs.vr_controllers import VRControllerInputProvider
from telegrip.livekit_auth import LiveKitAuthServer


logger = logging.getLogger(__name__)

def _camera_process_wrapper(cam_index: int, room_name: str, participant_name: str):
    """Wrapper to run camera streamer in a separate process with asyncio"""
    async def run_camera():
        camera_streamer = CameraStreamer(cam_index=cam_index, cam_name="robot0-birds-eye")
        await camera_streamer.start(room_name, participant_name)
    
    asyncio.run(run_camera())

def _control_process_wrapper(room_name: str, participant_name: str, robot_enabled: bool, visualize: bool):
    """Wrapper to run control process in a separate process with asyncio"""
    asyncio.run(_run_control_process(room_name, participant_name, robot_enabled, visualize))


async def _run_control_process(
    room_name: str,
    participant_name: str,
    robot_enabled: bool,
    visualize: bool
) -> None:
    """
    Run the controll process (receiving, processing and sending commands to the robot)
    """
    command_queue = asyncio.Queue()
    controllers = VRControllerInputProvider(command_queue, config)
    control_loop = ControlLoop(config, robot_enabled, visualize)
    
    controllers_task = asyncio.create_task(controllers.start(room_name=room_name, participant_name=participant_name))
    control_loop_task = asyncio.create_task(control_loop.run(command_queue))

    await asyncio.gather(controllers_task, control_loop_task)
    
async def main():
    parser = argparse.ArgumentParser(description="Robot Server - Tactile Robotics Teleoperation System")

    # Control flags
    parser.add_argument("--no-robot", action="store_true", help="Disable robot connection (visualization only)")
    parser.add_argument("--vis", action="store_true", help="Enable visualization")
    parser.add_argument("--camera-index", type=int, default=0, help="Camera index to use")
    parser.add_argument("--auth-port", type=int, default=5050, help="Auth server port")
    parser.add_argument(
        "--log-level",
        default="info",
        choices=["debug", "info", "warning", "error", "critical"],
        help="Set logging level",
    )
    args = parser.parse_args()
    
    logging.basicConfig(level=getattr(logging, args.log_level.upper()),
                        format="%(asctime)s - %(message)s", 
                        datefmt="%H:%M:%S")
    

    robot_enabled = not args.no_robot
    visualize = args.vis
    camera_index = args.camera_index if args.camera_index is not None else 0
    auth_port = args.auth_port

    logger.info("Initializing server components...")
    
    # authentication ASGI server
    auth_server = LiveKitAuthServer(port=auth_port)
    
    try:
        # running background (daemon) processes
        logger.info("Starting auth server...")
        auth_server.start() 
        
        logger.info("Starting camera streamer and control loop in parallel...")
        logger.info(f"Camera participant: {config.camera_streamer_participant}")
        logger.info(f"Controller participant: {config.controllers_processing_participant}")
        
        # run camera task as process
        camera_process = mp.Process(target=_camera_process_wrapper, args=(camera_index, config.livekit_room, config.camera_streamer_participant))
        camera_process.start()
        
        # run control loop as process
        control_process = mp.Process(target=_control_process_wrapper, args=(config.livekit_room, config.controllers_processing_participant, robot_enabled, visualize))
        control_process.start()
        
        # Wait for processes to complete
        camera_process.join()
        control_process.join()
        
    except KeyboardInterrupt:
        logger.info("Keyboard interrupt received, shutting down...")
    
    finally:
        logger.info("Shutting down...")
        
        # stop external processes
        auth_server.stop()
        
        # Terminate and wait for processes to finish
        if 'camera_process' in locals() and camera_process.is_alive():
            logger.info("Terminating camera process...")
            camera_process.terminate()
            camera_process.join(timeout=5)
            
        if 'control_process' in locals() and control_process.is_alive():
            logger.info("Terminating control process...")
            control_process.terminate() 
            control_process.join(timeout=5)
            
        logger.info("All processes stopped")
        
def main_cli():
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logging.info("👋 telegrip stopped")


if __name__ == "__main__":
    main_cli()
