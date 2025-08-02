"""
The main entry point for the teleoperation system.
"""

import argparse
import asyncio
import logging

from camera_streaming.camera_streamer import CameraStreamer
from telegrip.config import config
from telegrip.control_loop import ControlLoop
from telegrip.inputs.vr_controllers import VRControllerInputProvider
from telegrip.livekit_auth import LiveKitAuthServer


logger = logging.getLogger(__name__)

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
    
    controllers_task = asyncio.create_task(controllers.start(room_name, participant_name))
    control_loop_task = asyncio.create_task(control_loop.run(command_queue))

    await asyncio.gather(controllers_task, control_loop_task)
    
async def main():
    parser = argparse.ArgumentParser(description="Robot Server - Tactile Robotics Teleoperation System")

    # Control flags
    parser.add_argument("--no-robot", action="store_true", help="Disable robot connection (visualization only)")
    parser.add_argument("--vis", action="store_true", help="Enable visualization")
    parser.add_argument("--camera-index", type=int, default=0, help="Camera index to use")
    parser.add_argument("--room-name", default="robot-vr-teleop-room", help="LiveKit room name")
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
    room_name = args.room_name
    auth_port = args.auth_port

    logger.info("Initializing server components...")
    
    # authentication ASGI server
    auth_server = LiveKitAuthServer(port=auth_port)
    
    # parallel processes
    camera_streamer = CameraStreamer(cam_index=camera_index, cam_name="robot0-birds-eye")
    
    try:
        # running background (daemon) processes
        logger.info("Starting auth server...")
        auth_server.start() 
        logger.info("Starting camera streamer...")
        camera_streamer.start(room_name, config.camera_streamer_participant)
        
        logger.info("Starting control loop...")
        await _run_control_process(room_name=room_name, participant_name=config.controller_participant, robot_enabled=robot_enabled, visualize=visualize)
        
    except KeyboardInterrupt:
        logger.info("Keyboard interrupt received, shutting down...")
    
    finally:
        logger.info("Shutting down...")
        
        # stop control loop processes (main process)
        pending = [t for t in asyncio.all_tasks() if t is not asyncio.current_task()]
        for task in pending:
            task.cancel()
        if pending:
            logger.info("Cancelling %d pending tasks", len(pending))
            await asyncio.gather(*pending, return_exceptions=True) # let's each task run through its cancellation
            
        # stop external processes
        auth_server.stop()
        camera_streamer.stop()
        logger.info("All processes stopped")
        
def main_cli():
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logging.info("👋 telegrip stopped")


if __name__ == "__main__":
    main_cli()
