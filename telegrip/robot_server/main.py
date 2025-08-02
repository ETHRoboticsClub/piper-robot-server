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

async def main():
    parser = argparse.ArgumentParser(description="Robot Server - Tactile Robotics Teleoperation System")

    # Control flags
    parser.add_argument("--no-robot", action="store_true", help="Disable robot connection (visualization only)")
    parser.add_argument("--vis", action="store_true", help="Enable visualization")
    parser.add_argument("--camera-index", type=int, help="Camera index to use")
    parser.add_argument("--room-name", default="robot-vr-teleop-room", help="LiveKit room name")
    parser.add_argument("--auth-port", type=int, default=5050, help="Auth server port")
    parser.add_argument(
        "--log-level",
        default="info",
        choices=["debug", "info", "warning", "error", "critical"],
        help="Set logging level",
    )

    args = parser.parse_args()

    robot_enabled = not args.no_robot
    visualize = args.vis
    camera_index = args.camera_index if args.camera_index is not None else 0
    room_name = args.room_name
    auth_port = args.auth_port

    vr_control_task = None
    vr_control_server = None
    control_loop_task = None
    control_loop = None
    camera_streamer_task = None
    camera_streamer = None
    auth_server = None
    
    # Configure logging
    log_level = getattr(logging, args.log_level.upper())
    logging.basicConfig(level=log_level, format="%(asctime)s - %(message)s", datefmt="%H:%M:%S")

    # Start auth server
    logger.info(f"Starting Flask auth server on https://localhost:{auth_port}...")
 
    
    try:
        logger.info("Initializing server components...")
        command_queue = asyncio.Queue()
        
        auth_server = LiveKitAuthServer(port=auth_port)
        camera_streamer = CameraStreamer(camera_index)
        vr_control_server = VRControllerInputProvider(command_queue, config)
        control_loop = ControlLoop(config, robot_enabled, visualize)

        # Starting server conponents (parallel processes)
        logger.info("Starting auth server...")
        auth_server_task = asyncio.create_task(auth_server.start())
        logger.info("Starting camera streamer...")
        camera_streamer_task = asyncio.create_task(camera_streamer.start(room_name, 'vr-teleop-viewer'))
        logger.info("Starting VR controller input provider...")
        vr_control_task = asyncio.create_task(vr_control_server.start(room_name, 'vr-teleop-viewer'))
        logger.info("Starting control loop...")
        control_loop_task = asyncio.create_task(control_loop.run(command_queue))
        
        logger.info("Starting server components...")
        await asyncio.gather(control_loop_task, vr_control_task, camera_streamer_task, auth_server_task)

    except KeyboardInterrupt:
        logging.info("\n🛑 Keyboard interrupt. Shutting down...")
    except asyncio.CancelledError:
        logging.info("\n🛑 Task cancelled. Shutting down...")
    except Exception as e:
        logging.error(f"🚨 Unexpected error: {e}")
    finally:
        try:
            tasks_to_cancel = []
            if camera_streamer_task:
                camera_streamer_task.cancel()
                tasks_to_cancel.append(camera_streamer_task)
            if vr_control_task:
                vr_control_task.cancel()
                tasks_to_cancel.append(vr_control_task)
            if control_loop_task:
                control_loop_task.cancel()
                tasks_to_cancel.append(control_loop_task)
                
            try:
                if tasks_to_cancel:
                    await asyncio.wait_for(asyncio.gather(*tasks_to_cancel, return_exceptions=True), timeout=5.0)
            except asyncio.TimeoutError:
                logger.warning("Some tasks did not complete within timeout")
            except KeyboardInterrupt:
                logger.warning("Cleanup interrupted, forcing shutdown")


            if camera_streamer:
                await camera_streamer.stop()
            if vr_control_server:
                await vr_control_server.stop()
            if control_loop:
                await control_loop.stop()
            if auth_server:
                auth_server.stop()
                
            logging.info("✅ Shutdown complete.")

        except KeyboardInterrupt:
            logger.warning("Cleanup forcibly interrupted")
        except Exception as e:
            logger.error(f"Error during cleanup: {e}")

def main_cli():
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logging.info("👋 telegrip stopped")


if __name__ == "__main__":
    main_cli()
