#!/usr/bin/env python3
"""
Simple script to run the video streaming pipeline
Usage: python run_streaming.py [--camera-index 0] [--room test_room] [--auth-port 5000]
"""
import argparse
import asyncio
import logging
import os
import subprocess
import sys
import time
from camera_streaming.video_streamer import main as streamer_main

def setup_logging():
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )

def start_auth_server(port=5000):
    """Start the Flask auth server in background"""
    cmd = [sys.executable, "-m", "flask", "--app", "camera_streaming.auth", "run", "--port", str(port)]
    env = dict(os.environ)
    env["FLASK_ENV"] = "development"
    return subprocess.Popen(cmd, env=env)

async def main():
    parser = argparse.ArgumentParser(description="Run telegrip video streaming")
    parser.add_argument("--camera-index", type=int, default=4, help="Camera index to use")
    parser.add_argument("--room", default="test_room", help="LiveKit room name")
    parser.add_argument("--participant", default="camera-streamer", help="Participant name")
    parser.add_argument("--auth-port", type=int, default=5000, help="Auth server port")
    parser.add_argument("--auth-only", action="store_true", help="Only run auth server")
    
    args = parser.parse_args()
    setup_logging()
    
    logger = logging.getLogger(__name__)
    
    # Start auth server
    logger.info(f"Starting Flask auth server on port {args.auth_port}...")
    auth_process = start_auth_server(args.auth_port)
    time.sleep(2)  # Give server time to start
    
    if args.auth_only:
        logger.info(f"Auth server running on http://localhost:{args.auth_port}")
        logger.info("Open web-ui/video-example.html in your browser")
        try:
            auth_process.wait()
        except KeyboardInterrupt:
            auth_process.terminate()
        return
    
    try:
        logger.info(f"Starting camera streamer (camera {args.camera_index}, room {args.room})")
        await streamer_main(args.participant, args.camera_index, args.room)
    except KeyboardInterrupt:
        logger.info("Shutting down...")
    finally:
        auth_process.terminate()
        auth_process.wait()

if __name__ == "__main__":
    asyncio.run(main())