import os
import logging
import asyncio
import time
import cv2
import threading
from pathlib import Path
from dotenv import load_dotenv
from livekit import rtc 

from camera_streaming.auth import generate_token

# Load environment variables from the project root
project_root = Path(__file__).parent.parent
env_file = project_root / "development.env"
load_dotenv(env_file)

LIVEKIT_URL = os.environ.get("LIVEKIT_URL")

WIDTH = 640
HEIGHT = 480
DEFAULT_CAM_INDEX = 4


class VideoStreamer:
    def __init__(self, cam_index: int, cap_backend: int = cv2.CAP_V4L2, cam_name="robot0-birds-eye"):
        self.logger = logging.getLogger(__name__)
        
        self.cam_index = cam_index
        self.cap_backend = cap_backend
        
        # Video source
        self.source = rtc.VideoSource(WIDTH, HEIGHT)
        self.track = rtc.LocalVideoTrack.create_video_track(cam_name, self.source)

        # Track publish options
        self.options = rtc.TrackPublishOptions(
            source=rtc.TrackSource.SOURCE_CAMERA,
            simulcast=False,
            video_encoding=rtc.VideoEncoding(
                max_framerate=30,
                max_bitrate=3_000_000,
            ),
            video_codec=rtc.VideoCodec.H264,
        )
        
        # Camera capture
        self.cap = cv2.VideoCapture(index=self.cam_index, apiPreference=self.cap_backend)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # reduce buffering to 1 frame
        
        self.is_running = False
        self.frame_thread = None
        
        # Check if camera opened successfully
        if not self.cap.isOpened():
            raise RuntimeError(f"Failed to open camera at index {self.cam_index}")
        
    def start_camera_loop(self):
        self.is_running = True
        self.cap_thread = threading.Thread(target=self._camera_loop) # independently running the camera loop
        self.cap_thread.daemon = True
        self.cap_thread.start()
        self.logger.info(f"Started camera capture loop for camera {self.cam_index}")
        
    def stop_camera_loop(self):
        self.is_running = False
        if self.cap_thread: 
            self.cap_thread.join(timeout=2)  # Avoid hanging indefinitely
            if self.cap_thread.is_alive():
                self.logger.warning("Camera thread did not shut down cleanly")
        self.cap.release()
        self.logger.info("Stopped Camera Stream")
        
    def _camera_loop(self):
        """Continuous loop to capture the camera frames and push them to the livekit track"""
        while self.is_running: 
            ret, frame = self.cap.read() #blocking call (synchronous)
            
            if not ret:
                self.logger.warning("Can't receive frame (stream end?). Retrying...")
                time.sleep(0.1)  # Small delay before retry
                continue
            
            try:
                # Convert BGR to RGB and resize if needed
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                if frame_rgb.shape[:2] != (HEIGHT, WIDTH):
                    frame_rgb = cv2.resize(frame_rgb, (WIDTH, HEIGHT))
                
                # Push frame to livekit track
                frame_bytes = frame_rgb.tobytes()
                video_frame = rtc.VideoFrame(WIDTH, HEIGHT, rtc.VideoBufferType.RGB24, frame_bytes)
                self.source.capture_frame(video_frame)
            except Exception as e:
                self.logger.error(f"Error processing frame: {e}")
                continue

    async def publish_track(self, room: rtc.Room):
        """Publish the video track to livekit room"""
        await room.local_participant.publish_track(self.track, self.options)
        self.logger.info(f"Published video track to room {room.name}")

        
async def main(participant_name: str, cam_index: int, room_name: str):
    logger = logging.getLogger(__name__)
    logger.info("=== STARTING CAMERA VIDEO STREAMER ===")
    
    # Check environment variables
    if not LIVEKIT_URL:
        logger.error("LIVEKIT_URL environment variables must be set")
        return
    
    room = rtc.Room()
    
    lk_token = generate_token(room_name, participant_identity=participant_name)
    
    @room.on("participant_connected")
    def on_participant_connected(participant: rtc.RemoteParticipant):
        logger.info(f"Participant connected {participant.sid}, {participant.identity}")
        
    # track_subscribed is emitted whenever the local participant is subscribed to a new track
    @room.on("track_subscribed")
    def on_track_subscribed(track: rtc.Track, publication: rtc.RemoteTrackPublication, participant: rtc.RemoteParticipant):
        logger.info("track subscribed: %s", publication.sid)
            
    streamer = VideoStreamer(cam_index=cam_index)
    streamer.start_camera_loop()
    
    try:
        await room.connect(LIVEKIT_URL, lk_token)
        await streamer.publish_track(room)
        
        # keep running
        while True:
            await asyncio.sleep(1) # livekit room stay alive signal     
    except KeyboardInterrupt: 
        logger.info("KeyboardInterrupt, shutting down")  
    finally:
        streamer.stop_camera_loop()
        await room.disconnect()