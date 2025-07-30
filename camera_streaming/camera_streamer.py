import os
import logging
import asyncio
import cv2
import threading
from dotenv import load_dotenv
from livekit import rtc 

from camera_streaming.auth import generate_token

load_dotenv("development.env")

URL = os.environ.get("LIVEKIT_URL")

WIDTH = 640
HEIGHT = 480
DEFAULT_CAM_INDEX = 5


class VideoStreamer:
    def __init__(self, cam_index: int, cap_backend: int = cv2.CAP_V4L2, cam_name="robot0-birds-eye"):
        self.logger = logging.getLogger(__name__)
        
        self.cam_index = cam_index
        self.cap_backend = cap_backend
        
        self.is_running = False
        
        # Video source
        self.source = rtc.VideoSource(WIDTH, HEIGHT)
        self.track = rtc.LocalVideoTrack.create_video_track(cam_name, self.source)

        # Track publish options
        self.options = rtc.TrackPublishOptions(
            source=rtc.TrackSource.SOURCE_CAMERA,
            simulcast=False,
            video_encoding=rtc.VideoEncoding(
                max_framerate=60,
                max_bitrate=3_000_000,
            ),
            video_codec=rtc.VideoCodec.H264,
        )
        
        # Camera capture
        self.cap = cv2.VideoCapture(index=self.cam_index, apiPreference=self.cap_backend)
        
    def start_camera_loop(self):
        self.is_running = True
        self.cap_thread = threading.Thread(target=self._camera_loop) # independently running the camera loop
        self.cap_thread.daemon = True
        self.cap_thread.start()
        self.logger.info(f"Started camera capture loop for camera {self.cam_index}")
        
    def stop_camera_loop(self):
        self.is_running = False
        if self.cap_thread: 
            self.cap_thread.join() # cap worker thread -> main thread
        self.cap.release()
        self.logger.info("Stopped Camera Stream")
        
    def _camera_loop(self):
        """Continuous loop to capture the camera frames and push them to the livekit track"""
        while self.is_running: 
            ret, frame = self.cap.read() #blocking call (synchronous)
            
            if not ret:
                print("Can't receive frame (stream end?). Exiting the camera loop ...")
                break
            
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frame_resized = cv2.resize(frame_rgb, (WIDTH, HEIGHT))
            
            # push frame to livekit track
            self.source.capture_frame(frame_resized)
            
    async def publish_track(self, room: rtc.Room):
        """Publish the video track to livekit room"""
        await room.local_participant.publish_track(self.track, self.options)
        self.logger.info(f"Published video track to room {room.name}")  # Fixed: was self.r

        
async def main(participant_name: str, cam_index: int, room_name: str = 'test_room'):
    logger = logging.getLogger(__name__)
    logger.info("=== STARTING CAMERA VIDEO STREAMER ===")
    
    # Check environment variables
    if not URL:
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
        await room.connect(URL, lk_token)
        await streamer.publish_track(room)
        
        # keep running
        while True:
            await asyncio.sleep(1) # livekit room stay alive signal     
    except KeyboardInterrupt: 
        logger.info("KeyboardInterrupt, shutting down")  
    finally:
        streamer.stop_camera_loop()
        await room.disconnect()
    

if __name__ == "__main__":
    asyncio.run(main("test_participant", DEFAULT_CAM_INDEX))
        