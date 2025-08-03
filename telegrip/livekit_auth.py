# auth.py - Authentication and token generation for LiveKit
import logging
import os
import ssl
import multiprocessing as mp
import uvicorn

from pathlib import Path
from datetime import timedelta

from dotenv import load_dotenv
from livekit import api
from fastapi import FastAPI
from fastapi.responses import JSONResponse
from fastapi.middleware.cors import CORSMiddleware

from telegrip.config import config

logger = logging.getLogger(__name__)
load_dotenv()

# SSL context helper
def _ssl_context(cert: Path, key: Path) -> ssl.SSLContext | None:
    if not cert.is_file() or not key.is_file():
        logger.warning("SSL cert not found (%s, %s); running HTTP only", cert, key)
        return None
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ctx.load_cert_chain(cert, key)
    return ctx
    
# Get the LiveKit credentials from .env file
LIVEKIT_API_KEY = os.getenv('LIVEKIT_API_KEY')
LIVEKIT_API_SECRET = os.getenv('LIVEKIT_API_SECRET')
LIVEKIT_URL = os.getenv('LIVEKIT_URL')

def generate_token(
    room_name: str,
    participant_identity: str,
    canPublish: bool = False,
    ttl_minutes: int = 60,
    display_name: str | None = None,
) -> str:
    """
    Generate a LiveKit access token for room access.
    
    Args:
        room_name: The name of the room to join
        participant_identity: The participant identity
        display_name: The display name (default: same as identity)
        ttl_minutes: Token expiration time in minutes
    
    Returns:
        JWT token string
    """ 
    
    if not display_name:
        display_name = participant_identity

    if not LIVEKIT_API_KEY or not LIVEKIT_API_SECRET:
        raise ValueError(
            "LIVEKIT_API_KEY and LIVEKIT_API_SECRET must be set in the .env file"
        )

    token = (api.AccessToken(LIVEKIT_API_KEY, LIVEKIT_API_SECRET)
        .with_identity(participant_identity)
        .with_name(display_name)
        .with_ttl(timedelta(minutes=ttl_minutes))
        .with_grants(
            api.VideoGrants(
                room_join=True,
                room=room_name,
                can_publish=canPublish,
                can_subscribe=True,
            )
        )
    )
    return token.to_jwt()

class LiveKitAuthServer:
    def __init__(self, 
                 port: int = 5050,
                 host: str = '0.0.0.0',
                 cert: str = 'cert.pem',
                 key: str = 'key.pem',
                 ):
        self.port = port
        self.host = host
        
        self._proc: mp.Process | None = None
        self.cert = cert
        self.key = key
        self._ssl = _ssl_context(Path(cert), Path(key))
        
        self.app = FastAPI()
        self.app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],
            allow_credentials=True,
            allow_methods=["*"],
            allow_headers=["*"],
        )
        
        self._register_api_endpoints()
        
        if not LIVEKIT_API_KEY or not LIVEKIT_API_SECRET or not LIVEKIT_URL:
            raise ValueError("LIVEKIT_API_KEY, LIVEKIT_API_SECRET, and LIVEKIT_URL must be set in the .env file")
        
    def _get_token(self, payload: dict) -> JSONResponse:
        """API endpoint to get a LiveKit token for the A-Frame frontend"""
        try:
            room_name = payload.get('room_name')
            participant_identity = payload.get('participant_identity')
            canPublish = payload.get('canPublish', False)
            ttl_minutes = payload.get('ttl', 60)  # Default 1 hour
            
            if not room_name: 
                return JSONResponse({'error': 'room_name is required'}, status_code=400)
            
            if not participant_identity:
                return JSONResponse({'error': 'participant_identity is required'}, status_code=400)
            
            token = generate_token(room_name, participant_identity, canPublish, ttl_minutes=ttl_minutes)
            return JSONResponse({
                'token': token, 
                'room_name': room_name,
                'livekit_url': LIVEKIT_URL
            }, status_code=200)
            
        except Exception as e:
            return JSONResponse({'error': str(e)}, status_code=500)
        
    def _register_api_endpoints(self) -> None:
        """Attach FastAPI API endpoints to the app"""
        
        @self.app.post('/api/get-token')
        def get_token(payload: dict) -> JSONResponse:
            return self._get_token(payload)
        
        @self.app.get('/api/livekit-config')
        def get_livekit_config() -> JSONResponse:
            return JSONResponse(config.__dict__, status_code=200)
        
        @self.app.post('/api/shutdown')
        def shutdown() -> JSONResponse:
            return self.stop()
        
    def _run_server(self, host: str, port: int, cert: str, key: str) -> None:
        """Internal entry point executed in the subprocess to run uvicorn."""
        ssl_certfile = cert if cert and Path(cert).is_file() else None
        ssl_keyfile = key if key and Path(key).is_file() else None
        uvicorn.run(
            self.app,
            host=host,
            port=port,
            ssl_certfile=ssl_certfile,
            ssl_keyfile=ssl_keyfile,
            log_level="info",
        )
        
    def start(self) -> None:
        """Launch the auth server process"""
        if self._proc and self._proc.is_alive():
            logger.info("Auth server already running on %s:%s", self.host, self.port)
            return
        
        self._proc = mp.Process(target=self._run_server, args=(self.host, self.port, self.cert, self.key), daemon=True)
        self._proc.start()
        logger.info("LiveKit auth server started on %s:%s", self.host, self.port)
        
    def stop(self) -> JSONResponse:
        """Stop the auth server process"""
        logger.info("Shutting down LiveKit auth server...")
        if not self._proc:
            return JSONResponse({'message': 'Tried to shut down auth server, but it was not running in the first place'}, status_code=200)
        
        if self._proc.is_alive():
            logger.info(f"Stopping LiveKit auth server (pid={self._proc.pid})...")
            self._proc.terminate()
            self._proc.join(timeout=3.0)
            
            if self._proc.is_alive():
                logger.warning(f"Auth server (pid={self._proc.pid}) did not stop within 3.0s; killing")
                self._proc.kill()
                self._proc.join()
                
        self._proc = None
        return JSONResponse({'message': 'Shutting down...'}, status_code=200)
            
__all__ = ["generate_token", "LiveKitAuthServer"]
        