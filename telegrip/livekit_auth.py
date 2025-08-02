# auth.py - Authentication and token generation for LiveKit
import logging
import os
import ssl
import threading

from pathlib import Path
from datetime import timedelta

from dotenv import load_dotenv
from livekit import api
from flask import Flask, request, jsonify, Response
from flask_cors import CORS
from werkzeug.serving import make_server

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
    display_name: str | None = None
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
        raise ValueError("LIVEKIT_API_KEY and LIVEKIT_API_SECRET must be set in the .env file")

    # Create token with video grants and expiration
    token = api.AccessToken(LIVEKIT_API_KEY, LIVEKIT_API_SECRET) \
    .with_identity(participant_identity) \
    .with_name(display_name) \
    .with_ttl(timedelta(minutes=ttl_minutes)) \
    .with_grants(api.VideoGrants(
        room_join=True,
        room=room_name,
        can_publish=canPublish,
        can_subscribe=True,
    ))

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
        
        self.app = Flask(__name__)
        CORS(self.app)
        
        self._ssl = _ssl_context(Path(cert), Path(key))
        self._register_api_endpoints()
        
        if not LIVEKIT_API_KEY and LIVEKIT_API_SECRET and LIVEKIT_URL:
            raise ValueError("LIVEKIT_API_KEY, LIVEKIT_API_SECRET, and LIVEKIT_URL must be set in the .env file")
        
    def _get_token(self) -> tuple[Response, int]:
        """API endpoint to get a LiveKit token for the A-Frame frontend"""
        try:
            payload = request.get_json() or {}
            room_name = payload.get('room_name')
            participant_identity = payload.get('participant_identity')
            canPublish = payload.get('canPublish', False)
            ttl_minutes = payload.get('ttl', 60)  # Default 1 hour
            
            if not room_name: 
                return jsonify({'error': 'room_name is required'}), 400
            
            if not participant_identity:
                return jsonify({'error': 'participant_identity is required'}), 400
            
            token = generate_token(room_name, participant_identity, canPublish, ttl_minutes=ttl_minutes)
            return jsonify({
                'token': token, 
                'room_name': room_name,
                'livekit_url': LIVEKIT_URL
            }), 200
            
        except Exception as e:
            return jsonify({'error': str(e)}), 500
        
    def start(self) -> None:
        # Create a WSGI server and run it in a background thread
        self._server = make_server(
            self.host,
            self.port,
            self.app,
            ssl_context=self._ssl,
        )

        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._thread.start()
        logger.info(f"LiveKit auth server started on {self.host}:{self.port}")
        
    def stop(self, timeout: float = 3.0) -> None:
        """Stop the WSGI server and wait for its thread to exit."""
        if not getattr(self, "_server", None):
            return  # never started

        self._server.shutdown()  # tell serve_forever() to return
        if self._thread:
            self._thread.join(timeout=timeout)
            if self._thread.is_alive():
                logger.warning("Auth server thread did not terminate within %.1fs", timeout)
            else:
                logger.info("Auth server stopped")
            
    def _register_api_endpoints(self) -> None:
        """Attach Flask API endpoints to the app"""
        
        @self.app.route('/api/get-token', methods=['POST'])
        def get_token() -> tuple[Response, int]:
            return self._get_token()
        
        @self.app.route('/api/shutdown', methods=['POST'])
        def shutdown() -> tuple[Response, int]:
            self.stop()
            return jsonify({'message': 'Shutting down...'}), 200
        
__all__ = ["generate_token", "LiveKitAuthServer"]
        