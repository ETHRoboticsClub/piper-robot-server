# server.py
import os
import ssl
import argparse
from pathlib import Path
from datetime import timedelta
from dotenv import load_dotenv
from livekit import api
from flask import Flask, request, jsonify
from flask_cors import CORS

load_dotenv()

# Get the LiveKit credentials
LIVEKIT_API_KEY = os.getenv('LIVEKIT_API_KEY')
LIVEKIT_API_SECRET = os.getenv('LIVEKIT_API_SECRET')
LIVEKIT_URL = os.getenv('LIVEKIT_URL')

app = Flask(__name__)
CORS(app)

def generate_token(room_name: str, participant_identity=None, display_name=None, ttl_minutes=60):
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

    if not participant_identity:
       participant_identity = f"python-user-{room_name}"
       
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
        can_publish=True,
        can_subscribe=True,
    ))

    return token.to_jwt()

@app.route('/api/subscriber-token', methods=['POST'])
def get_subscriber_token():
    """API endpoint to get a LiveKit token for the A-Frame frontend"""
    try:
        data = request.get_json() or {}
        room_name = data.get('room_name')
        
        if not room_name:
            return jsonify({'error': 'room_name is required'}), 400
            
        participant_identity = data.get('participant_identity')
        ttl_minutes = data.get('ttl', 60)  # Default 1 hour
        
        token = generate_token(room_name, participant_identity, ttl_minutes=ttl_minutes)
        return jsonify({
            'token': token, 
            'room_name': room_name,
            'livekit_url': LIVEKIT_URL
        })
        
    except Exception as e:
        return jsonify({'error': str(e)}), 500

def get_ssl_context(cert_path="cert.pem", key_path="key.pem"):
    """Create SSL context using the specified certificate files."""
    try:
        # Convert to absolute paths relative to project root
        project_root = Path(__file__).parent.parent
        cert_abs_path = project_root / cert_path
        key_abs_path = project_root / key_path
        
        if not cert_abs_path.exists() or not key_abs_path.exists():
            print(f"SSL certificates not found: {cert_abs_path}, {key_abs_path}")
            return None
            
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        context.load_cert_chain(str(cert_abs_path), str(key_abs_path))
        return context
    except Exception as e:
        print(f"Error loading SSL certificates: {e}")
        return None

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='LiveKit Auth Server')
    parser.add_argument('--port', type=int, default=5050, help='Port to run the server on')
    parser.add_argument('--host', default='0.0.0.0', help='Host to bind to')
    parser.add_argument('--https', action='store_true', help='Use HTTPS')
    parser.add_argument('--cert', default='cert.pem', help='SSL certificate file')
    parser.add_argument('--key', default='key.pem', help='SSL private key file')
    
    args = parser.parse_args()
    
    if args.https:
        ssl_context = get_ssl_context(args.cert, args.key)
        if ssl_context:
            app.run(debug=True, port=args.port, host=args.host, ssl_context=ssl_context)
        else:
            print("Failed to load SSL certificates, falling back to HTTP")
            app.run(debug=True, port=args.port, host=args.host)
    else:
        app.run(debug=True, port=args.port, host=args.host) 