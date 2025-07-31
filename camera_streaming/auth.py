# server.py
import os
from pathlib import Path
from datetime import timedelta
from dotenv import load_dotenv
from livekit import api
from flask import Flask, request, jsonify
from flask_cors import CORS

# Load environment variables from the project root
project_root = Path(__file__).parent.parent
env_file = project_root / "development.env"
load_dotenv(env_file)

# Get the LiveKit credentials
LIVEKIT_API_KEY = os.getenv('LIVEKIT_API_KEY')
LIVEKIT_API_SECRET = os.getenv('LIVEKIT_API_SECRET')

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

@app.route('/api/token', methods=['POST'])
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
            'livekit_url': os.getenv('LIVEKIT_URL')
        })
        
    except Exception as e:
        return jsonify({'error': str(e)}), 500
    

if __name__ == '__main__':
    app.run(debug=True, port=5000) 