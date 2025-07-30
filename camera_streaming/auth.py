# server.py
import os
from livekit import api

# Get the LiveKit credentials
LIVEKIT_API_KEY = os.getenv('LIVEKIT_API_KEY')
LIVEKIT_API_SECRET = os.getenv('LIVEKIT_API_SECRET')

def generate_token(room_name: str, participant_identity=None, display_name=None):
    """
    Generate a LiveKit access token for room access.
    
    Args:
        room_name: The name of the room to join
        identity: The participant identity
        name: The display name (default: same as identity)
    
    Returns:
        JWT token string
    """

    if not participant_identity:
       participant_identity = f"python-user-{room_name}"
       
    if not display_name:
        display_name = participant_identity
        
    if not LIVEKIT_API_KEY or not LIVEKIT_API_SECRET:
        raise ValueError("LIVEKIT_API_KEY and LIVEKIT_API_SECRET must be set in the .env file")

    # Create token with video grants
    token = api.AccessToken(LIVEKIT_API_KEY, LIVEKIT_API_SECRET) \
    .with_identity(participant_identity) \
    .with_name(display_name) \
    .with_grants(api.VideoGrants(
        room_join=True,
        room=room_name,
    ))

    return token.to_jwt()
    

