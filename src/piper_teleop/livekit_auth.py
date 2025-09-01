# livekit_auth.py - Authentication and token generation for LiveKit
import os
from datetime import timedelta

from dotenv import load_dotenv
from livekit import api

load_dotenv()

# Get the LiveKit credentials from .env file
LIVEKIT_API_KEY = os.getenv("LIVEKIT_API_KEY")
LIVEKIT_API_SECRET = os.getenv("LIVEKIT_API_SECRET")
LIVEKIT_URL = os.getenv("LIVEKIT_URL")


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
        canPublish: Whether the participant can publish
        ttl_minutes: Token expiration time in minutes
        display_name: The display name (default: same as identity)

    Returns:
        JWT token string
    """
    if not display_name:
        display_name = participant_identity

    if not LIVEKIT_API_KEY or not LIVEKIT_API_SECRET:
        raise ValueError("LIVEKIT_API_KEY and LIVEKIT_API_SECRET must be set in the .env file")

    token = (
        api.AccessToken(LIVEKIT_API_KEY, LIVEKIT_API_SECRET)
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
