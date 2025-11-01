"""Mock websocket policy server for testing the Piper policy bridge."""

from __future__ import annotations

import argparse
import asyncio
import logging
import os
from typing import Dict

import numpy as np
import websockets.asyncio.server as ws_server
import websockets.frames

try:  # pragma: no cover - optional dependency
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None  # type: ignore[assignment]

from openpi_client import msgpack_numpy

logger = logging.getLogger(__name__)


def build_action(observation: Dict, random_actions: bool) -> Dict[str, Dict[str, float]]:
    actions: Dict[str, Dict[str, float]] = {}

    for key, value in observation.items():
        if not key.endswith("_arm"):
            continue

        joints: Dict[str, float] = {}
        if isinstance(value, dict):
            if "joints_dict" in value and isinstance(value["joints_dict"], dict):
                joint_keys = value["joints_dict"].keys()
            elif "joint_names" in value and isinstance(value["joint_names"], (list, tuple)):
                joint_keys = value["joint_names"]
            else:
                logger.warning("Unable to determine joints for %s", key)
                continue
            for joint in joint_keys:
                joints[str(joint)] = float(np.random.uniform(-0.1, 0.1)) if random_actions else 0.0
        actions[key] = joints

    return actions


async def show_images(observation: Dict) -> None:
    if cv2 is None:
        return
    for key, value in observation.items():
        if not isinstance(value, np.ndarray):
            continue
        if value.ndim == 3:
            frame = value
            if frame.dtype != np.uint8:
                frame = frame.astype(np.uint8)
            cv2.imshow(key, frame[..., ::-1] if frame.shape[2] == 3 else frame)
            cv2.waitKey(1)


async def handler(websocket: ws_server.ServerConnection, random_actions: bool) -> None:
    logger.info("Client connected: %s", websocket.remote_address)
    packer = msgpack_numpy.Packer()
    await websocket.send(packer.pack({"mock": True}))

    try:
        while True:
            raw = await websocket.recv()
            observation = msgpack_numpy.unpackb(raw)
            await show_images(observation)
            actions = build_action(observation, random_actions)
            await websocket.send(packer.pack(actions))

    except websockets.ConnectionClosedOK:
        logger.info("Connection closed normally.")
    except websockets.ConnectionClosedError as exc:
        logger.warning("Connection closed with error: %s", exc)
    finally:
        if cv2 is not None:
            cv2.destroyAllWindows()


async def serve_mock() -> None:
    parser = argparse.ArgumentParser(description="Mock policy server for Piper bridge testing.")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--random", action="store_true", help="Send random actions instead of zeros.")
    parser.add_argument(
        "--log-level",
        default=os.getenv("LOG_LEVEL", "INFO"),
        choices=["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"],
    )
    args = parser.parse_args()

    logging.basicConfig(level=getattr(logging, args.log_level))

    async with ws_server.serve(
        lambda ws: handler(ws, args.random),
        args.host,
        args.port,
        compression=None,
        max_size=None,
    ):
        logger.info("Mock policy server listening on %s:%d", args.host, args.port)
        await asyncio.Future()


def main() -> None:
    try:
        asyncio.run(serve_mock())
    except KeyboardInterrupt:
        logger.info("Mock policy server stopped.")


if __name__ == "__main__":
    main()
