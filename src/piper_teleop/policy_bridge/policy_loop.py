"""Main policy execution loop for the Piper policy bridge."""

from __future__ import annotations

import logging
import time
from collections import deque
from typing import Deque, Dict, Mapping, MutableMapping, Sequence

from openpi_client import websocket_client_policy

from .camera import CameraError, CameraStream
from .config import ServerConfig
from .robot import PiperRobot
from .visualizer import DualArmVisualizer, VisualizationUnavailable

logger = logging.getLogger(__name__)


class ActionQueue:
    """Maintains an action horizon returned by the policy server."""

    def __init__(self, max_horizon: int | None = None) -> None:
        self._buffer: Deque[Mapping[str, Mapping[str, float]]] = deque()
        self._max_horizon = max_horizon

    def push_chunk(self, chunk: Sequence[Mapping[str, Mapping[str, float]]]) -> None:
        if self._max_horizon and len(chunk) != self._max_horizon:
            logger.debug(
                "Received action chunk of length %d, expected %d",
                len(chunk),
                self._max_horizon,
            )
        self._buffer.clear()
        self._buffer.extend(chunk)

    def next(self) -> Mapping[str, Mapping[str, float]] | None:
        if not self._buffer:
            return None
        return self._buffer.popleft()


class PolicyLoop:
    """Loops observations from Piper arms through an OpenPI websocket policy."""

    def __init__(self, config: ServerConfig, robot: PiperRobot):
        self._config = config
        self._robot = robot
        self._client = websocket_client_policy.WebsocketClientPolicy(
            host=config.policy.host,
            port=config.policy.port,
            api_key=config.policy.api_key,
        )
        max_horizon = config.chunk_horizon if config.chunk_horizon > 1 else None
        self._action_queue = ActionQueue(max_horizon=max_horizon)
        self._metadata = self._client.get_server_metadata()
        logger.info("Connected to policy server; metadata: %s", self._metadata)
        self._cameras: list[CameraStream] = []
        for camera_cfg in self._config.observations.cameras:
            if not camera_cfg.enabled:
                continue
            try:
                camera_stream = CameraStream(camera_cfg)
            except CameraError as exc:
                logger.error("Failed to initialise camera %s: %s", camera_cfg.device, exc)
                continue
            self._cameras.append(camera_stream)
        if self._cameras:
            logger.info("Initialised %d camera stream(s) for observations.", len(self._cameras))
        self._visualizer: DualArmVisualizer | None = None
        if self._config.visualize:
            try:
                arms = [arm.name for arm in self._robot.arms]
                self._visualizer = DualArmVisualizer(arms)
            except VisualizationUnavailable as exc:
                logger.error("%s", exc)
                self._visualizer = None

    def _build_observation(self) -> Dict[str, object]:
        obs_cfg = self._config.observations
        observation: Dict[str, object] = {}
        if obs_cfg.include_timestamp:
            observation["timestamp"] = time.time()

        for arm in self._robot.arms:
            obs = arm.get_observation()
            joint_names = arm.joint_names
            arm_payload: Dict[str, object] = {}
            if obs_cfg.include_joint_dict:
                arm_payload["joints_dict"] = obs
            if obs_cfg.include_joint_names:
                arm_payload["joint_names"] = list(joint_names)
            if obs_cfg.include_joint_list:
                arm_payload["joints"] = [obs.get(joint, 0.0) for joint in joint_names]
            if obs_cfg.include_ee_pose:
                arm_payload["ee_pose"] = arm.get_end_effector_pose()

            observation[f"{arm.name}_arm"] = arm_payload

        for camera_stream in self._cameras:
            try:
                frame = camera_stream.read()
            except CameraError as exc:
                logger.error("Failed to read from camera %s: %s", camera_stream.key, exc)
                continue
            observation[camera_stream.key] = frame

        return observation

    def _extract_action(self, response: MutableMapping[str, object]) -> Mapping[str, Mapping[str, float]] | None:
        arm_keys = [key for key in response.keys() if key.endswith("_arm")]
        if arm_keys:
            return {key: response[key] for key in arm_keys}  # type: ignore[return-value]

        chunk = response.get("actions")
        if isinstance(chunk, list) and chunk and isinstance(chunk[0], dict):
            self._action_queue.push_chunk(chunk)  # type: ignore[arg-type]
            return self._action_queue.next()

        logger.error("Policy response did not contain per-arm actions: keys=%s", list(response.keys()))
        return None

    def run(self) -> None:
        logger.info("Starting policy loop against %s:%d", self._config.policy.host, self._config.policy.port)
        period = 1.0 / self._config.control_rate_hz if self._config.control_rate_hz > 0 else 0.0

        while True:
            loop_start = time.time()
            observation = self._build_observation()

            action_response = self._client.infer(observation)
            action_dict = self._extract_action(action_response)

            if action_dict is None:
                logger.warning("Skipping action application due to malformed policy response.")
            else:
                self._robot.send_actions(action_dict)
                if self._visualizer is not None:
                    self._visualizer.update_from_actions(action_dict)

            elapsed = time.time() - loop_start
            sleep_time = max(0.0, period - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)
            if logger.isEnabledFor(logging.DEBUG):
                logger.debug("Control loop %.1f Hz (iteration %.1f ms)", 1.0 / max(elapsed, 1e-6), elapsed * 1000)

    def close(self) -> None:
        for camera_stream in self._cameras:
            camera_stream.close()
        self._cameras.clear()
        if self._visualizer is not None:
            self._visualizer.close()
            self._visualizer = None
