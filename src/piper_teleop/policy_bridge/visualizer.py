"""Optional visualization utilities leveraging teleoperation components."""

from __future__ import annotations

import logging
from typing import Dict, Iterable, Mapping

import numpy as np

logger = logging.getLogger(__name__)


class VisualizationUnavailable(RuntimeError):
    """Raised when visualization dependencies are missing."""


def _load_visual_modules():
    try:
        from piper_teleop.config import config as teleop_config  # type: ignore
        from piper_teleop.robot_server.core.kinematics import Arm_IK  # type: ignore
        from piper_teleop.utils import get_absolute_path  # type: ignore
    except ImportError as exc:
        raise VisualizationUnavailable(
            "Visualization requires the full piper-robot-server dependencies "
            "(pinocchio, casadi, meshcat). Install them to enable --visualize."
        ) from exc

    return teleop_config, Arm_IK, get_absolute_path


class ArmMeshcatVisualizer:
    """Wraps Arm_IK Meshcat visualizer to show joint states."""

    def __init__(self, arm_name: str):
        teleop_config, Arm_IK, get_absolute_path = _load_visual_modules()
        urdf_path = get_absolute_path(teleop_config.urdf_path)
        ground_height = teleop_config.ground_height
        logger.info("Initializing Meshcat visualizer for arm %s using URDF %s", arm_name, urdf_path)
        self._ik = Arm_IK(urdf_path, ground_height)
        self._nq = self._ik.reduced_robot.model.nq
        self._arm_name = arm_name

    def update(self, joint_positions: Iterable[float]) -> None:
        q = np.array(list(joint_positions), dtype=np.float64)
        if q.size < self._nq:
            logger.debug(
                "Arm %s visualization received %d joints; expected %d. Padding with zeros.",
                self._arm_name,
                q.size,
                self._nq,
            )
            q = np.pad(q, (0, self._nq - q.size))
        elif q.size > self._nq:
            q = q[: self._nq]

        self._ik.vis.display(q)

    def close(self) -> None:
        try:
            self._ik.vis.viewer.delete()  # type: ignore[attr-defined]
        except Exception:  # pragma: no cover - best effort cleanup
            pass


class DualArmVisualizer:
    """Convenience wrapper to update both arms."""

    def __init__(self, arms: Iterable[str]):
        self._visualizers: Dict[str, ArmMeshcatVisualizer] = {
            arm: ArmMeshcatVisualizer(arm) for arm in arms
        }

    def update_from_actions(self, action_dict: Mapping[str, Mapping[str, float]]) -> None:
        for arm_key, vis in self._visualizers.items():
            arm_action = action_dict.get(f"{arm_key}_arm") or action_dict.get(arm_key)
            if not arm_action:
                continue
            joint_values = [
                float(arm_action.get(f"joint_{idx}.pos", 0.0)) for idx in range(6)
            ]
            vis.update(joint_values)

    def close(self) -> None:
        for vis in self._visualizers.values():
            vis.close()
        self._visualizers.clear()
