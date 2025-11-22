import logging
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np

from .geometry import xyzrpy2transform

logger = logging.getLogger(__name__)

# Add the cloned i2rt repo to sys.path
I2RT_PATH = Path(__file__).parent / "i2rt"
if str(I2RT_PATH) not in sys.path:
    sys.path.append(str(I2RT_PATH))

DEFAULT_NUM_DOFS = 7
DEFAULT_ARM_DOFS = 6
DEFAULT_GRIPPER = "crank_4310"

HAS_I2RT = False
HAS_YAM_KINEMATICS = False

try:
    from i2rt.robots.get_robot import get_yam_robot
    from i2rt.robots.utils import GripperType

    HAS_I2RT = True
except ImportError:  # pragma: no cover - hardware dependency
    get_yam_robot = None  # type: ignore
    GripperType = None  # type: ignore
    logger.warning("Could not import i2rt. Ensure the repo and dependencies are available before using Yam.")

try:  # pragma: no cover - requires mujoco + mink
    from i2rt.robots.kinematics import Kinematics

    HAS_YAM_KINEMATICS = True
except ImportError:
    Kinematics = None  # type: ignore
    logger.debug("YAM forward kinematics unavailable (mujoco/mink missing). Falling back to zero pose transforms.")


def _zero_observation(num_dofs: int) -> Dict[str, float]:
    return {f"joint_{i}.pos": 0.0 for i in range(num_dofs)}


@dataclass
class YamConfig:
    port: str = "can0"
    id: str = "yam"
    gripper: str = DEFAULT_GRIPPER
    zero_gravity_mode: bool = True
    end_effector_site: str = "grasp_site"


class Yam:
    config_class = YamConfig
    name = "yam"

    def __init__(self, config: YamConfig):
        self.config = config
        self.robot: Optional[Any] = None
        self._is_connected = False
        self._gripper_type: Optional[Any] = None
        self._has_motorized_gripper = True
        self._num_dofs = DEFAULT_NUM_DOFS
        self._arm_dofs = DEFAULT_ARM_DOFS
        self._kinematics: Optional[Any] = None
        self._zero_obs = _zero_observation(self._num_dofs)

    # ------------------------------------------------------------------
    # Configuration helpers
    # ------------------------------------------------------------------
    def _resolve_gripper_type(self, gripper_name: str) -> Any:
        if not HAS_I2RT or GripperType is None:
            raise RuntimeError("i2rt SDK is unavailable; cannot resolve YAM gripper type.")
        if isinstance(gripper_name, GripperType):
            return gripper_name
        try:
            return GripperType.from_string_name(gripper_name)
        except ValueError:
            logger.warning("Unknown gripper '%s', defaulting to %s.", gripper_name, DEFAULT_GRIPPER)
            return GripperType.from_string_name(DEFAULT_GRIPPER)

    def _build_kinematics(self, gripper_type: Any) -> Optional[Any]:
        if not HAS_YAM_KINEMATICS or Kinematics is None:
            return None
        try:
            xml_path = gripper_type.get_xml_path()
            return Kinematics(xml_path, self.config.end_effector_site)
        except Exception as exc:  # pragma: no cover - dependent on mujoco assets
            logger.warning("Failed to initialize YAM kinematics: %s", exc)
            return None

    @property
    def _motors_ft(self) -> Dict[str, type]:
        return {f"joint_{i}.pos": float for i in range(self._num_dofs)}

    @property
    def observation_features(self) -> Dict[str, type]:
        return self._motors_ft

    @property
    def action_features(self) -> Dict[str, type]:
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        return self._is_connected and self.robot is not None

    @property
    def is_calibrated(self) -> bool:  # parity with Piper interface
        return True

    # ------------------------------------------------------------------
    # Lifecycle management
    # ------------------------------------------------------------------
    def connect(self, calibrate: bool = True) -> None:
        print("YAM connect called")  # Debug print
        if self.is_connected:
            logger.info("YAM robot %s already connected.", self.config.id)
            return

        if not HAS_I2RT or get_yam_robot is None:
            raise RuntimeError("Cannot connect to YAM robot: i2rt SDK not available.")

        gripper_type = self._resolve_gripper_type(self.config.gripper)
        logger.info(
            "Connecting to YAM robot %s on %s with gripper %s",
            self.config.id,
            self.config.port,
            gripper_type.value,
        )

        try:
            self.robot = get_yam_robot(
                channel=self.config.port,
                gripper_type=gripper_type,
               # zero_gravity_mode=self.config.zero_gravity_mode,
            )
        except Exception as exc:  # pragma: no cover - hardware specific
            self.robot = None
            logger.exception("Failed to start YAM robot: %s", exc)
            raise

        self._gripper_type = gripper_type
        self._has_motorized_gripper = gripper_type not in {
            getattr(GripperType, "YAM_TEACHING_HANDLE", None),
            getattr(GripperType, "NO_GRIPPER", None),
        }
        self._num_dofs = int(self.robot.num_dofs()) if hasattr(self.robot, "num_dofs") else DEFAULT_NUM_DOFS
        if not self._has_motorized_gripper:
            self._arm_dofs = self._num_dofs
        else:
            self._arm_dofs = max(1, self._num_dofs - 1)
        self._zero_obs = _zero_observation(self._num_dofs)
        self._kinematics = self._build_kinematics(gripper_type)
        self._is_connected = True

    def disconnect(self) -> None:
        if self.robot is None:
            self._is_connected = False
            return
        logger.info("Disconnecting YAM robot %s", self.config.id)
        try:
            if hasattr(self.robot, "close"):
                self.robot.close()
            elif hasattr(self.robot, "disconnect"):
                self.robot.disconnect()
        except Exception as exc:  # pragma: no cover - hardware specific
            logger.warning("Error while disconnecting YAM robot: %s", exc)
        finally:
            self.robot = None
            self._is_connected = False

    def calibrate(self) -> None:
        """Placeholder for API parity."""

    def configure(self) -> None:
        """Placeholder for API parity."""

    # ------------------------------------------------------------------
    # Observation helpers
    # ------------------------------------------------------------------
    def _extract_joint_positions(self, observation: Dict[str, Any]) -> np.ndarray:
        joint_pos = observation.get("joint_pos")
        if joint_pos is None:
            joint_pos = []
        joint_array = np.array(joint_pos, dtype=float).flatten()

        if "gripper_pos" in observation:
            gripper_vals = np.array(observation["gripper_pos"], dtype=float).flatten()
            joint_array = np.concatenate([joint_array, gripper_vals])

        if joint_array.size < self._num_dofs:
            joint_array = np.pad(joint_array, (0, self._num_dofs - joint_array.size))
        elif joint_array.size > self._num_dofs:
            joint_array = joint_array[: self._num_dofs]
        return joint_array

    def get_observation(self) -> Dict[str, float]:
        if not self.is_connected or self.robot is None:
            return dict(self._zero_obs)

        try:
            raw_obs = self.robot.get_observations()
            joint_array = self._extract_joint_positions(raw_obs)
            return {f"joint_{idx}.pos": float(value) for idx, value in enumerate(joint_array)}
        except Exception as exc:  # pragma: no cover - hardware specific
            logger.warning("Failed to read YAM observations: %s", exc)
            return dict(self._zero_obs)

    # ------------------------------------------------------------------
    # Control
    # ------------------------------------------------------------------
    def send_action(self, action: Dict[str, Any]) -> Dict[str, Any]:
        if not self.is_connected or self.robot is None:
            return action

        ordered_positions: List[float] = []
        for i in range(self._num_dofs):
            key = f"joint_{i}.pos"
            value = action.get(key)
            if value is None:
                raise ValueError(f"Missing required action key '{key}' for YAM send_action")
            ordered_positions.append(float(value))

        try:
            self.robot.command_joint_pos(np.array(ordered_positions, dtype=float))
            print("YAM command_joint_pos:", self.robot.get_joint_pos())  # Placeholder for hardware command
        except Exception as exc:  # pragma: no cover - hardware specific
            logger.warning("Failed to send YAM action: %s", exc)
        return action

    # ------------------------------------------------------------------
    # Cartesian pose helpers
    # ------------------------------------------------------------------
    def get_end_effector_transform(self) -> np.ndarray:
        raw_pose = self.sdk.get_end_effector_pose()
        raw_transform = xyzrpy2transform(
            raw_pose["x"], raw_pose["y"], raw_pose["z"], raw_pose["roll"], raw_pose["pitch"], raw_pose["yaw"]
        )
        link6_to_gripper_transform = xyzrpy2transform(0.0, 0.0, 0.13, 0.0, -1.57, 0.0)
        gripper_transform = raw_transform @ link6_to_gripper_transform
        return gripper_transform
