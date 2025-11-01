"""Camera helpers for streaming images in observations."""

from __future__ import annotations

import logging
from typing import Optional

from .config import CameraConfig

logger = logging.getLogger(__name__)

try:  # pragma: no cover - optional dependency guard
    import cv2  # type: ignore
except ImportError as exc:  # pragma: no cover - handled at runtime
    cv2 = None  # type: ignore
    _cv2_import_error = exc
else:  # pragma: no cover - import side effect only
    _cv2_import_error = None

try:  # pragma: no cover - optional dependency guard
    import numpy as np
except ImportError as exc:  # pragma: no cover - handled at runtime
    np = None  # type: ignore
    _np_import_error = exc
else:  # pragma: no cover
    _np_import_error = None


class CameraError(RuntimeError):
    """Raised when a camera operation fails."""


class CameraStream:
    """Simple wrapper around OpenCV VideoCapture."""

    def __init__(self, config: CameraConfig):
        if not config.enabled:
            raise ValueError("CameraStream requires an enabled CameraConfig.")
        if cv2 is None:
            raise CameraError(
                "opencv-python is required for camera streaming. "
                "Install the optional dependency or disable camera support."
            ) from _cv2_import_error
        if np is None:
            raise CameraError(
                "numpy is required for camera streaming. "
                "Install the optional dependency or disable camera support."
            ) from _np_import_error

        self._config = config
        self._capture: Optional[cv2.VideoCapture] = None  # type: ignore[name-defined]
        self._open_capture()

    @property
    def key(self) -> str:
        return self._config.key

    def _open_capture(self) -> None:
        device = self._config.device
        if self._config.backend is not None:
            capture = cv2.VideoCapture(device, self._config.backend)  # type: ignore[name-defined]
        else:
            capture = cv2.VideoCapture(device)  # type: ignore[name-defined]
        if not capture or not capture.isOpened():
            raise CameraError(f"Unable to open camera device {device!r}")

        # Try to apply resolution / fps; ignore failures but log.
        if not capture.set(cv2.CAP_PROP_FRAME_WIDTH, float(self._config.width)):  # type: ignore[name-defined]
            logger.debug("Camera %s did not accept width %d", device, self._config.width)
        if not capture.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self._config.height)):  # type: ignore[name-defined]
            logger.debug("Camera %s did not accept height %d", device, self._config.height)
        if not capture.set(cv2.CAP_PROP_FPS, float(self._config.fps)):  # type: ignore[name-defined]
            logger.debug("Camera %s did not accept fps %d", device, self._config.fps)

        self._capture = capture

    def read(self):
        if self._capture is None:
            raise CameraError("Camera capture not initialised.")
        ok, frame = self._capture.read()
        if not ok or frame is None:
            raise CameraError("Failed to read frame from camera.")

        if self._config.convert_to_rgb:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # type: ignore[name-defined]

        if self._config.resize_width and self._config.resize_height:
            frame = cv2.resize(
                frame,
                (self._config.resize_width, self._config.resize_height),
                interpolation=cv2.INTER_AREA,  # type: ignore[name-defined]
            )

        return frame.astype(np.uint8, copy=False)  # type: ignore[name-defined]

    def close(self) -> None:
        if self._capture is not None:
            self._capture.release()
            self._capture = None
