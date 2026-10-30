"""Camera backends: Picamera2 (CSI camera module) and OpenCV (USB/V4L2)."""

from __future__ import annotations

import logging
import threading
import time
from typing import Optional

import numpy as np

from robot.hardware.base import Camera, DeviceHealth, HardwareUnavailable, HealthState

log = logging.getLogger(__name__)


def frame_quality(frame: Optional[np.ndarray]) -> tuple[bool, str]:
    """Reject frames that are empty, almost black, saturated or frozen-looking."""
    if frame is None or frame.size == 0:
        return False, "empty frame"
    if frame.ndim != 3 or frame.shape[2] != 3:
        return False, "unexpected frame shape"
    mean = float(frame.mean())
    if mean < 8:
        return False, "frame too dark (lens covered?)"
    if mean > 250:
        return False, "frame saturated"
    return True, ""


class _ThreadedCamera(Camera):
    """Grabs frames on a background thread so readers always get the newest."""

    def __init__(self, name: str):
        super().__init__(name)
        self._frame: Optional[np.ndarray] = None
        self._stamp = 0.0
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self.fps_measured = 0.0
        self.last_quality = ""

    def _grab(self) -> Optional[np.ndarray]:
        raise NotImplementedError

    def _loop(self) -> None:
        last = time.time()
        frames = 0
        while self._running:
            try:
                frame = self._grab()
            except Exception as exc:
                self._record_error(exc)
                time.sleep(0.1)
                continue
            if frame is None:
                self._record_error("no frame")
                time.sleep(0.05)
                continue
            now = time.time()
            with self._lock:
                self._frame, self._stamp = frame, now
            frames += 1
            if now - last >= 1.0:
                self.fps_measured = frames / (now - last)
                frames, last = 0, now
            self._record_ok()

    def _start(self) -> None:
        self._running = True
        self._thread = threading.Thread(target=self._loop, name=f"cam-{self.name}", daemon=True)
        self._thread.start()

    def read(self):
        with self._lock:
            frame, stamp = self._frame, self._stamp
        if frame is None:
            return False, None, 0.0
        ok, why = frame_quality(frame)
        self.last_quality = why
        return ok, frame, stamp

    def health(self) -> DeviceHealth:
        h = super().health()
        if h.state == HealthState.OK and time.time() - self._stamp > 2.0:
            h.state, h.message = HealthState.FAULT, "no frames for 2 s"
        elif h.state == HealthState.OK and self.last_quality:
            h.state, h.message = HealthState.DEGRADED, self.last_quality
        return h

    def shutdown(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        super().shutdown()


class Picamera2Camera(_ThreadedCamera):
    def __init__(self, name: str, width: int, height: int, fps: int):
        super().__init__(name)
        self.size = (width, height)
        self.fps = fps
        self._cam = None

    def initialize(self) -> None:
        try:
            from picamera2 import Picamera2
        except ImportError as exc:
            raise HardwareUnavailable("picamera2 not installed (apt install python3-picamera2)") from exc
        try:
            if not Picamera2.global_camera_info():
                raise HardwareUnavailable("no CSI camera detected")
            self._cam = Picamera2()
            cfg = self._cam.create_video_configuration(
                main={"size": self.size, "format": "RGB888"},
                controls={"FrameRate": self.fps})
            self._cam.configure(cfg)
            self._cam.start()
        except HardwareUnavailable:
            raise
        except Exception as exc:
            self._record_error(exc)
            raise HardwareUnavailable(f"picamera2: {exc}") from exc
        super().initialize()
        self._start()

    def _grab(self):
        # Picamera2 "RGB888" is actually BGR byte order, which is what OpenCV wants
        return self._cam.capture_array("main")

    def shutdown(self) -> None:
        super().shutdown()
        if self._cam:
            self._cam.stop()
            self._cam.close()


class OpenCVCamera(_ThreadedCamera):
    def __init__(self, name: str, device: int | str, width: int, height: int, fps: int):
        super().__init__(name)
        self.device = device
        self.size = (width, height)
        self.fps = fps
        self._cap = None

    def initialize(self) -> None:
        import cv2
        cap = cv2.VideoCapture(self.device)
        if not cap.isOpened():
            raise HardwareUnavailable(f"cannot open camera {self.device}")
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.size[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.size[1])
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        ok, _ = cap.read()
        if not ok:
            cap.release()
            raise HardwareUnavailable(f"camera {self.device} opened but returned no frame")
        self._cap = cap
        super().initialize()
        self._start()

    def _grab(self):
        ok, frame = self._cap.read()
        return frame if ok else None

    def shutdown(self) -> None:
        super().shutdown()
        if self._cap:
            self._cap.release()
