"""Vision pipeline: camera -> quality check -> detection -> tracking -> publish."""

from __future__ import annotations

import logging
import threading
import time
from collections import deque
from typing import Callable, Optional

import cv2
import numpy as np

from robot.camera.cameras import frame_quality
from robot.hardware.base import Camera
from robot.vision.detector import ColorDetector, Detection, VisionParams, annotate
from robot.vision.tracker import CentroidTracker

log = logging.getLogger(__name__)


class VisionPipeline:
    def __init__(self, camera: Optional[Camera], cfg: dict,
                 on_detections: Optional[Callable[[list[Detection]], None]] = None):
        vc, cc = cfg["vision"], cfg["camera"]
        self.camera = camera
        self.params = VisionParams.from_config(vc)
        self.detector = ColorDetector(self.params, cc["focal_length_px"], cc["mount_height_m"],
                                      cc["mount_pitch_deg"], cc.get("calibrated", False))
        self.tracker = CentroidTracker(vc["track_max_distance_px"], vc["track_max_missed"])
        self.target_fps = cc.get("fps", 15)
        self.on_detections = on_detections
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self.detections: list[Detection] = []
        self.raw_jpeg: Optional[bytes] = None
        self.annotated_jpeg: Optional[bytes] = None
        self.mask_jpeg: Optional[bytes] = None
        self.last_frame: Optional[np.ndarray] = None
        self.last_frame_time = 0.0
        self.fps = 0.0
        self.process_ms = 0.0
        self.frames = 0
        self.rejected_frames = 0
        self.last_error = ""
        self.color_counts: dict[str, int] = {}
        self._seen_ids: set[int] = set()
        self._times: deque[float] = deque(maxlen=30)

    @property
    def available(self) -> bool:
        return self.camera is not None

    @property
    def running(self) -> bool:
        return self._running

    def start(self) -> None:
        if not self.available:
            raise RuntimeError("no camera available")
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, name="vision", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None

    def _loop(self) -> None:
        period = 1.0 / max(1, self.target_fps)
        while self._running:
            t0 = time.time()
            try:
                self.process_once()
            except Exception as exc:
                self.last_error = str(exc)
                log.exception("vision step failed")
            time.sleep(max(0.0, period - (time.time() - t0)))

    def process_once(self) -> list[Detection]:
        ok, frame, stamp = self.camera.read()
        if frame is None:
            self.rejected_frames += 1
            self.last_error = "no frame from camera"
            return []
        good, why = frame_quality(frame)
        if not (ok and good):
            self.rejected_frames += 1
            self.last_error = why or "camera reported bad frame"
            with self._lock:
                self.detections = []
            return []
        t0 = time.perf_counter()
        dets, masks = self.detector.detect(frame, timestamp=stamp)
        dets = self.tracker.update(dets)
        self.process_ms = (time.perf_counter() - t0) * 1000.0
        now = time.time()
        self._times.append(now)
        if len(self._times) > 1:
            self.fps = (len(self._times) - 1) / (self._times[-1] - self._times[0])
        for d in dets:
            if d.object_id not in self._seen_ids:
                self._seen_ids.add(d.object_id)
                self.color_counts[d.color] = self.color_counts.get(d.color, 0) + 1
        combined = None
        for m in masks.values():
            combined = m if combined is None else cv2.bitwise_or(combined, m)
        enc = [int(cv2.IMWRITE_JPEG_QUALITY), 70]
        with self._lock:
            self.detections = dets
            self.last_frame = frame
            self.last_frame_time = stamp
            self.frames += 1
            self.last_error = ""
            self.raw_jpeg = cv2.imencode(".jpg", frame, enc)[1].tobytes()
            self.annotated_jpeg = cv2.imencode(".jpg", annotate(frame, dets, self.fps), enc)[1].tobytes()
            if combined is not None:
                self.mask_jpeg = cv2.imencode(".jpg", cv2.bitwise_and(frame, frame, mask=combined), enc)[1].tobytes()
        if self.on_detections and dets:
            self.on_detections(dets)
        return dets

    def latest(self) -> tuple[list[Detection], float]:
        with self._lock:
            return list(self.detections), self.last_frame_time

    def frame_copy(self) -> Optional[np.ndarray]:
        with self._lock:
            return None if self.last_frame is None else self.last_frame.copy()

    def jpeg(self, kind: str = "annotated") -> Optional[bytes]:
        with self._lock:
            return {"raw": self.raw_jpeg, "annotated": self.annotated_jpeg, "mask": self.mask_jpeg}.get(kind)

    def status(self) -> dict:
        return {
            "available": self.available,
            "running": self._running,
            "fps": round(self.fps, 1),
            "process_ms": round(self.process_ms, 1),
            "frames": self.frames,
            "rejected_frames": self.rejected_frames,
            "active_tracks": len(self.detections),
            "unique_objects": dict(self.color_counts),
            "last_error": self.last_error,
            "distance_estimation": self.detector.calibrated,
            "simulated": bool(self.camera and self.camera.simulated),
        }
