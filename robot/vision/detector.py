"""HSV colour segmentation and contour based object detection."""

from __future__ import annotations

import math
import time
from dataclasses import asdict, dataclass, field
from typing import Optional

import cv2
import numpy as np


@dataclass
class HSVRange:
    lower: tuple[int, int, int]
    upper: tuple[int, int, int]


@dataclass
class Detection:
    color: str
    bbox: tuple[int, int, int, int]          # x, y, w, h in pixels
    centroid: tuple[float, float]
    area: float
    confidence: float
    timestamp: float
    object_id: Optional[int] = None          # filled in by the tracker
    distance_m: Optional[float] = None       # only when the camera is calibrated
    bearing_rad: Optional[float] = None
    solidity: float = 0.0
    circularity: float = 0.0

    def as_dict(self) -> dict:
        d = asdict(self)
        d["bbox"] = list(self.bbox)
        d["centroid"] = [round(self.centroid[0], 1), round(self.centroid[1], 1)]
        d["confidence"] = round(self.confidence, 3)
        return d


@dataclass
class VisionParams:
    blur_kernel: int = 5
    morph_kernel: int = 5
    min_area_px: int = 400
    max_area_ratio: float = 0.6
    min_solidity: float = 0.6
    min_confidence: float = 0.45
    colors: dict[str, list[HSVRange]] = field(default_factory=dict)
    enabled_colors: list[str] = field(default_factory=list)

    @classmethod
    def from_config(cls, vc: dict) -> "VisionParams":
        colors = {name: [HSVRange(tuple(r["lower"]), tuple(r["upper"])) for r in ranges]
                  for name, ranges in vc["colors"].items()}
        return cls(blur_kernel=vc["blur_kernel"], morph_kernel=vc["morph_kernel"],
                   min_area_px=vc["min_area_px"], max_area_ratio=vc["max_area_ratio"],
                   min_solidity=vc["min_solidity"], min_confidence=vc["min_confidence"],
                   colors=colors, enabled_colors=list(vc.get("enabled_colors", colors.keys())))

    def colors_as_config(self) -> dict:
        return {name: [{"lower": list(r.lower), "upper": list(r.upper)} for r in ranges]
                for name, ranges in self.colors.items()}


def _odd(k: int) -> int:
    k = max(1, int(k))
    return k if k % 2 else k + 1


class ColorDetector:
    def __init__(self, params: VisionParams, focal_px: Optional[float] = None,
                 cam_height: Optional[float] = None, pitch_deg: Optional[float] = None,
                 calibrated: bool = False):
        self.params = params
        self.focal_px = focal_px
        self.cam_height = cam_height
        self.pitch = math.radians(pitch_deg) if pitch_deg is not None else None
        self.calibrated = calibrated

    # ------------------------------------------------------------ preprocessing
    def preprocess(self, frame: np.ndarray) -> np.ndarray:
        k = _odd(self.params.blur_kernel)
        blurred = cv2.GaussianBlur(frame, (k, k), 0)
        return cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    def mask_for(self, hsv: np.ndarray, color: str) -> np.ndarray:
        mask = None
        for r in self.params.colors[color]:
            m = cv2.inRange(hsv, np.array(r.lower, np.uint8), np.array(r.upper, np.uint8))
            mask = m if mask is None else cv2.bitwise_or(mask, m)
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (_odd(self.params.morph_kernel),) * 2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)    # remove speckle
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)   # fill holes
        return mask

    # ---------------------------------------------------------------- detection
    def detect(self, frame: np.ndarray, colors: Optional[list[str]] = None,
               timestamp: Optional[float] = None) -> tuple[list[Detection], dict[str, np.ndarray]]:
        ts = timestamp or time.time()
        hsv = self.preprocess(frame)
        h, w = frame.shape[:2]
        max_area = self.params.max_area_ratio * h * w
        out: list[Detection] = []
        masks: dict[str, np.ndarray] = {}
        for color in colors or self.params.enabled_colors:
            if color not in self.params.colors:
                continue
            mask = self.mask_for(hsv, color)
            masks[color] = mask
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in contours:
                area = cv2.contourArea(c)
                if area < self.params.min_area_px or area > max_area:
                    continue
                hull_area = cv2.contourArea(cv2.convexHull(c))
                solidity = area / hull_area if hull_area > 0 else 0.0
                if solidity < self.params.min_solidity:
                    continue
                perim = cv2.arcLength(c, True)
                circularity = 4 * math.pi * area / (perim * perim) if perim > 0 else 0.0
                x, y, bw, bh = cv2.boundingRect(c)
                m = cv2.moments(c)
                cx, cy = m["m10"] / m["m00"], m["m01"] / m["m00"]
                fill = area / float(bw * bh)
                conf = self._confidence(hsv, mask, c, solidity, fill, area, x, y, bw, bh, w, h)
                if conf < self.params.min_confidence:
                    continue
                det = Detection(color, (x, y, bw, bh), (cx, cy), area, conf, ts,
                                solidity=round(solidity, 3), circularity=round(circularity, 3))
                self._estimate_range(det, w, h)
                out.append(det)
        out = self._suppress_overlaps(out)
        return out, masks

    def _confidence(self, hsv, mask, contour, solidity, fill, area, x, y, bw, bh, w, h) -> float:
        """Heuristic quality score in [0, 1].

        Combines shape regularity, mean saturation of the blob (washed out
        colours are less reliable), size and whether the blob is cut by the
        image border (partially visible objects).
        """
        roi_mask = np.zeros(mask.shape, np.uint8)
        cv2.drawContours(roi_mask, [contour], -1, 255, -1)
        sat = cv2.mean(hsv[:, :, 1], mask=roi_mask)[0] / 255.0
        size = min(1.0, area / (self.params.min_area_px * 6))
        border = 0.7 if (x <= 1 or y <= 1 or x + bw >= w - 1 or y + bh >= h - 1) else 1.0
        score = (0.35 * solidity + 0.2 * fill + 0.25 * max(sat, 0.3) + 0.2 * size) * border
        return float(max(0.0, min(1.0, score)))

    @staticmethod
    def _suppress_overlaps(dets: list[Detection]) -> list[Detection]:
        """When two colours claim the same blob (e.g. orange/red border), keep the better one."""
        dets = sorted(dets, key=lambda d: -d.confidence)
        keep: list[Detection] = []
        for d in dets:
            x, y, w, h = d.bbox
            dup = False
            for k in keep:
                kx, ky, kw, kh = k.bbox
                ix = max(0, min(x + w, kx + kw) - max(x, kx))
                iy = max(0, min(y + h, ky + kh) - max(y, ky))
                inter = ix * iy
                if inter > 0.5 * min(w * h, kw * kh):
                    dup = True
                    break
            if not dup:
                keep.append(d)
        return keep

    def _estimate_range(self, det: Detection, w: int, h: int) -> None:
        """Ground plane distance from the bottom of the bounding box.

        Only meaningful when the camera intrinsics and mount have been
        calibrated *and* the object rests on a flat floor. Otherwise left None.
        """
        if not (self.calibrated and self.focal_px and self.cam_height and self.pitch is not None):
            return
        x, y, bw, bh = det.bbox
        v = y + bh            # contact point with the floor
        u = x + bw / 2
        angle_below = -self.pitch + math.atan2(v - h / 2, self.focal_px)
        if angle_below <= math.radians(1):
            return
        dist = self.cam_height / math.tan(angle_below)
        det.distance_m = round(dist, 3)
        det.bearing_rad = round(-math.atan2(u - w / 2, self.focal_px), 4)


def annotate(frame: np.ndarray, dets: list[Detection], fps: Optional[float] = None) -> np.ndarray:
    from robot.simulation.world import OBJECT_BGR
    img = frame.copy()
    for d in dets:
        x, y, w, h = d.bbox
        color = OBJECT_BGR.get(d.color, (255, 255, 255))
        cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)
        cv2.circle(img, (int(d.centroid[0]), int(d.centroid[1])), 3, (255, 255, 255), -1)
        label = f"#{d.object_id} {d.color} {d.confidence:.2f}" if d.object_id else f"{d.color} {d.confidence:.2f}"
        if d.distance_m is not None:
            label += f" {d.distance_m:.2f}m"
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
        cv2.rectangle(img, (x, max(0, y - th - 6)), (x + tw + 4, y), color, -1)
        cv2.putText(img, label, (x + 2, y - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1, cv2.LINE_AA)
    if fps is not None:
        cv2.putText(img, f"{fps:4.1f} fps  {len(dets)} obj", (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (255, 255, 255), 1, cv2.LINE_AA)
    return img
