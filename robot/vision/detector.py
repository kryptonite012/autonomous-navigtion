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
    shape: str = "unknown"                   # circle | square | rectangle | triangle | other

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
    min_contrast: float = 20.0      # Lab distance between blob and its surroundings
    white_balance: str = "off"      # off | gray_world | reference
    wb_gains: tuple[float, float, float] = (1.0, 1.0, 1.0)   # BGR gains for "reference"
    colors: dict[str, list[HSVRange]] = field(default_factory=dict)
    enabled_colors: list[str] = field(default_factory=list)

    @classmethod
    def from_config(cls, vc: dict) -> "VisionParams":
        colors = {name: [HSVRange(tuple(r["lower"]), tuple(r["upper"])) for r in ranges]
                  for name, ranges in vc["colors"].items()}
        return cls(blur_kernel=vc["blur_kernel"], morph_kernel=vc["morph_kernel"],
                   min_area_px=vc["min_area_px"], max_area_ratio=vc["max_area_ratio"],
                   min_solidity=vc["min_solidity"], min_confidence=vc["min_confidence"],
                   min_contrast=vc.get("min_contrast", 20.0),
                   white_balance=vc.get("white_balance", "off"),
                   wb_gains=tuple(vc.get("wb_gains", (1.0, 1.0, 1.0))),
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
    @staticmethod
    def gray_world(frame: np.ndarray) -> np.ndarray:
        """Gray-world white balance: scale each channel so the scene average is grey.

        Removes the colour cast of warm (tungsten) or cool (LED/daylight)
        lighting before HSV thresholds are applied. Very dark and saturated
        pixels are excluded from the averages. It assumes the scene is mostly
        neutral (floor, walls), which holds for this robot's camera view.
        """
        f = frame.astype(np.float32)
        lum = f.mean(axis=2)
        sel = (lum > 30) & (lum < 230)
        if sel.sum() < 0.05 * lum.size:
            return frame
        means = np.array([f[:, :, i][sel].mean() for i in range(3)])
        gain = means.mean() / np.maximum(means, 1.0)
        gain = np.clip(gain, 0.5, 2.0)
        return np.clip(f * gain[None, None, :], 0, 255).astype(np.uint8)

    @staticmethod
    def reference_gains(frame: np.ndarray, roi: tuple[int, int, int, int]) -> tuple[float, float, float]:
        """BGR gains that make a white/grey reference patch neutral."""
        x, y, w, h = roi
        patch = frame[y:y + h, x:x + w].reshape(-1, 3).astype(np.float32)
        if patch.size == 0:
            raise ValueError("ROI outside frame")
        means = patch.mean(axis=0)
        if means.min() < 40:
            raise ValueError("reference patch too dark: use a white or light grey sheet")
        if means.max() > 250:
            raise ValueError("reference patch saturated: reduce exposure or use light grey")
        gains = np.clip(means.mean() / means, 0.5, 2.0)
        return tuple(round(float(g), 4) for g in gains)

    @staticmethod
    def classify_shape(contour, circularity: float) -> str:
        perim = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.04 * perim, True)
        n = len(approx)
        if circularity > 0.82 and n > 4:
            return "circle"
        if n == 3:
            return "triangle"
        if n == 4:
            (_, _), (rw, rh), _ = cv2.minAreaRect(contour)
            ratio = max(rw, rh) / max(1e-6, min(rw, rh))
            return "square" if ratio < 1.2 else "rectangle"
        if circularity > 0.75:
            return "circle"
        return "other"

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
        if self.params.white_balance == "gray_world":
            frame = self.gray_world(frame)
        elif self.params.white_balance == "reference" and self.params.wb_gains != (1.0, 1.0, 1.0):
            frame = np.clip(frame.astype(np.float32) * np.array(self.params.wb_gains, np.float32), 0, 255).astype(np.uint8)
        hsv = self.preprocess(frame)
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
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
                if self._contrast(lab, c, x, y, bw, bh) < self.params.min_contrast:
                    continue        # same colour as its surroundings: floor/wall tint, not an object
                conf = self._confidence(hsv, mask, c, solidity, fill, area, x, y, bw, bh, w, h)
                if conf < self.params.min_confidence:
                    continue
                det = Detection(color, (x, y, bw, bh), (cx, cy), area, conf, ts,
                                solidity=round(solidity, 3), circularity=round(circularity, 3),
                                shape=self.classify_shape(c, circularity))
                self._estimate_range(det, w, h)
                out.append(det)
        out = self._suppress_overlaps(out)
        return out, masks

    @staticmethod
    def _contrast(lab: np.ndarray, contour, x: int, y: int, bw: int, bh: int) -> float:
        """Lab colour distance between a blob and a ring around it."""
        pad = max(4, int(0.3 * max(bw, bh)))
        x0, y0 = max(0, x - pad), max(0, y - pad)
        x1, y1 = min(lab.shape[1], x + bw + pad), min(lab.shape[0], y + bh + pad)
        roi = lab[y0:y1, x0:x1]
        inner = np.zeros(roi.shape[:2], np.uint8)
        cv2.drawContours(inner, [contour - [x0, y0]], -1, 255, -1)
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * pad + 1, 2 * pad + 1))
        ring = cv2.subtract(cv2.dilate(inner, k), inner)
        if cv2.countNonZero(ring) < 10:
            return 255.0      # blob fills the image: nothing to compare against
        a = np.array(cv2.mean(roi, mask=inner)[:3])
        b = np.array(cv2.mean(roi, mask=ring)[:3])
        return float(np.linalg.norm(a - b))

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
