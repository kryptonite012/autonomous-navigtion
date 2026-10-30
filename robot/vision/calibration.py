"""Helpers for HSV threshold calibration."""

from __future__ import annotations

import cv2
import numpy as np

from robot.vision.detector import HSVRange


def suggest_range(frame: np.ndarray, roi: tuple[int, int, int, int],
                  margin: tuple[int, int, int] = (6, 50, 50)) -> list[HSVRange]:
    """Propose HSV range(s) from a region containing only the target colour.

    Uses the 5th/95th percentiles to ignore outliers. If the hue straddles
    the 0/180 wrap-around (reds) two ranges are returned.
    """
    x, y, w, h = roi
    if w <= 2 or h <= 2:
        raise ValueError("ROI too small")
    patch = frame[y:y + h, x:x + w]
    if patch.size == 0:
        raise ValueError("ROI outside frame")
    hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV).reshape(-1, 3).astype(np.int32)
    hue = hsv[:, 0]
    s_lo, s_hi = np.percentile(hsv[:, 1], [5, 95])
    v_lo, v_hi = np.percentile(hsv[:, 2], [5, 95])
    s_lo, v_lo = max(0, s_lo - margin[1]), max(0, v_lo - margin[2])
    s_hi, v_hi = min(255, s_hi + margin[1]), min(255, v_hi + margin[2])

    wraps = (np.mean(hue < 15) > 0.2) and (np.mean(hue > 165) > 0.2)
    if wraps:
        low_part = hue[hue < 90]
        high_part = hue[hue >= 90]
        lo_hi = int(min(179, np.percentile(low_part, 95) + margin[0]))
        hi_lo = int(max(0, np.percentile(high_part, 5) - margin[0]))
        return [HSVRange((0, int(s_lo), int(v_lo)), (lo_hi, int(s_hi), int(v_hi))),
                HSVRange((hi_lo, int(s_lo), int(v_lo)), (179, int(s_hi), int(v_hi)))]
    h_lo, h_hi = np.percentile(hue, [5, 95])
    return [HSVRange((int(max(0, h_lo - margin[0])), int(s_lo), int(v_lo)),
                     (int(min(179, h_hi + margin[0])), int(s_hi), int(v_hi)))]


def validate_range(lower, upper) -> HSVRange:
    lo = [int(v) for v in lower]
    hi = [int(v) for v in upper]
    if len(lo) != 3 or len(hi) != 3:
        raise ValueError("HSV bounds need 3 values")
    limits = (179, 255, 255)
    for i in range(3):
        if not (0 <= lo[i] <= limits[i] and 0 <= hi[i] <= limits[i]):
            raise ValueError(f"HSV channel {i} out of range 0..{limits[i]}")
        if lo[i] > hi[i]:
            raise ValueError(f"HSV channel {i}: lower > upper")
    return HSVRange(tuple(lo), tuple(hi))
