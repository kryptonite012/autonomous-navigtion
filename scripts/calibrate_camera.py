#!/usr/bin/env python3
"""Camera intrinsics calibration with a printed chessboard.

1. Print docs/assets or any 9x6 inner-corner chessboard; measure the square size.
2. Run:  python3 scripts/calibrate_camera.py --square 0.025 --frames 20
3. Move the board around the view (tilt it, fill the corners) until enough
   frames are captured. The focal length is written to config/nexus.yaml and
   ``camera.calibrated`` is set to true.

Distance estimates are only enabled after this calibration AND after you
measured mount_height_m / mount_pitch_deg on the robot.
"""

import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from robot.config import load_config, save_section  # noqa: E402
from robot.hardware.factory import build_physical  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--cols", type=int, default=9)
    ap.add_argument("--rows", type=int, default=6)
    ap.add_argument("--square", type=float, required=True, help="square size in metres")
    ap.add_argument("--frames", type=int, default=20)
    ap.add_argument("--dry-run", action="store_true", help="print result without saving")
    args = ap.parse_args()

    cfg = load_config(overrides={"robot": {"mode": "hardware"}})
    # only the camera is needed
    cfg["hardware"] = {k: {**v, "enabled": False} for k, v in cfg["hardware"].items()}
    hw = build_physical(cfg)
    if hw.camera is None:
        print("no camera found")
        return 1

    pattern = (args.cols, args.rows)
    objp = np.zeros((args.cols * args.rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2) * args.square
    objpoints, imgpoints = [], []
    last = 0.0
    shape = None
    print("capturing... move the chessboard around the field of view")
    while len(objpoints) < args.frames:
        ok, frame, _ = hw.camera.read()
        if frame is None:
            time.sleep(0.05)
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        shape = gray.shape[::-1]
        found, corners = cv2.findChessboardCorners(gray, pattern, None)
        if found and time.time() - last > 1.0:
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                       (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
            objpoints.append(objp)
            imgpoints.append(corners)
            last = time.time()
            print(f"  frame {len(objpoints)}/{args.frames}")
        time.sleep(0.05)
    hw.shutdown()

    rms, K, dist, _, _ = cv2.calibrateCamera(objpoints, imgpoints, shape, None, None)
    fx, fy = K[0, 0], K[1, 1]
    print(f"RMS reprojection error: {rms:.3f} px")
    print(f"fx={fx:.1f} fy={fy:.1f} cx={K[0, 2]:.1f} cy={K[1, 2]:.1f}")
    print(f"distortion: {dist.ravel().round(4).tolist()}")
    if rms > 1.0:
        print("RMS above 1 px: recalibrate with more varied board poses before trusting distances")
    if not args.dry_run:
        cam = dict(cfg["camera"])
        cam["focal_length_px"] = round(float((fx + fy) / 2), 1)
        cam["calibrated"] = True
        save_section(None, "camera", cam)
        print("saved to config/nexus.yaml (camera.focal_length_px, camera.calibrated=true)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
