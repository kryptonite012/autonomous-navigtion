#!/usr/bin/env python3
"""Re-run odometry, mapping and scan matching offline on a flight recording.

    python3 scripts/replay.py data/recordings/run1.jsonl.gz
    python3 scripts/replay.py run1.jsonl.gz --no-scan-matching --map-out run1_odom
    python3 scripts/replay.py run1.jsonl.gz --k-rot 0.12 --gyro-std 0.01

Prints the trajectory error of the recorded estimate, raw odometry and the
replayed estimate (against simulation ground truth when the recording has
it), and optionally saves the rebuilt map. Use it to tune parameters on a
laptop with data recorded on the robot.
"""

import argparse
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from robot.localization.ekf import OdometryEKF  # noqa: E402
from robot.localization.localizer import compose, inverse  # noqa: E402
from robot.mapping.occupancy_grid import OccupancyGrid  # noqa: E402
from robot.mapping.scan_matcher import ScanMatcher  # noqa: E402
from robot.motors.kinematics import DiffDriveKinematics  # noqa: E402
from robot.recorder import read  # noqa: E402


def rms(errors):
    return math.sqrt(sum(e * e for e in errors) / len(errors)) if errors else float("nan")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("recording")
    ap.add_argument("--no-scan-matching", action="store_true")
    ap.add_argument("--map-out", help="save the rebuilt map under data/maps/<name>")
    ap.add_argument("--k-rot", type=float, default=None, help="EKF heading noise per radian turned")
    ap.add_argument("--gyro-std", type=float, default=None, help="EKF gyro rate noise (rad/s)")
    ap.add_argument("--no-gyro", action="store_true", help="ignore the recorded gyro")
    args = ap.parse_args()

    records = read(args.recording)
    meta = next(records)
    if meta.get("type") != "meta":
        print("not a NEXUS recording (missing meta record)")
        return 1
    ch, mc = meta["chassis"], meta["mapping"]
    kin = DiffDriveKinematics(ch["wheel_radius_m"], ch["wheel_base_m"], ch["max_linear_mps"] * 1.6)
    ekf = OdometryEKF(kin, ch["encoder_ticks_per_rev"])
    if args.k_rot is not None:
        ekf.k_rot = args.k_rot
    if args.gyro_std is not None:
        ekf.gyro_rate_std = args.gyro_std
    grid = OccupancyGrid.from_config(mc)
    matcher = None if args.no_scan_matching or not meta.get("lidar") else ScanMatcher()
    radius = ch.get("robot_radius_m", 0.12)

    correction = None
    err_rec, err_odom, err_rep = [], [], []
    steps = scans = accepted = 0
    raw_anchor = None
    for rec in records:
        if rec.get("type") != "step" or rec.get("ticks") is None:
            continue
        steps += 1
        gyro = None if args.no_gyro else rec.get("gyro")
        odom = ekf.update(tuple(rec["ticks"]), rec["t"], gyro).as_tuple()
        if correction is None:
            # anchor the replay where the recorded estimate started
            start = rec.get("pose") or rec["odom"]
            correction = compose(tuple(start), inverse(odom))
        pose = compose(correction, odom)
        scan = rec.get("scan")
        if scan is not None:
            scans += 1
            r = np.array([np.inf if v is None else v for v in scan["r"]])
            if matcher is not None:
                pts = ScanMatcher.scan_points(scan["amin"], scan["ainc"], r, scan["rmax"], matcher.max_beams)
                res = matcher.match(grid, pts, pose)
                if res is not None and res.accepted:
                    accepted += 1
                    correction = compose(res.pose, inverse(odom))
                    pose = res.pose
            grid.integrate_scan(pose, scan["amin"], scan["ainc"], r, scan["rmax"], step=2)
        grid.clear_area(pose[0], pose[1], radius * 0.8)
        for name, d, ang, valid in rec.get("ranges", []):
            if valid:
                dd = d if d is not None and d <= mc.get("max_range_m", 2.0) else None
                grid.integrate_ray(pose[0], pose[1], pose[2] + ang, dd, mc.get("max_range_m", 2.0),
                                   math.radians(7.5) if name.startswith("us_") else 0.0)
        gt = rec.get("gt")
        if gt is not None:
            if raw_anchor is None:
                # raw odometry, anchored at the true start pose
                raw_anchor = compose(tuple(gt), inverse(odom))
            err_odom.append(math.dist(compose(raw_anchor, odom)[:2], gt[:2]))
            if rec.get("pose"):
                err_rec.append(math.dist(rec["pose"][:2], gt[:2]))
            err_rep.append(math.dist(pose[:2], gt[:2]))

    print(f"recording : {args.recording}")
    print(f"robot     : {meta.get('robot')} ({meta.get('hardware_mode')}), lidar={meta.get('lidar')}, imu={meta.get('imu')}")
    print(f"steps     : {steps}, scans: {scans}" + (f", scan matches accepted: {accepted}" if matcher else ""))
    if err_rep:
        print("error vs ground truth (RMS / final):")
        if err_rec:
            print(f"  recorded estimate : {rms(err_rec):.3f} m / {err_rec[-1]:.3f} m")
        print(f"  raw odometry      : {rms(err_odom):.3f} m / {err_odom[-1]:.3f} m")
        print(f"  replayed estimate : {rms(err_rep):.3f} m / {err_rep[-1]:.3f} m")
    else:
        print("no ground truth in this recording (hardware run): compare maps visually")
    st = grid.stats()
    print(f"map       : free {st['free_ratio']:.3f}, occupied {st['occupied_ratio']:.3f}")
    if args.map_out:
        from robot.mapping.mapper import MapStore
        path = MapStore(mc.get("maps_dir", "data/maps")).save(grid, args.map_out,
                                                             {"experimental": not meta.get("lidar"),
                                                              "source": f"replay of {Path(args.recording).name}"})
        print(f"map saved : {path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
