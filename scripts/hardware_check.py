#!/usr/bin/env python3
"""Probe every configured device and print what was found.

Run this on the Pi before starting the service. It never drives the motors
unless --motor-test is given, and even then only for a short, slow pulse
(lift the robot first so the wheels spin freely).

    python3 scripts/hardware_check.py
    python3 scripts/hardware_check.py --motor-test
"""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from robot.config import load_config  # noqa: E402
from robot.hardware.factory import build_physical  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--motor-test", action="store_true", help="spin each wheel briefly at 25%% duty")
    ap.add_argument("--samples", type=int, default=5)
    args = ap.parse_args()

    cfg = load_config(overrides={"robot": {"mode": "hardware"}})
    hw = build_physical(cfg)
    print("\nDevice health")
    print("-" * 60)
    for h in hw.health():
        print(f"  {h.name:<16} {h.kind:<15} {h.state.value:<9} {h.message}")
    print("\nCapabilities")
    print("-" * 60)
    for k, v in hw.capabilities().items():
        print(f"  {k:<22} {'yes' if v else 'no'}")

    if hw.estop:
        print(f"\nE-stop pressed: {hw.estop.is_pressed()}")
    if hw.battery:
        v = hw.battery.voltage()
        print(f"Battery voltage: {v:.2f} V" if v is not None else "Battery voltage: read failed")
    for s in hw.ranges:
        vals = []
        for _ in range(args.samples):
            r = s.read()
            vals.append("err" if not r.valid else ("none" if r.distance_m is None else f"{r.distance_m:.3f}"))
            time.sleep(0.06)
        print(f"Range {s.name:<14} {' '.join(vals)}")
    if hw.imu:
        r = hw.imu.read()
        print(f"IMU accel {tuple(round(a, 2) for a in r.accel)} gyro {tuple(round(g, 3) for g in r.gyro)}")
    if hw.encoders:
        print(f"Encoders {hw.encoders.ticks()}")
    if hw.camera:
        time.sleep(1.0)
        ok, frame, _ = hw.camera.read()
        print(f"Camera frame ok={ok} shape={None if frame is None else frame.shape}")
    if hw.lidar:
        time.sleep(2.0)
        sc = hw.lidar.scan()
        print(f"LiDAR scan: {None if sc is None else int((sc.ranges < 50).sum())} valid beams")

    if args.motor_test:
        if not hw.motors:
            print("\nmotor test skipped: no motor driver (or no e-stop input)")
        elif hw.estop.is_pressed():
            print("\nmotor test skipped: release the emergency stop first")
        else:
            input("\nLift the robot so the wheels are free, then press Enter...")
            for name, duty in (("left forward", (0.25, 0)), ("right forward", (0, 0.25))):
                t0 = hw.encoders.ticks() if hw.encoders else None
                hw.motors.set_duty(*duty)
                time.sleep(0.8)
                hw.motors.stop()
                t1 = hw.encoders.ticks() if hw.encoders else None
                delta = None if t0 is None else (t1[0] - t0[0], t1[1] - t0[1])
                print(f"  {name}: encoder delta {delta}  (expect positive on the driven side)")
                time.sleep(0.5)
    hw.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
