#!/usr/bin/env python3
"""Run frontier exploration in the simulator and save the resulting map.

Usage: python3 scripts/sim_explore.py [--minutes 3] [--name sim_living_room]
"""

import argparse
import logging
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from robot.config import load_config  # noqa: E402
from robot.runtime import Robot  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--minutes", type=float, default=3.0)
    ap.add_argument("--name", default="sim_living_room")
    ap.add_argument("--lidar", action="store_true", help="enable the simulated LiDAR")
    args = ap.parse_args()
    logging.basicConfig(level=logging.WARNING, format="%(levelname)s %(message)s")

    overrides = {"robot": {"mode": "simulation"}}
    if args.lidar:
        overrides["hardware"] = {"lidar": {"enabled": True}}
    robot = Robot(load_config(overrides=overrides))
    robot.start()
    time.sleep(0.5)
    robot.start_mapping(explore=True, reset=True)
    end = time.time() + args.minutes * 60
    try:
        while time.time() < end:
            time.sleep(5)
            s = robot.state()
            gt = robot.ground_truth()
            p = s["pose"]
            err = ((p["x"] - gt["x"]) ** 2 + (p["y"] - gt["y"]) ** 2) ** 0.5
            print(f"{s['navigation']['state']:<12} pose=({p['x']:+.2f},{p['y']:+.2f}) "
                  f"odom_err={err:.3f}m free={s['mapping']['free_ratio']:.3f} "
                  f"occ={s['mapping']['occupied_ratio']:.3f} collisions={robot.hw.world.state.collisions} "
                  f"{s['navigation']['message']}", flush=True)
            if s["navigation"]["state"] in ("reached", "failed", "idle"):
                break
    finally:
        path = robot.mapper.save(args.name)
        print("map saved to", path)
        robot.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
