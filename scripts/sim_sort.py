#!/usr/bin/env python3
"""Run the colour sorting mode in the simulator and print the results."""

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
    ap.add_argument("--failure-rate", type=float, default=None)
    args = ap.parse_args()
    logging.basicConfig(level=logging.WARNING, format="%(levelname)s %(message)s")
    overrides = {"robot": {"mode": "simulation"}}
    if args.failure_rate is not None:
        overrides["simulation"] = {"pickup_failure_rate": args.failure_rate}
    robot = Robot(load_config(overrides=overrides))
    robot.start()
    time.sleep(0.5)
    robot.start_sorting()
    end = time.time() + args.minutes * 60
    last = None
    try:
        while time.time() < end and robot.sorter.running:
            time.sleep(0.5)
            s = robot.sorter.status()
            line = f"{s['state']:<15} {s['message']}"
            if line != last:
                print(line, flush=True)
                last = line
    finally:
        robot.stop_sorting()
        print("statistics:", robot.sorter.status()["statistics"])
        for rec in robot.sorter.history:
            print(" ", rec.as_dict())
        objs = robot.hw.world.objects
        print("world:", [(o.color, o.sorted_bin) for o in objs])
        robot.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
