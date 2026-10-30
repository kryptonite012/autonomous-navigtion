#!/usr/bin/env python3
"""Performance and accuracy benchmarks.

Measures, on whatever machine it runs on:
  * colour detection throughput (640x480 frames, 5 objects)
  * colour classification accuracy on a generated test set with lighting
    variation, noise and partial occlusion (ground truth known)
  * A* / Dijkstra planning time on an 8 m x 8 m, 5 cm grid
  * software emergency-stop latency (call -> motor stop issued)
  * simulated sorting success rate (optional, slow: --sorting)

The output is Markdown so it can be pasted into docs/benchmark-report.md
together with the hardware description. Numbers from a desktop are NOT
Raspberry Pi numbers: run it on the Pi for the real report.
"""

import argparse
import math
import platform
import statistics
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from robot.config import load_config  # noqa: E402
from robot.vision.detector import ColorDetector, VisionParams  # noqa: E402

BGR = {"red": (30, 30, 200), "green": (50, 170, 40), "blue": (200, 80, 20),
       "yellow": (30, 210, 230), "orange": (20, 125, 245)}


def make_scene(rng, n=3):
    bg = tuple(int(v) for v in rng.integers(90, 170, 3))
    img = np.full((480, 640, 3), bg, np.uint8)
    truth = []
    placed = []
    for _ in range(n):
        for _attempt in range(30):
            r = int(rng.integers(22, 55))
            x, y = int(rng.integers(r + 5, 635 - r)), int(rng.integers(r + 5, 475 - r))
            if all(math.dist((x, y), p[:2]) > r + p[2] + 10 for p in placed):
                break
        color = list(BGR)[int(rng.integers(0, len(BGR)))]
        shade = np.array(BGR[color]) * rng.uniform(0.85, 1.1)
        cv2.circle(img, (x, y), r, tuple(int(c) for c in np.clip(shade, 0, 255)), -1)
        placed.append((x, y, r))
        truth.append((color, x, y, r))
    if rng.random() < 0.3:      # occluder
        x, y = int(rng.integers(0, 600)), int(rng.integers(0, 440))
        cv2.rectangle(img, (x, y), (x + 40, y + 40), bg, -1)
    gain = rng.uniform(0.6, 1.15)
    img = np.clip(img.astype(np.float32) * gain + rng.normal(0, 5, img.shape), 0, 255).astype(np.uint8)
    return img, truth


def bench_vision(cfg, frames=300):
    det = ColorDetector(VisionParams.from_config(cfg["vision"]))
    rng = np.random.default_rng(42)
    scenes = [make_scene(rng, 5)[0] for _ in range(20)]
    det.detect(scenes[0])
    t0 = time.perf_counter()
    for i in range(frames):
        det.detect(scenes[i % len(scenes)])
    dt = time.perf_counter() - t0
    return frames / dt, dt / frames * 1000


def bench_accuracy(cfg, n_images=400):
    det = ColorDetector(VisionParams.from_config(cfg["vision"]))
    rng = np.random.default_rng(7)
    tp = fn = wrong = fp = 0
    for _ in range(n_images):
        img, truth = make_scene(rng, int(rng.integers(1, 5)))
        dets, _ = det.detect(img)
        used = set()
        for color, x, y, r in truth:
            match = None
            for i, d in enumerate(dets):
                if i not in used and math.dist(d.centroid, (x, y)) < r:
                    match = i
                    break
            if match is None:
                fn += 1
                continue
            used.add(match)
            if dets[match].color == color:
                tp += 1
            else:
                wrong += 1
        fp += len(dets) - len(used)
    total = tp + fn + wrong
    return {"objects": total, "correct": tp, "misclassified": wrong, "missed": fn, "false_positives": fp,
            "accuracy": tp / total, "classification_accuracy": tp / max(1, tp + wrong)}


def bench_planner(cfg):
    from robot.mapping.occupancy_grid import OccupancyGrid
    from robot.navigation.planners import plan
    g = OccupancyGrid.from_config(cfg["mapping"])
    blocked = np.zeros((g.height, g.width), bool)
    rng = np.random.default_rng(1)
    for _ in range(40):
        x, y = rng.integers(0, g.width - 20), rng.integers(0, g.height - 20)
        blocked[y:y + int(rng.integers(5, 20)), x:x + int(rng.integers(5, 20))] = True
    blocked[5:15, 5:15] = False
    blocked[-15:-5, -15:-5] = False
    out = {}
    for algo in ("astar", "dijkstra"):
        times = []
        for _ in range(5):
            t0 = time.perf_counter()
            p = plan(blocked, (10, 10), (g.width - 10, g.height - 10), algo)
            times.append((time.perf_counter() - t0) * 1000)
        out[algo] = (statistics.median(times), len(p) if p else None)
    return out


def bench_estop():
    from robot.hardware.base import MotorDriver
    from robot.motors.kinematics import DiffDriveKinematics
    from robot.safety.controller import SafetyConfig, SafetyController

    class M(MotorDriver):
        def __init__(self):
            super().__init__("m")
            self.t_stop = None

        def set_duty(self, l, r):
            pass

        def stop(self):
            self.t_stop = time.perf_counter()

    lat = []
    for _ in range(200):
        m = M()
        sc = SafetyController(m, DiffDriveKinematics(0.033, 0.16, 0.48), SafetyConfig())
        t0 = time.perf_counter()
        sc.emergency_stop("bench")
        lat.append((m.t_stop - t0) * 1e6)
    return statistics.median(lat), max(lat)


def bench_sorting(minutes):
    from robot.runtime import Robot
    cfg = load_config(overrides={"robot": {"mode": "simulation"}, "simulation": {"pickup_failure_rate": 0.1}})
    r = Robot(cfg)
    r.start()
    time.sleep(0.5)
    r.start_sorting()
    end = time.time() + minutes * 60
    while time.time() < end and r.sorter.running:
        time.sleep(1)
    r.stop_sorting()
    st = r.sorter.status()
    placed = sum(1 for o in r.hw.world.objects if o.sorted_bin is not None)
    correct = sum(1 for o in r.hw.world.objects if o.sorted_bin == r.sorter.bin_for(o.color))
    r.shutdown()
    return st["statistics"], placed, correct, len(r.hw.world.objects)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--sorting", action="store_true", help="also run the simulated sorting benchmark")
    ap.add_argument("--sorting-minutes", type=float, default=4.0)
    args = ap.parse_args()
    cfg = load_config()
    print(f"## Benchmark results\n\nHost: `{platform.node()}` · {platform.machine()} · "
          f"{platform.processor() or 'n/a'} · Python {platform.python_version()} · OpenCV {cv2.__version__}\n")
    fps, ms = bench_vision(cfg)
    print(f"| Metric | Result |\n|---|---|\n| Colour detection, 640x480, 5 objects | {fps:.1f} FPS ({ms:.1f} ms/frame) |")
    acc = bench_accuracy(cfg)
    print(f"| Detection accuracy (synthetic set, {acc['objects']} objects) | {acc['accuracy'] * 100:.1f} % "
          f"(missed {acc['missed']}, misclassified {acc['misclassified']}, false positives {acc['false_positives']}) |")
    print(f"| Colour classification accuracy (detected objects) | {acc['classification_accuracy'] * 100:.1f} % |")
    pl = bench_planner(cfg)
    for algo, (t, n) in pl.items():
        print(f"| {algo} on 160x160 grid, diagonal route | {t:.1f} ms ({n} cells) |")
    med, worst = bench_estop()
    print(f"| Software e-stop latency (call → motor stop) | median {med:.1f} µs, max {worst:.1f} µs |")
    if args.sorting:
        stats, placed, correct, total = bench_sorting(args.sorting_minutes)
        print(f"| Simulated sorting ({args.sorting_minutes} min, 10% pickup failure) | {correct}/{total} objects in "
              f"correct bin; records {stats} |")
    print("\nSoftware e-stop latency excludes the motor driver and mechanics; the physical e-stop relay "
          "must be measured separately (see docs/test-report.md).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
