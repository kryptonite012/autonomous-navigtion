# Performance benchmark report

Produced with `python3 scripts/benchmark.py [--sorting]`. Re-run on the
target Raspberry Pi and add a section below; the desktop numbers only show
relative cost and must not be quoted as Raspberry Pi performance.

## Targets (from the specification)

| Metric | Target |
|---|---|
| Dashboard updates | 5–10 Hz or better |
| Object detection | 10 FPS on suitable hardware |
| Navigation control | configured control rate (20 Hz) |
| Colour classification | ≥ 90 % on a defined test set, controlled lighting |
| Sorting success | ≥ 90 % over a defined repeated test |
| Emergency stop | measured and validated against the hardware design |

## Run 1 — development VM (x86-64), simulation, 2026-10-30

Host: x86-64 Linux VM · Python 3.13 · OpenCV 5.0

| Metric | Result | vs target |
|---|---|---|
| Colour detection, 640×480, 5 objects, single thread | 84 FPS (11.9 ms/frame) | ✅ on this host; Pi not measured |
| Detection accuracy, generated set (400 images, 992 objects, lighting ×0.6–1.15, noise σ=5, random occluders, tinted backgrounds) | 98.6 % (14 missed, 0 misclassified, 0 false positives) | ✅ synthetic set only |
| Colour classification accuracy of detected objects | 100 % | ✅ synthetic set only |
| A* on 160×160 grid (8 × 8 m at 5 cm), diagonal route | 23 ms | — |
| Dijkstra, same route | 77 ms | A* is 3.3× faster |
| Software e-stop latency (`emergency_stop()` → `motors.stop()` issued) | median 0.1 µs, max 1.3 µs | software path only |
| Simulated sorting, 3 min, 10 % random pickup failure | 5/5 objects in correct bin, all bin-sensor verified | ✅ simulation |
| Dashboard telemetry rate (WebSocket) | 10 Hz configured | ✅ |
| Control loop | 20 Hz configured | ✅ |

Before the contrast check was added, the same accuracy set produced 102
false positives (tinted backgrounds) at 6.3 ms/frame; the check removed all
of them at a cost of ~5 ms/frame.

**Interpretation.** The pipeline cost is dominated by per-colour masking and
morphology (5 colours × blur/open/close) plus the contrast check. A Pi 5 is
roughly 3–5× slower single-threaded than this VM, which would put the
detector at an estimated 17–28 FPS — but this is an estimate, not a
measurement. Synthetic accuracy is an upper bound: real objects have
specular highlights, shadows and colour casts.

## Run 2 — Raspberry Pi 5 (to be filled in)

| Metric | Result |
|---|---|
| Hardware | Pi 5 8 GB, active cooler, Camera Module 3, ... |
| Detection FPS (benchmark.py) | |
| Detection FPS in the running service (`/api/vision/status` fps) | |
| CPU temperature after 10 min of sorting | |
| Control loop rate / overruns (`/api/system/status`) | |
| Real-object classification (100 objects, list lighting) | |
| Physical sorting success (20 objects) | |
| Wheel stop time after e-stop (HIL `test_motor_stop_latency_with_encoders`) | |
