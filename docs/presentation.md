---
marp: true
theme: default
paginate: true
title: PROJECT NEXUS
---

# PROJECT NEXUS
### Autonomous vision-based indoor navigation and colour sorting robot

Raspberry Pi 5 · OpenCV · FastAPI · React · ROS 2 (optional)

---

## Problem

* Indoor robots must **know where they are**, **avoid people and furniture**
  and **manipulate objects** — on low-cost hardware.
* Student/hobby projects often stop at disconnected scripts or a dashboard
  that fakes results.
* Goal: one platform that maps, navigates and sorts, runs offline, and is
  honest about what it has verified.

---

## Two operating modes

1. **Mapping & navigation** — occupancy grid, A* planning, pure pursuit,
   recovery, frontier exploration (Nav2 + SLAM Toolbox with LiDAR).
2. **Colour sorting** — HSV detection → approach → pick → verify lift →
   drop → **bin break-beam confirms** the sort.

---

## Hardware (≈ ₹22k core, ≈ ₹31k with LiDAR)

* Pi 5 8 GB, Camera Module 3, TB6612 + encoder gear motors
* 3× HC-SR04, VL53L0X, MPU6050, ADS1115
* PCA9685 + 4 servos (arm + carousel), IR break-beam
* 2S Li-ion + BMS, separate motor/servo supplies, **hardware e-stop relay**

---

## Architecture

* Hardware interfaces → physical drivers **or** simulation (same API)
* One **SafetyController** is the only path to the motors
* Robot runtime: 20 Hz control loop, vision thread, sorter thread
* FastAPI: REST + WebSocket + MJPEG; SQLite/PostgreSQL
* React dashboard: 12 screens

---

## Safety in depth

1. Physical e-stop cuts motor power (no software involved)
2. BMS
3. Software: e-stop latch, faults, stop/slow zones, stale-sensor block,
   0.6 s command watchdog, blind-reverse limit, stall detection
4. Navigator holds still if the pose is invalid
5. API: roles, rate limits, audit, e-stop for every user

---

## Vision

* Gaussian blur → HSV masks (2 ranges for red) → open/close
* Contours filtered by area, solidity, **contrast with surroundings**
* Confidence from shape, saturation, size, border cut
* Centroid tracker removes 1-frame blips
* Dashboard HSV calibration from a camera sample

---

## Results so far (simulation / dev machine)

| | |
|---|---|
| Tests | 89 passed, 7 HIL pending |
| Detection | 84 FPS on x86 VM (Pi not yet measured) |
| Synthetic accuracy | 98.6 %, 0 false positives |
| Simulated sorting | 5/5 correct, verified |
| Navigation to goal | reached < 0.2 m, 0 collisions |

---

## Honest limitations

* Without LiDAR the map is **experimental** (odometry drift, no loop closure)
* Single RGB camera ⇒ no metric depth unless calibrated + flat floor
* Same-colour object on same-colour floor is undetectable
* ROS 2 packages and all hardware numbers still to be validated on the robot

---

## Next steps

1. Build the robot, run HIL tests, fill the hardware benchmark
2. Add RPLIDAR A1 → SLAM Toolbox + Nav2
3. Load-cell verification, second gripper camera
4. Charging dock + return-to-dock behaviour

---

# Thank you

Repository: `github.com/kryptonite012/autonomous-navigtion`
