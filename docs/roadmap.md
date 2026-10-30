# Development roadmap and status

Legend: ✅ done and tested in simulation · 🟡 implemented, not yet validated
on hardware · ⏳ not started / needs hardware · ❌ out of scope for now

## Phase 1 — Foundation

| Item | Status |
|---|---|
| Repository structure, config, env template | ✅ |
| Hardware abstraction interfaces (init / validate / health / shutdown) | ✅ |
| Simulation world + simulated devices | ✅ |
| Hardware factory with missing-device detection and capabilities | ✅ (missing-hardware path tested) |
| Camera initialisation (Picamera2, OpenCV) | 🟡 |
| TB6612 motor driver | 🟡 |
| Raspberry Pi setup script, systemd units | 🟡 |

## Phase 2 — Computer vision

| Item | Status |
|---|---|
| HSV segmentation, multi-range red, blur, morphology | ✅ |
| Contour filters, contrast check, confidence score | ✅ |
| Centroid tracker | ✅ |
| Calibration (sample from camera, manual sliders, persist) | ✅ |
| Distance estimate (calibrated camera, flat floor only) | 🟡 |
| Real-camera accuracy test | ⏳ |

## Phase 3 — Mobility

| Item | Status |
|---|---|
| Differential drive kinematics, odometry, IMU blend | ✅ |
| Encoder and IMU drivers | 🟡 |
| Safety controller (zones, latches, watchdog, stall) | ✅ |
| Potential-field avoidance | ✅ |
| Physical obstacle stop distance | ⏳ |

## Phase 4 — Mapping

| Item | Status |
|---|---|
| Log-odds occupancy grid, ROS map export/import | ✅ |
| Map store (save, rename, load, delete) + dashboard | ✅ |
| Experimental range-sensor mapping | ✅ simulation |
| RPLIDAR driver (built-in) | 🟡 |
| SLAM Toolbox configuration | 🟡 (not run) |

## Phase 5 — Autonomous navigation

| Item | Status |
|---|---|
| Odometry EKF (encoders + gyro, covariance) | ✅ (v1.1) |
| Monte Carlo localisation on saved maps, correlative global search | ✅ simulation with LiDAR (v1.1) |
| Waypoint missions, patrol loops, skip/abort policy | ✅ simulation (v1.1) |
| Automatic return to dock on low battery | ✅ simulation (v1.1) |
| A*, Dijkstra, path simplification, clearance cost | ✅ |
| Pure pursuit, replanning, recovery, localisation-loss hold | ✅ |
| Frontier exploration | ✅ simulation |
| Named locations / return home | ✅ |
| Nav2 integration (params, launch, base node) | 🟡 (not built) |
| AMCL localisation on a saved map | 🟡 (Nav2 only; built-in MCL ✅ simulation) |

## Phase 6 — Sorting

| Item | Status |
|---|---|
| Colour→bin mapping, capacities, unclassified bin | ✅ |
| Search / approach / pick / lift-check / drop / verify state machine | ✅ |
| Retries, failed pickup, full bins, servo failure | ✅ |
| Break-beam verification; `unverified` without sensor | ✅ |
| Real arm poses, physical sorting rate | ⏳ |
| Load-cell verification | ⏳ (optional hardware) |
| Object map; sorter drives to remembered objects | ✅ simulation (v1.1) |

## Phase 7 — Dashboard

| Item | Status |
|---|---|
| FastAPI REST + WebSocket + MJPEG | ✅ |
| 12 dashboard screens + Missions + Simulation control room | ✅ (checked in browser against the simulator) |
| Live telemetry charts, alerts | ✅ |

## Phase 8 — Production readiness

| Item | Status |
|---|---|
| Auth, bcrypt, revocable sessions, RBAC, lockout | ✅ |
| Network allow-list, rate limiting, audit log | ✅ |
| Migrations, backups, retention | ✅ |
| Docker Compose | 🟡 (not built in CI) |
| Prometheus metrics | ✅ (v1.1) |
| Fault-injection test bench (simulation) | ✅ (v1.1) |
| Hardware-in-the-loop tests | ⏳ written, not run |
| Documentation | ✅ |

## Acceptance criteria (specification §20)

| | Criterion | Status |
|---|---|---|
| A | Pi boots and initialises the robot | ⏳ hardware |
| B | Camera captures live frames | 🟡 code ready; ✅ simulated |
| C | OpenCV identifies configured colours | ✅ synthetic + simulated; ⏳ real |
| D | Safe manual control | ✅ simulation; ⏳ hardware |
| E | Distance sensors trigger stopping | ✅ simulation; ⏳ hardware |
| F | Construct and save a map with mapping sensors | ✅ simulation (experimental); ⏳ LiDAR |
| G | Autonomous navigation to a destination | ✅ simulation; ⏳ hardware |
| H | Physically sort test objects | ⏳ hardware |
| I | Dashboard shows real telemetry | ✅ simulated telemetry; ⏳ hardware |
| J | API and WebSocket reliable | ✅ tests |
| K | Simulation and hardware clearly separated | ✅ |
| L | E-stop and fault handling validated | ✅ software; ⏳ physical relay |
| M | Documentation and reproducible setup | ✅ |

The project is **not complete** under §20 until the ⏳ items are validated on
the physical robot.
