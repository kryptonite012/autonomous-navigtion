# Test report

## Environment of this run

| | |
|---|---|
| Date | 2026-10-30 |
| Host | x86-64 Linux VM (development machine, **not a Raspberry Pi**) |
| Python / OpenCV | 3.13 / 5.0 |
| Mode | simulation (no physical hardware attached) |
| Command | `python3 -m pytest` |
| Result | **147 passed, 7 skipped** (the 7 skipped are hardware-in-the-loop tests) in ~4 min |

## Coverage by area

### Unit tests (`tests/unit`, 95 tests)

| Area | File | What is checked |
|---|---|---|
| Configuration | `test_config.py` | overrides, mode validation, persisted sections keep a backup |
| Kinematics/odometry | `test_kinematics.py` | inverse/forward round-trip, saturation keeps curvature, straight line and in-place rotation odometry, angle wrap |
| Occupancy grid | `test_occupancy_grid.py` | Bresenham, free/occupied/unknown marking, no-return rays, PGM/YAML round-trip, inflation |
| Planners | `test_planners.py` | A* and Dijkstra around a wall, equal path cost, unreachable goal, goal in obstacle, no corner cutting, path simplification, nearest free cell |
| Vision | `test_vision.py` | each of 5 colours; lighting ×0.55–1.15; red vs orange; 5 objects of one colour; partial occlusion and image-border objects (lower confidence); random noise; tiny blobs; plain background; low-solidity shapes; distance only when calibrated; tracker IDs and blip rejection; HSV range suggestion incl. red wrap-around; range validation; tinted floor rejected |
| Safety | `test_safety.py` | forward allowed when clear; stop zone blocks forward only; slow zone scaling; software e-stop latch/reset; hardware e-stop overrides and needs reset after release; stale and invalid sensors block forward; command watchdog; battery undervoltage fault; motor communication fault stays latched; drive stall; velocity limits; non-finite commands; e-stop call latency |
| Odometry EKF | `test_ekf.py` | straight line, rotation, gyro overrides a slipping wheel, covariance growth/symmetry, reset |
| Particle filter | `test_mcl.py` | simulated reference map; tracking corrects 8 % scale + heading drift (< 8 cm vs > 15 cm odometry); global localisation from scratch; minimum beams; localiser correction and auto/on modes |
| Scan matcher | `test_scan_matcher.py` | recovers ±8 cm / ±0.08 rad offsets to < 2.5 cm / 0.02 rad (this test exposed the half-cell map bias); rejects empty maps and sparse scans |
| DWA | `test_dwa.py` | free space straight and fast; turns towards a side target; swerves around an obstacle with clearance; boxed in → rotation only; dynamic window respects acceleration |
| Scheduler / coverage | `test_scheduler_coverage.py` | fires once per day within the grace window, weekday filter, skip when busy, validation; coverage waypoints in free space, alternating rows, limits |
| Sorting | `test_sorter.py` | verified success; unknown colour → unclassified bin; no bin sensor → `unverified` (never success); drop not detected → `failed_verification`; failed pickup with retries; full bin skipped; servo failure → error state; config validation |

### Integration tests (`tests/integration`, 52 tests)

| Area | File | What is checked |
|---|---|---|
| API | `test_api.py` | auth required on all routes; login lockout after 5 failures; logout revokes token; viewer/operator/admin permissions; viewer can e-stop; password policy; input validation (speed, path traversal in map names, HSV, bin numbers); e-stop blocks teleop and sorting until reset; navigation refused without a map; endpoint payloads; map save/rename/load/download/delete; WebSocket needs a token and streams telemetry; JPEG snapshot; audit log |
| Simulation closed loop | `test_simulation.py` | missing hardware disables features and motion; manual drive moves robot and odometry matches ground truth (< 3 cm); watchdog stops robot; obstacle triggers stop with no collision; sensor disconnect blocks forward; hardware e-stop latch; motor failure fault; battery undervoltage; localisation loss holds position; navigation to a goal on a saved map (reached within 0.2 m, 0 collisions); end-to-end sort verified by the bin sensor into the correct bin |
| Autonomy (simulation) | `test_autonomy.py` | MCL beats odometry while driving (< 10 cm); kidnapped robot re-localised (< 20 cm) with global search; ultrasonic-only robots keep MCL off; initial pose; 2-waypoint mission completes and returns home with 0 collisions; mission validation; low battery aborts the mission and heads to the dock; object map places the red object within 12 cm |
| API v1.1 | `test_api_v2.py` | missions CRUD + validation + persistence across restarts; localisation endpoints; simulation fault injection, obstacles, scatter, teleport (incl. into an obstacle → 422), ground-truth map, reset; simulation routes refused in hardware mode; object map; map snapshot layers; Prometheus metrics |
| v1.2 (simulation) | `test_autonomy.py`, `test_recorder.py` | LiDAR scan matching reduces mapping drift and raises wall precision vs odometry only (0 collisions); DWA and pure pursuit both reach the goal past an unmapped box and a crossing obstacle with 0 collisions; record a run, replay offline with/without scan matching (replay < 5 cm RMS and better than without); recording API (validation, single active recording, download only after stop) |
| Database | `test_database.py` | migrations create all tables and are idempotent; backup + restore; map name validation |

### Hardware-in-the-loop (`tests/hil`, 7 tests) — **NOT YET RUN**

Run on the robot: `NEXUS_HIL=1 python3 -m pytest tests/hil -s`. They cover
required devices, camera frames, front sensor detecting a hand, e-stop
input, wheel stop time after `stop()`, battery reading range and the bin
break-beam.

## Scenario matrix from the specification

| Scenario | Status | Evidence |
|---|---|---|
| **Vision** different lighting / similar colours / multiple / occluded / false detections | ✅ synthetic | `test_vision.py`; benchmark accuracy below |
| Vision with real camera and objects | ⏳ pending hardware | |
| **Navigation** empty room / furniture | ✅ simulation | `test_navigate_to_goal_with_saved_map`, `scripts/sim_explore.py` |
| Narrow passages | ⚠️ partial | planner unit tests; not measured on a real robot |
| Dynamic obstacles | ✅ simulation | `test_obstacle_triggers_stop` (stop), navigator replans from range obstacles |
| Localisation failure | ✅ simulation | `test_localization_loss_holds_position`; kidnapped robot + global localisation (`test_kidnapped_robot_global_localization`) |
| **Sorting** correct colour / wrong classification / failed pickup / full bin / servo failure / release verification | ✅ unit + simulation | `test_sorter.py`, `test_sorting_end_to_end` |
| Physical sorting with the real arm | ⏳ pending hardware | |
| **Safety** e-stop / sensor disconnection / motor failure / undervoltage / communication timeout | ✅ unit + simulation | `test_safety.py`, `test_simulation.py` |
| Software crash | ⚠️ by design | physical e-stop relay is independent of software; systemd restarts the service; watchdog stops motors if commands stop. Not yet demonstrated on hardware. |
| Physical e-stop cuts motor power | ⏳ pending hardware | measure with a multimeter on TB6612 VM |

## Simulation runs (manual scripts)

| Script | Result on this machine |
|---|---|
| `scripts/sim_explore.py --minutes 1.8` | explored the 5 × 4 m room, 0 collisions, final odometry error 0.15 m vs ground truth (expected drift without LiDAR), map saved |
| `scripts/sim_sort.py --minutes 2.8` | 4/5 objects sorted, all confirmed by the simulated bin sensor (time limit reached on the 5th) |
| `scripts/benchmark.py --sorting` (3 min, 10 % pickup failure) | 5/5 objects in correct bins |

## Known issues / limitations found during testing

* (v1.2) The scan-matcher unit test showed a constant 2–3 cm offset even
  from the exact pose: every map was shifted half a cell by boundary
  rounding of ray hits. Fixed in `OccupancyGrid.integrate_ray`, with
  bilinear field lookups; MCL accuracy improved as a side effect.
* (v1.2) With LiDAR, a robot whose filter lost confidence right after a
  pose reset stopped (correctly) and then never recovered, because the
  filter only updated on motion. It now updates while stationary.
* (v1.2) The first DWA version crept straight at obstacles: a slow arc did
  not reach them within the time horizon. Collision checks now cover at
  least 0.45 m along every arc.
* (v1.2) The simulator counted a moving obstacle hitting a stopped robot as
  a robot collision (288 "collisions" in one run). Collisions and contacts
  are now counted separately.
* (v1.2) Gray-world white balance lowers accuracy on tinted scenes
  (98 → 91 % under neutral light), so it is off by default; the reference
  mode does not have this problem.

* (v1.1) Uniform random particles alone sometimes locked onto a look-alike
  pose after a kidnap (2 of 5 trials). Fixed by the correlative global
  search and by requiring a good scan match before reporting convergence;
  5 of 5 trials then re-localised to 3–4 cm.
* (v1.1) A scripted test drive that curved into the table through the
  ultrasonic blind spot made the wheels slip against it, and the particle
  filter diverged (0.3 m). The test route was moved to open floor; the
  failure mode is documented in docs/localization.md.
* (v1.1) The first EKF design (constant-velocity state) lagged one step
  behind during constant motion (3° error over a 90° turn). It was redesigned
  to use encoder increments as the control input.

* Without LiDAR, odometry drift reached ~0.15 m after ~2 minutes of
  exploration in simulation (2 % simulated wheel scale error). Maps from
  range sensors are labelled experimental.
* A coloured object on a floor of the same colour cannot be separated by
  colour segmentation (now rejected by the contrast check instead of
  producing false positives).
* Objects further than ~1.5 m are below the minimum blob size at 640×480;
  the sorter relocates to find them.
* The ROS 2 packages have not been built yet (no ROS in the test
  environment).

## To record after hardware testing

| Item | Target | Measured |
|---|---|---|
| Odometry error over 2 m straight | < 2 % | |
| Heading error after 5 turns | < 5° | |
| Obstacle stop distance at max speed | < emergency distance | |
| Physical e-stop: motor power off | < 100 ms | |
| Software e-stop to wheels stopped | < 500 ms | |
| Colour classification, 100 objects, room lighting | ≥ 90 % | |
| Sorting success, 20 objects | ≥ 90 % | |
| Detection FPS on Pi 5 | ≥ 10 FPS | |
