# Changelog

## 1.2.0

### Mapping and localisation
- **LiDAR scan-to-map matching while mapping** (correlative coarse + fine
  search). Simulated final pose error drops from 31–63 cm to under 2 cm, and
  map wall precision rises from 20–31 % to 83–100 %. About 6 ms per match.
- **Fixed a half-cell (2.5 cm) shift in every map**, caused by ray hits on
  cell boundaries rounding differently in different directions. Hits are
  now pushed a quarter cell past the surface, and distance fields use
  bilinear interpolation. MCL tracking error 4.3 → 1.9 cm, global
  localisation 3.3 → 0.8 cm.
- **Fixed a localisation lock-up**: a filter that lost confidence while the
  robot was stopped never updated again. It now refreshes while stationary.
  The scan-match score is weight-based, and random particles are injected
  before weighting.
- Fixed float32 particle weights (precision loss and log(0) warnings).

### Navigation
- **Dynamic Window Approach** local planner, now the default controller:
  collision checks at least 0.45 m along every arc, uses map walls, fresh
  range hits and LiDAR points, and shows its trajectory on the map. Pure
  pursuit stays selectable.
- Stuck detection counts turning towards the path as progress.
- **Coverage missions**: lawnmower sweeps generated from the map.
- **Mission schedules**: weekday/time schedules, stored in the database
  (migration 4); skipped runs are logged.

### Vision
- **Shape classification** (circle, square, rectangle, triangle, other).
- **White balance**: gray-world, and a reference mode measured from a white
  sheet. Under coloured casts, accuracy improves from 63–80 % to 97–98 %
  (synthetic benchmark) with no loss under neutral light.

### Tooling
- **Flight recorder**: raw sensor logs (encoders, gyro, ranges, LiDAR,
  commands, estimates, ground truth) to `.jsonl.gz`; download from the
  dashboard.
- **`scripts/replay.py`**: re-runs odometry, mapping and scan matching
  offline with different parameters and reports trajectory errors.
- Simulator separates robot-caused collisions from contacts by moving
  obstacles; moving obstacles bounce off the robot and furniture.

### Dashboard
- Works on phones (drawer menu, compact header, scrollable tables).
- Flight recorder tab, white balance controls, shape column, scan-matching
  statistics, DWA trajectory, coverage generator, schedules, ● REC
  indicator.

## 1.1.0

### Localisation
- Odometry **EKF** fusing encoder increments with the gyro heading rate,
  with pose covariance (replaces the fixed complementary blend). Also used
  by the ROS base node, which now publishes the real covariance.
- **Monte Carlo localisation** against saved maps: likelihood-field sensor
  model, odometry motion model, low-variance resampling, random injection
  while the scan matches poorly. About 1 ms per update with 400 particles.
- **Global localisation** by correlative search over position × heading,
  seeding the filter around the best candidates. Convergence requires a
  good scan match, not only a tight cloud.
- The pose is reported invalid while the filter is lost or searching, so
  navigation holds still.
- Initial pose from the dashboard (*Navigation → Set pose on map*).

### Autonomy
- **Missions**: waypoint sequences and patrol loops with dwell times,
  maximum loops, return home, and a skip/abort policy. Stored in the
  database (migration 3).
- **Return to dock on low battery**: aborts the running task below
  `navigation.return_voltage`.
- **Object map**: calibrated detections are placed in world coordinates.
  The sorter drives to remembered objects instead of searching blindly.

### Simulation and operations
- **Simulation control room**: fault injection (motor, encoder, servo,
  camera, e-stop, individual range sensors, battery), kidnapped-robot test,
  static and moving obstacles, scatter objects, ground-truth reference map,
  reset. Refused in hardware mode.
- The simulated camera counts as calibrated, since it is rendered with
  exactly the configured intrinsics.
- `GET /metrics` in Prometheus format.

### Dashboard
- New screens: **Missions** and **Simulation**.
- Map layers: particle cloud, 2σ uncertainty ring, mapped objects, true
  pose (simulation).
- Localisation card on the Navigation screen; mission and localisation on
  Overview; object map table on Object Detection.

### Tests
- 115 tests (from 89): EKF, particle filter tracking and global
  localisation, kidnapped robot, missions, low-battery return, object map,
  simulation control API, missions API and persistence, metrics.

## 1.0.0

First release: hardware abstraction and simulator, HSV vision, safety
controller, occupancy-grid mapping, A*/Dijkstra navigation with frontier
exploration, colour sorting with bin-sensor verification, FastAPI backend
with authentication, React dashboard (12 screens), ROS 2 packages, and
documentation.
