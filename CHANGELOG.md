# Changelog

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
