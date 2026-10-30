# Calibration

Do these in order; each step depends on the previous ones.

## 1. Wheel odometry

1. Mark a start line, place the robot on it, reset the pose
   (*Navigation* or `POST /api/robot/pose/reset`).
2. Drive straight 2.0 m (measured with a tape) using the dashboard.
3. Read `x` on the *Overview* screen.
   `wheel_radius_m_new = wheel_radius_m × 2.0 / x_reported`
4. Spin in place 5 full turns (10π rad) and read the accumulated heading
   change from the logs or the telemetry (`theta` wraps at ±π; count turns).
   `wheel_base_m_new = wheel_base_m × θ_reported / θ_true`
   With an IMU the heading is blended 85 % gyro, so check this with
   `hardware.imu.enabled: false` first.
5. Repeat until the error is < 2 %. Record the values in
   `docs/test-report.md`.

## 2. IMU

The gyro bias is measured automatically for 0.4 s at start-up: **keep the
robot still while the service starts.** If the heading drifts when
stationary, restart with the robot motionless.

## 3. Distance sensors

For each sensor place a flat board at 0.20, 0.50 and 1.00 m and compare with
*Sensors* readings. HC-SR04 typically reads ±1–2 cm; if one is far off,
check the echo divider. Soft materials (cushions, curtains) absorb
ultrasound: test with the actual furniture.

Set the safety zones in *Settings → Safety limits* (persist in
`config/nexus.yaml`):

* `emergency_stop_distance_m` ≥ stopping distance at max speed + 5 cm.
  Measure: drive at max speed towards a box, note where it stops.
* `slow_down_distance_m` > stop distance + 0.15 m.

## 4. Camera intrinsics (optional, enables distance estimates)

```bash
python3 scripts/calibrate_camera.py --square 0.025
```

Use a 9×6 inner-corner chessboard printed flat. Aim for RMS < 0.5 px. Then
measure the camera height above the floor and its downward tilt and set
`camera.mount_height_m` and `camera.mount_pitch_deg`. Distance estimates
assume objects resting on a flat floor and appear as "n/a" until
`camera.calibrated: true`.

## 5. Colour thresholds (HSV)

Lighting changes colours a lot. Calibrate under the lighting you will
demonstrate in.

1. *Object Detection* screen → choose a colour.
2. Place one object of that colour in the centre of the view.
3. Set the sample region (default `270,190,100,100` is the image centre)
   and click **Sample from camera** — thresholds are proposed from the 5th–95th
   percentiles of the pixels (two ranges for red, which wraps around hue 0).
4. Fine-tune with the sliders while watching the mask view: the object
   should be solid white, the background black.
5. **Apply & save** writes `vision.colors` to `config/nexus.yaml`.
6. Repeat for each colour, then check with all objects in view that no two
   colours claim the same object.

Tips: matte objects work far better than glossy ones; avoid colours close to
the floor colour (the detector rejects blobs that do not contrast with their
surroundings, so a blue object on a blue floor cannot be detected).

## 6. Servos and sorting poses

1. *Color Sorting → Servo calibration*: move each joint to find
   * `home`: arm folded, camera view unobstructed;
   * `pre_grip`: gripper open just above an object in the pickup zone;
   * `grip`: shoulder lowered so the jaws surround the object;
   * `lift`: object above the carousel opening;
   * `gripper_open` / `gripper_closed` (closed must actually squeeze a test
     object without stalling the servo for long).
2. For each bin turn the `sorter` channel until the opening is over that
   bin; note the angle in `sorting.bin_angles`.
3. Enter the values in `config/nexus.yaml → sorting.poses` and restart.

## 7. Pickup zone

Put an object where the gripper reliably picks it. On *Object Detection*
read its bounding box; the bottom edge (`(y + h) / 480`) and centre
(`(x + w/2) / 640`) must fall inside `sorting.pickup_zone`. Adjust the zone
so it covers the positions where picking succeeds, and only those.

## 8. Battery

Measure the pack voltage with a multimeter and compare with *Sensors*.
Adjust `hardware.battery.divider_ratio` until they agree within 0.05 V.
