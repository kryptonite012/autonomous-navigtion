# Localisation

NEXUS estimates its pose in two layers, the same split ROS uses between the
`odom` and `map` frames.

```mermaid
flowchart LR
    ENC["Wheel encoders"] --> EKF["Odometry EKF<br/>(odom frame)"]
    GYRO["IMU gyro"] --> EKF
    EKF -- "odom pose + covariance" --> LOC["Localizer"]
    MAP[("Saved map")] --> PF["Particle filter (MCL)"]
    LIDAR["LiDAR scan"] --> PF
    RANGE["Ultrasonic / ToF"] --> PF
    LOC <--> PF
    LOC -- "map pose = correction ∘ odom pose" --> NAV["Navigator · mapper · sorter · dashboard"]
```

## 1. Odometry EKF (`robot/localization/ekf.py`)

Runs every control cycle (20 Hz) whether or not a map is loaded.

* **Motion increment** from the encoders: distance `ds` and heading change
  `dθ_enc`. Noise grows with distance and, more strongly, with rotation,
  because wheel slip mainly corrupts rotation.
* **Gyro** heading change `ω_z·dt`, with noise that grows with time instead
  (rate noise plus residual bias after the start-up calibration).
* The two heading increments are fused with inverse-variance weights, then
  the pose `[x, y, θ]` and its 3×3 covariance are propagated with the
  unicycle model's Jacobians.

Using the increments directly as the control input (rather than a
constant-velocity state) avoids the one-step lag a velocity model has. The
pose covariance is published to the dashboard (σ ring around the robot) and,
in the ROS base node, in `nav_msgs/Odometry.pose.covariance`.

The odometry pose always drifts: a few percent of the distance driven, more
on slippery floors.

## 2. Monte Carlo localisation (`robot/localization/mcl.py`)

Active when a map is loaded (*Mapping → Load*) and localisation is enabled.
It never runs while mapping.

| Part | Implementation |
|---|---|
| Motion model | sample-based odometry model (rot1, trans, rot2), noise α = (0.08, 0.15, 0.08, 0.02) |
| Sensor model | likelihood field: distance transform of the occupied cells, Gaussian σ_hit = 8 cm + 15 % random; vectorised over particles × beams |
| Beams | up to 72 LiDAR beams (subsampled to 36), plus the ultrasonic/ToF beams from their mounting offset |
| Resampling | low-variance, when N_eff < N/2 or the scan matches poorly |
| Recovery | random particles injected while the scan matches poorly (augmented MCL) |
| Update rate | after 5 cm or 0.1 rad of motion; ~1 ms per update with 400 particles on the dev VM |

The localiser keeps the **map ← odom correction** and only changes it when
the filter is confident. Between updates the pose follows odometry smoothly,
so the robot never sees a jump of more than one filter correction.

### Convergence and validity

The filter counts as **converged** when the particle cloud is tight
(σ < 15 cm, σθ < 0.25 rad) *and* the scan agrees with the map (mean per-beam
likelihood ≥ 0.7). A tight cloud with a poor scan match means it has
collapsed onto a look-alike place, so that is deliberately not called
converged.

The pose is reported **invalid** (the navigator holds still, missions and
navigation refuse to start) when:

* the cloud spreads beyond `lost_std_m` (0.5 m), or
* a global localisation is still searching.

### Initial pose

After loading a map, the filter starts around the robot's current estimate.
If the robot was carried somewhere else, tell it where it is:
*Navigation → Set pose on map* (click the position, enter the heading), or
`POST /api/localization/initial-pose`.

### Global localisation (LiDAR only)

*Navigation → Global*, or `POST /api/localization/global`. On the next full
scan the localiser runs a **correlative search**: every free position on a
10 cm grid × 36 headings is scored against the scan with the likelihood
field, and the filter is seeded around the 12 best candidates with 10× the
usual particle count. As the robot turns or moves, the wrong candidates lose
weight; once converged the filter shrinks back to 400 particles.

In simulation (5 × 4 m room) this re-localised a kidnapped robot to 3–4 cm
in 5 of 5 trials within about 2 s of spinning in place. Uniform random
particles alone sometimes locked onto the wrong place, which is why the
search is used.

## 3. Modes (`config/nexus.yaml → localization.mode`)

| Mode | Behaviour |
|---|---|
| `auto` (default) | MCL only when a LiDAR is detected |
| `on` | MCL always; without LiDAR it is labelled **experimental** |
| `off` | odometry EKF only |

Why `auto` excludes ultrasonic-only robots: three or four wide ultrasonic
beams carry too little information to localise. The filter can slow drift
when the starting pose is known, but it also tends to become overconfident.
Use `on` only to experiment, and keep resetting the pose when it drifts.

## 4. Known limitations

* **Wheel slip against furniture**: if the robot pushes against an obstacle
  in a sensor blind spot, the encoders report motion that did not happen.
  The filter can diverge. It then reports the pose invalid, and with a LiDAR
  the random injection or a global localisation recovers it. The three
  ultrasonic sensors leave blind sectors around ±30°: a front-corner bumper
  or a LiDAR closes them.
* **Changed rooms**: moved furniture lowers the scan match. Small changes
  are tolerated by the 15 % random term; large changes need a new map.
* **Symmetric rooms**: a perfectly symmetric room has several equally good
  poses. Global localisation then keeps several clusters until a
  distinguishing feature comes into view, and the pose stays invalid until it
  does.
* All numbers above are from simulation. Validate on the real robot before
  relying on them (`docs/test-report.md`).
