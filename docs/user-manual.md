# User manual

## Safety first

* The red **mushroom button on the robot** cuts motor power instantly and
  works even if the software has crashed. Use it whenever something looks
  wrong. Twist to release; then press **Reset** next to the dashboard
  E-STOP button — motion stays disabled until an operator resets it.
* The **E-STOP button in the dashboard header** is available to every
  logged-in user on every screen.
* The robot never reverses faster than 6 cm/s (it has no rear sensors).
* The robot will not drive forward if the front sensors stop reporting.
* Stay within sight of the robot while it moves autonomously.

## Logging in and roles

Open `http://<robot>:8000`. Accounts are created by an admin under
*Settings*. Viewers can watch and stop; operators can drive and run tasks;
admins also manage users, safety limits and the system.

The header always shows: connection, **SIMULATION / HARDWARE** badge,
current mode, obstacle zone, battery, message lag and the E-STOP.

## Screens

| Screen | Use it to |
|---|---|
| Overview | check health at a glance: battery, CPU, pose, navigation, detections, sorted count, sensor health, e-stop |
| Live Camera | see the raw, annotated and mask feeds; start/stop the vision pipeline |
| Mapping | explore autonomously or map while driving; save/load/reset the map |
| Navigation | click on the map to send the robot somewhere; named locations; pause/resume/stop; manual drive pad |
| Object Detection | detection table; enable colours; tune filters; calibrate HSV thresholds |
| Color Sorting | start/pause/stop sorting; colour→bin mapping; bin fill levels; history; servo calibration |
| Telemetry | live charts of velocity, distances, battery, CPU; IMU; commanded vs applied velocity |
| Hardware | which devices were detected, which capabilities are enabled, configured pins |
| Sensors | top-down obstacle view with stop/slow zones; raw readings, encoders, battery |
| Map Management | load, rename, delete, download (PGM/YAML) saved maps |
| Event Logs | system events (filter, search, CSV export); admin: audit log |
| Settings | system info, backups, restart/power off, safety limits, users, password |

## Manual driving

*Navigation → Manual control*. Hold an arrow (or W/A/S/D, Q/E) to drive;
release to stop. Space or the STOP button stops immediately. Speeds are set
with the sliders and further limited by the safety controller (see
*Telemetry → Commanded vs applied* for why a command was reduced).

## Building a map

1. Place the robot in the room, wheels on the floor; the pose starts at
   (0, 0) facing +x.
2. *Mapping → Start autonomous exploration*. The robot first spins in place,
   then drives to the nearest unexplored boundary (frontier) repeatedly.
3. When it reports *exploration complete* (or you are satisfied), click
   **Stop mapping**, type a name and **Save**.

Without a LiDAR the dashboard shows **EXPERIMENTAL**: the map is a rough
outline from three ultrasonic beams and wheel odometry, and it drifts over
time. With a LiDAR, use the ROS 2 stack (SLAM Toolbox) for metric maps.

## Navigating

1. Load a saved map (or keep the one just built). Make sure the robot's
   pose matches: if you moved the robot, put it back at the map's start
   point and reset the pose.
2. Click a free spot on the map, or type coordinates, or pick a named
   location. The planned path appears as a dashed cyan line.
3. The robot replans when sensors see something blocking the path, tries
   up to 3 recoveries (back off, rotate) and then reports **failed** with
   the reason. It holds position if its pose becomes invalid.

## Sorting

1. Put the bins in place and empty them; click **Bins emptied**.
2. Check the colour→bin mapping (default red→1, green→2, blue→3, yellow→4,
   others→5) and the bin capacity.
3. Scatter test objects on the floor and click **Start**.
4. The robot searches (spins, relocates), approaches the largest object,
   aligns, picks it, checks the object left the floor, drops it into the
   bin and waits for the **bin break-beam** to confirm.
5. Results: `success` = confirmed by the bin sensor; `unverified` = no bin
   sensor installed; `failed_pickup` = still on the floor after the retries;
   `failed_verification` = released but the beam saw nothing.

Sorting stops by itself when no objects are found after 6 search cycles or
all bins are full.

## Shutting down

*Settings → Power off* (admin), or press the e-stop and switch off. Avoid
cutting power without shutting down: the SD card can be corrupted.
