# Troubleshooting

Start with `journalctl -u nexus -n 200` and the *Hardware* screen: every
missing device has a reason.

## Startup and hardware

| Symptom | Likely cause | Fix |
|---|---|---|
| All devices `missing: gpiozero not installed` | venv without Pi packages | `pip install -r requirements-pi.txt`; venv must be created with `--system-site-packages` |
| `cannot open TB6612 pins: ... GPIO busy` | another process owns the pins (ROS base node, old instance) | stop it: `sudo systemctl stop nexus`, check `ps aux | grep python` |
| Motor driver `disabled because emergency stop input is missing` | e-stop wire not connected to GPIO 4 | wire it; this is deliberate — no motors without an e-stop input |
| `I2C bus 1 not available` | I2C disabled | `sudo raspi-config nonint do_i2c 0`, reboot |
| `MPU6050 not found at 0x68` | wiring / address | `i2cdetect -y 1`; AD0 high gives 0x69 → change `hardware.imu.address` |
| VL53L0X missing after a restart | sensor kept the old address | power-cycle; the XSHUT re-addressing runs at boot |
| `picamera2 not installed` | apt package missing | `sudo apt install python3-picamera2` |
| `no CSI camera detected` | ribbon cable | Pi 5 needs the 22-pin cable; contacts facing the right way; `rpicam-hello --list-cameras` |
| Camera `frame too dark` | lens cap / covered | check the lens; the vision pipeline rejects these frames |
| Pi reboots when motors start | supply sag | motors on a separate supply? Schottky + bulk capacitor on the 5 V rail; check `vcgencmd get_throttled` (0x50005 = under-voltage) |

## Motion

| Symptom | Cause | Fix |
|---|---|---|
| Robot does not move, header shows MOTION DISABLED | e-stop latched or fault | release button → Reset; see faults on Overview/Telemetry |
| `drive_stall` fault | wheels not turning while driven: jammed, motor unplugged, encoder dead | fix the cause, then clear the fault (`POST /api/robot/faults/clear {"fault":"drive_stall"}`) |
| `motor_communication` fault | driver exception (pins lost, I/O error) | check wiring; restart service |
| Robot stops in open space | stale/invalid front sensor or phantom echo | *Sensors* screen: a sensor showing "no data" blocks forward motion; check echo divider and wiring |
| One wheel runs backwards | motor polarity | swap motor leads or set `invert_left/right` |
| Robot curves when driving straight | wheel speed mismatch | calibrate `wheel_radius_m`; check tyre wear; the encoders correct pose but not motion |
| Commands are limited | safety controller | *Telemetry → Limited by* says why (slow zone, timeout, side too close …) |
| `battery_undervoltage` | pack below 6.6 V | charge or swap the pack, then clear the fault (refused while the voltage is still below 6.6 V) |

## Mapping and navigation

| Symptom | Cause | Fix |
|---|---|---|
| Walls appear twice / map smeared | odometry drift (expected without LiDAR) | shorter mapping runs; calibrate odometry; add a LiDAR |
| `no path found` | goal inside an obstacle/inflation or unknown space | click further from walls; `navigation.allow_unknown: true` (less safe); reduce `inflation_radius_m` carefully |
| `robot position is not in free space` | pose wrong after the robot was carried | put it back at the start point, reset the pose |
| Navigation `waiting_localization` | encoders not responding | check encoder wiring; the robot holds still by design |
| Endless recoveries | tight space smaller than robot + inflation | reduce inflation slightly or clear the area |

## Localisation and missions

| Symptom | Cause | Fix |
|---|---|---|
| Pose source says `ekf` although a map is loaded | no LiDAR and `localization.mode: auto` | expected; add a LiDAR or set `mode: on` (experimental) |
| Navigation refused: `localisation lost: set the initial pose` | particle cloud spread > 0.5 m | *Navigation → Set pose on map*; with LiDAR also **Global** |
| Status stays `searching` after Global | robot has not moved/turned, or the room is symmetric | spin the robot slowly; drive near a distinctive feature |
| Estimate jumps after bumping furniture | wheel slip against an obstacle | expected with blind spots (see localization.md); re-localise |
| Mission skips every waypoint | waypoints inside inflated obstacles or unknown space | place waypoints in free (white) map area, at least 20 cm from walls |
| Robot heads for the dock in the middle of a task | battery below `navigation.return_voltage` | charge; tune the voltage in the config |

## Vision and sorting

| Symptom | Cause | Fix |
|---|---|---|
| Colour not detected | thresholds / lighting | recalibrate HSV in the actual lighting (calibration §5) |
| Red and orange confused | overlapping hue ranges | tighten the ranges; use more distinct objects |
| Objects on a similarly coloured floor not found | contrast check rejects them | use objects that contrast with the floor |
| Always `failed_pickup` | pickup zone doesn't match the arm's reach | calibrate poses and `sorting.pickup_zone` (calibration §6–7) |
| Always `failed_verification` | break-beam misaligned | *Sensors*/HIL test `test_bin_sensor_counts_drop` |
| Only `unverified` results | no bin sensor configured | install the break-beam — results are never reported as success without it |
| Low FPS on Pi | CPU throttling, high resolution | check cooling/temperature; use 640×480; disable colours you don't need |
| No objects on the map | camera not calibrated | run `scripts/calibrate_camera.py` and set mount height/pitch (simulation is calibrated by construction) |

## Dashboard

| Symptom | Cause | Fix |
|---|---|---|
| "Disconnected" but REST works | WebSocket blocked | uvicorn needs `websockets` (installed by `uvicorn[standard]`); proxies must allow WS upgrade |
| `client network not allowed` | client IP not in `NEXUS_ALLOWED_NETWORKS` | add your subnet |
| Camera tile "unavailable — retrying" | no camera or token expired | check *Hardware*; log in again |
| Locked out of admin | forgot password | stop service, delete the user row or set `NEXUS_ADMIN_PASSWORD` and remove all users with `sqlite3 data/nexus.db "delete from users"` (recreates admin on start) |
