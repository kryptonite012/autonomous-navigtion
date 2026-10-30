# NEXUS ROS 2 workspace

Optional ROS 2 Jazzy integration. The standalone stack (`robot/` + `backend/`)
does not need ROS; use this workspace when you want Nav2 and SLAM Toolbox,
which require a 2D LiDAR.

| Package | Purpose |
|---|---|
| `nexus_description` | URDF + robot_state_publisher |
| `nexus_hardware` | base driver: `/cmd_vel` → safety controller → motors; `/odom`, TF, `/range/*`, `/imu/data_raw`, `/battery`, e-stop services |
| `nexus_navigation` | Nav2 parameters (regulated pure pursuit, NavFn A*, range sensor layer), SLAM Toolbox parameters |
| `nexus_vision` | `/vision/detections` (vision_msgs/Detection2DArray), annotated JPEG |
| `nexus_sorting` | sorting state machine as `/sorting/{start,stop,pause,resume}` services |
| `nexus_bringup` | `nexus.launch.py mode:=mapping|navigation|sorting|base` |

## Build

```bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
  ros-jazzy-rplidar-ros ros-jazzy-vision-msgs ros-jazzy-robot-state-publisher
source /opt/ros/jazzy/setup.bash
cd ros2_ws && colcon build --symlink-install
source install/setup.bash
export NEXUS_ROOT=$(cd .. && pwd)
```

## Run

```bash
ros2 launch nexus_bringup nexus.launch.py mode:=mapping
ros2 run nav2_map_server map_saver_cli -f ~/maps/room       # save when done
ros2 launch nexus_bringup nexus.launch.py mode:=navigation map:=$HOME/maps/room.yaml
```

Do not run the FastAPI backend at the same time as `nexus_base`: both would
try to own the motor driver and GPIO pins.

## Status

These packages are written against the Jazzy APIs but have **not been built
or run on a robot yet** (the development environment had no ROS install).
Treat them as untested until `colcon build` and a LiDAR mapping run succeed;
record the results in `docs/test-report.md`.
