# Bill of materials

Prices are typical Indian retail prices (Robu.in, Robocraze, ThinkRobotics,
Amazon.in) checked in 2026; expect ±20 % variation. GST included.

## Core build (navigation + sorting, no LiDAR)

| # | Component | Qty | Unit (₹) | Total (₹) | Notes |
|---|---|---|---|---|---|
| 1 | Raspberry Pi 5, 8 GB | 1 | 8,000 | 8,000 | Pi 4 (4 GB) works, see compatibility matrix |
| 2 | Raspberry Pi 5 Active Cooler | 1 | 550 | 550 | needed: the Pi throttles at 85 °C under vision load |
| 3 | 64 GB microSD, A2 / or 128 GB NVMe + HAT | 1 | 700 | 700 | NVMe (≈ ₹3,000) is faster and more reliable |
| 4 | Raspberry Pi Camera Module 3 (wide) | 1 | 3,300 | 3,300 | standard FOV version ≈ ₹2,700 |
| 5 | Camera cable for Pi 5 (22→15 pin, 300 mm) | 1 | 150 | 150 | Pi 5 uses the smaller connector |
| 6 | 2WD chassis plate (acrylic/aluminium, ~200 mm) + caster | 1 | 650 | 650 | |
| 7 | N20 / JGA25-370 12 V geared motor, 30:1, with magnetic encoder | 2 | 650 | 1,300 | must have quadrature encoder outputs |
| 8 | 65 mm wheels + hubs | 2 | 120 | 240 | match `wheel_radius_m` in config |
| 9 | TB6612FNG motor driver breakout | 1 | 220 | 220 | 1.2 A continuous per channel |
| 10 | HC-SR04 ultrasonic sensor | 3 | 70 | 210 | front, left, right |
| 11 | VL53L0X ToF breakout | 1 | 280 | 280 | low front, catches thin chair legs |
| 12 | MPU6050 IMU breakout (GY-521) | 1 | 150 | 150 | |
| 13 | ADS1115 16-bit ADC breakout | 1 | 350 | 350 | battery voltage |
| 14 | PCA9685 16-ch servo driver | 1 | 300 | 300 | offloads servo PWM from the Pi |
| 15 | MG90S metal-gear micro servo | 3 | 190 | 570 | arm base, shoulder, gripper |
| 16 | MG996R servo | 1 | 330 | 330 | sorter carousel |
| 17 | 2-DOF mini gripper arm kit | 1 | 650 | 650 | or 3D-print (STL not included) |
| 18 | IR break-beam sensor pair, 3 mm/5 mm | 1 | 250 | 250 | bin drop verification |
| 19 | 2S Li-ion 18650 holder + 2 × 18650 (2600 mAh) | 1 | 900 | 900 | 7.4 V nominal |
| 20 | 2S BMS board (with balancing) | 1 | 150 | 150 | over-discharge / short protection |
| 21 | 5 V 5 A buck converter (e.g. XL4015 / Pololu D36V50F5) | 1 | 450 | 450 | Pi 5 needs 5 V 5 A for full USB current |
| 22 | 5–6 V 3 A buck converter for servos | 1 | 250 | 250 | servos never on the Pi rail |
| 23 | Mushroom e-stop switch, NC, 22 mm | 1 | 350 | 350 | latching, twist-release |
| 24 | 5 V relay module (or P-MOSFET switch) for motor supply | 1 | 120 | 120 | opened by the e-stop |
| 25 | Rocker power switch, 10 A | 1 | 60 | 60 | main switch |
| 26 | Blade fuse holder + 5 A fuse | 1 | 80 | 80 | on battery positive |
| 27 | Resistors (1 kΩ, 2 kΩ, 10 kΩ, 20 kΩ), assortment | 1 | 100 | 100 | ultrasonic echo dividers, battery divider |
| 28 | Schottky diode 1N5822 | 2 | 15 | 30 | prevents backfeed into the Pi 5 V rail |
| 29 | Perfboard, JST/Dupont connectors, 22 AWG wire | 1 | 400 | 400 | |
| 30 | M3 standoffs, screws, double-sided tape | 1 | 250 | 250 | |
| 31 | Small plastic bins (×5) + carousel plate | 1 | 300 | 300 | |
| 32 | 18650 charger (2-slot) | 1 | 450 | 450 | |
| | **Core total** | | | **≈ 22,290** | |

## Optional upgrades

| Component | Price (₹) | What it enables |
|---|---|---|
| RPLIDAR A1M8 (360°, 12 m) | 8,500 | metric SLAM (SLAM Toolbox), reliable navigation, Nav2 |
| YDLIDAR X2 (budget alternative) | 6,000 | same, lower rate; needs `ydlidar_ros2_driver` instead of rplidar_ros |
| HX711 + 1 kg load cell | 300 | weight-based object/bin verification |
| Second camera (USB, gripper view) | 1,200 | better grasp alignment |
| NVMe HAT + 128 GB SSD | 3,000 | faster boot, fewer SD corruption issues |
| INA219 current sensor | 200 | motor current monitoring / stall detection |

**Recommended complete build with LiDAR: ≈ ₹30,800.**

## Hardware compatibility matrix

| Feature | Pi 5 8 GB | Pi 5 4 GB | Pi 4 4 GB | Pi 4 2 GB | Notes |
|---|---|---|---|---|---|
| Standalone stack (robot + FastAPI + dashboard) | ✅ | ✅ | ✅ | ⚠️ | Pi 4 2 GB: build the dashboard on a PC |
| Colour detection 640×480 @ ≥10 FPS | ✅ expected | ✅ expected | ⚠️ ~8–12 FPS expected | ⚠️ | to be measured, see benchmark report |
| ROS 2 Jazzy | ✅ via Ubuntu 24.04 or Docker | ✅ | ⚠️ Ubuntu 24.04 only | ❌ | Jazzy has no Raspberry Pi OS binaries |
| Nav2 + SLAM Toolbox | ✅ | ⚠️ tight | ⚠️ slow | ❌ | needs LiDAR |
| Camera Module 3 | ✅ (libcamera) | ✅ | ✅ | ✅ | Pi 5 needs the 22-pin cable |
| USB webcam | ✅ | ✅ | ✅ | ✅ | `camera.backend: opencv` |
| gpiozero/lgpio | ✅ | ✅ | ✅ | ✅ | RPi.GPIO does **not** work on Pi 5 |

| Sensor/actuator | Interface | Logic level | Driver in repo |
|---|---|---|---|
| TB6612FNG | GPIO + PWM | 3.3 V OK | `robot/motors/tb6612.py` |
| Hall encoders | GPIO | 3.3 V when powered from 3.3 V | `robot/sensors/gpio_sensors.py` |
| HC-SR04 | GPIO | **5 V echo → divider required** | `robot/sensors/gpio_sensors.py` |
| VL53L0X | I2C 0x29 (re-addressed) | 3.3 V | `robot/sensors/i2c_sensors.py` |
| MPU6050 | I2C 0x68 | 3.3 V | `robot/sensors/i2c_sensors.py` |
| ADS1115 | I2C 0x48 | 3.3 V | `robot/sensors/i2c_sensors.py` |
| PCA9685 | I2C 0x40 | 3.3 V logic, V+ 5–6 V | `robot/hardware/servos.py` |
| RPLIDAR A1 | USB serial | — | `robot/sensors/rplidar.py`, `rplidar_ros` |
| IR break-beam | GPIO, open collector | pull-up to 3.3 V | `robot/sensors/gpio_sensors.py` |

Missing hardware is detected at startup and the dependent features are
disabled (see the *Hardware* screen of the dashboard and
`HardwareSet.capabilities()`).
