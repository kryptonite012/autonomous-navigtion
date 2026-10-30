# Mechanical assembly

Target layout (top view, front is up):

```
            ┌──────── camera (11 cm high, tilted 20° down)
            │   ┌──── gripper arm (folded in "home" pose)
      US-L  ▼   ▼  US-R           US = HC-SR04, angled ±60°
        ╲  ┌─────┐ ╱              ToF = VL53L0X, 1 cm above floor
   ┌─────╲─│ ARM │─╱─────┐
   │  ToF ▪└─────┘        │
  ┌┤      US-F            ├┐
  ││wheel  [Pi 5]   wheel││       wheels on the centre axis
  └┤   [TB6612][PCA9685]  ├┘
   │      [battery]        │
   │   [carousel + bins]   │       carousel behind the arm,
   └───────────○───────────┘       bins under the carousel slots
             caster
```

## Steps

1. **Chassis and motors.** Mount both gear motors on the centre line so the
   wheel axis passes through the robot's centre (this makes the robot turn
   on the spot, which the planner assumes). Measure the wheel separation
   (centre of tyre to centre of tyre) and enter it as
   `chassis.wheel_base_m`. Measure the wheel diameter and set
   `chassis.wheel_radius_m`.
2. **Caster** at the rear. Check the chassis sits level.
3. **Battery** low and central, held with a strap; BMS and fuse next to it.
   The main switch and the **e-stop button must be reachable from above**.
4. **Power board** (perfboard): buck converters, Schottky diode, relay,
   star ground point. Keep motor wiring away from the camera ribbon.
5. **Raspberry Pi** on standoffs with the active cooler fitted; leave
   airflow above the fan.
6. **TB6612 and PCA9685** close to the motors/servos to keep high-current
   leads short. Add the 470–1000 µF capacitor on the PCA9685 V+.
7. **Sensors**:
   * front HC-SR04 centred, facing forward, 3 cm above the floor;
   * side HC-SR04s at ±60°, matching `angle_deg` in the config;
   * VL53L0X at the very front, ~1 cm above the floor, to catch low
     obstacles and thin legs the ultrasonics miss;
   * MPU6050 flat, near the centre of rotation, axes aligned with the robot
     (x forward, z up); mount with foam tape to reduce vibration;
   * optional LiDAR on a mast at the top so nothing blocks its view; its
     cable leaves at the back.
8. **Camera** at the front on a rigid bracket, 11 cm above the floor,
   tilted 20° down so the pickup zone (15–20 cm in front) is in the lower
   part of the image. Measure the actual height and tilt and enter them as
   `camera.mount_height_m` / `mount_pitch_deg`.
9. **Arm** in front, centred on the camera axis. In the "grip" pose the
   gripper jaws must be 10–22 cm in front of the wheel axis (the reach the
   sorter assumes). In the "lift" pose the object must be above the
   carousel opening.
10. **Carousel and bins**: the MG996R turns a plate with one opening; each
    bin slot sits under the opening at the angles in `sorting.bin_angles`.
    Mount the IR break-beam across the drop chute.
11. **Cable management**: zip-tie everything; no loose wire may reach a
    wheel. Label the e-stop and power switch.

## After assembly

* Weigh the robot and check it does not tip when the arm is extended.
* Run `scripts/hardware_check.py --motor-test` with the wheels lifted.
* Continue with [calibration](../calibration.md).
