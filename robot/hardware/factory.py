"""Builds the set of devices for the configured mode.

In hardware mode every device is probed. A device that is disabled in the
config or fails to initialise is left out, and the capabilities that need it
are switched off (see ``HardwareSet.capabilities``).
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Callable, Optional

from robot.hardware.base import (BatteryMonitor, BinSensor, Camera, Device, DeviceHealth,
                                 DistanceSensor, EmergencyStopInput, EncoderPair,
                                 HardwareUnavailable, HealthState, Imu, Lidar, MotorDriver,
                                 ServoController)

log = logging.getLogger(__name__)


@dataclass
class HardwareSet:
    mode: str
    motors: Optional[MotorDriver] = None
    encoders: Optional[EncoderPair] = None
    ranges: list[DistanceSensor] = field(default_factory=list)
    imu: Optional[Imu] = None
    lidar: Optional[Lidar] = None
    servos: Optional[ServoController] = None
    camera: Optional[Camera] = None
    battery: Optional[BatteryMonitor] = None
    estop: Optional[EmergencyStopInput] = None
    bin_sensor: Optional[BinSensor] = None
    unavailable: list[DeviceHealth] = field(default_factory=list)
    world: object = None   # SimWorld in simulation mode

    def devices(self) -> list[Device]:
        out = [self.motors, self.encoders, self.imu, self.lidar, self.servos,
               self.camera, self.battery, self.estop, self.bin_sensor, *self.ranges]
        return [d for d in out if d is not None]

    def health(self) -> list[DeviceHealth]:
        return [d.health() for d in self.devices()] + list(self.unavailable)

    def capabilities(self) -> dict[str, bool]:
        has_odom = self.encoders is not None
        return {
            "drive": self.motors is not None and self.estop is not None,
            "odometry": has_odom,
            "obstacle_detection": len(self.ranges) > 0 or self.lidar is not None,
            "vision": self.camera is not None,
            # metric mapping needs a range sensor with enough coverage plus odometry
            "mapping": has_odom and self.lidar is not None,
            "mapping_experimental": has_odom and self.lidar is None and len(self.ranges) > 0,
            "navigation": has_odom and self.motors is not None and (self.lidar is not None or len(self.ranges) > 0),
            "sorting": self.servos is not None and self.camera is not None,
            "sort_verification": self.bin_sensor is not None,
            "battery_monitoring": self.battery is not None,
        }

    def shutdown(self) -> None:
        if self.motors:
            try:
                self.motors.stop()
            except Exception:
                pass
        for d in self.devices():
            try:
                d.shutdown()
            except Exception as exc:
                log.warning("shutdown of %s failed: %s", d.name, exc)


def _try(hw: HardwareSet, name: str, kind: str, enabled: bool, build: Callable[[], Device]):
    if not enabled:
        hw.unavailable.append(DeviceHealth(name, kind, HealthState.DISABLED, "disabled in config"))
        return None
    try:
        dev = build()
        dev.initialize()
        if not dev.validate():
            raise HardwareUnavailable("validation failed")
        log.info("%s (%s) ready", name, kind)
        return dev
    except HardwareUnavailable as exc:
        log.warning("%s (%s) unavailable: %s", name, kind, exc)
        hw.unavailable.append(DeviceHealth(name, kind, HealthState.MISSING, str(exc)))
    except Exception as exc:  # unexpected driver failure
        log.exception("%s (%s) failed to initialise", name, kind)
        hw.unavailable.append(DeviceHealth(name, kind, HealthState.FAULT, str(exc)))
    return None


def build_hardware(cfg: dict) -> HardwareSet:
    if cfg["robot"]["mode"] == "simulation":
        return build_simulation(cfg)
    return build_physical(cfg)


def build_physical(cfg: dict) -> HardwareSet:
    from robot.camera.cameras import OpenCVCamera, Picamera2Camera
    from robot.hardware.servos import PCA9685Servos
    from robot.motors.tb6612 import TB6612Driver
    from robot.sensors.gpio_sensors import (HCSR04, BreakBeamBinSensor, EmergencyStopButton,
                                            QuadratureEncoders)
    from robot.sensors.i2c_sensors import ADS1115Battery, MPU6050, VL53L0XSensor

    hc = cfg["hardware"]
    hw = HardwareSet(mode="hardware")

    # The e-stop is probed first: without it the motors are never enabled.
    es = hc["emergency_stop"]
    hw.estop = _try(hw, "emergency_stop", "emergency_stop", es["enabled"],
                    lambda: EmergencyStopButton("emergency_stop", es["pin"]))

    md = hc["motor_driver"]
    hw.motors = _try(hw, "motor_driver", "motor_driver", md["enabled"],
                     lambda: TB6612Driver("motor_driver", md["pins"], md["pwm_frequency_hz"],
                                          md.get("invert_left", False), md.get("invert_right", False)))
    if hw.motors and not hw.estop:
        log.error("no emergency stop input: motor driver will stay disabled")
        hw.motors.shutdown()
        hw.unavailable.append(DeviceHealth("motor_driver", "motor_driver", HealthState.DISABLED,
                                           "disabled because emergency stop input is missing"))
        hw.motors = None

    enc = hc["encoders"]
    hw.encoders = _try(hw, "encoders", "encoders", enc["enabled"],
                       lambda: QuadratureEncoders("encoders", enc["left"], enc["right"]))

    us = hc["ultrasonic"]
    for s in us.get("sensors", []):
        dev = _try(hw, f"us_{s['name']}", "ultrasonic", us["enabled"],
                   lambda s=s: HCSR04(f"us_{s['name']}", s["trigger"], s["echo"], s.get("angle_deg", 0)))
        if dev:
            hw.ranges.append(dev)

    tof = hc["tof"]
    for s in tof.get("sensors", []):
        dev = _try(hw, f"tof_{s['name']}", "tof", tof["enabled"],
                   lambda s=s: VL53L0XSensor(f"tof_{s['name']}", tof["bus"], s.get("xshut"),
                                             s["address"], s.get("angle_deg", 0)))
        if dev:
            hw.ranges.append(dev)

    imu = hc["imu"]
    hw.imu = _try(hw, "imu", "imu", imu["enabled"],
                  lambda: MPU6050("imu", imu["bus"], imu["address"]))

    bat = hc["battery"]
    hw.battery = _try(hw, "battery", "battery", bat["enabled"],
                      lambda: ADS1115Battery("battery", bat["bus"], bat["address"], bat["channel"],
                                             bat["divider_ratio"]))

    sv = hc["servos"]
    hw.servos = _try(hw, "servos", "servos", sv["enabled"],
                     lambda: PCA9685Servos("servos", sv["address"], sv["channels"]))

    bs = hc["bin_sensor"]
    hw.bin_sensor = _try(hw, "bin_sensor", "bin_sensor", bs["enabled"],
                         lambda: BreakBeamBinSensor("bin_sensor", bs["pin"]))

    ld = hc["lidar"]
    if ld["enabled"]:
        from robot.sensors.rplidar import RPLidar
        hw.lidar = _try(hw, "lidar", "lidar", True, lambda: RPLidar("lidar", ld["port"]))
    else:
        hw.unavailable.append(DeviceHealth("lidar", "lidar", HealthState.DISABLED, "disabled in config"))

    cam = cfg["camera"]
    backend = cam.get("backend", "auto")
    if backend in ("auto", "picamera2"):
        hw.camera = _try(hw, "camera", "camera", True,
                         lambda: Picamera2Camera("camera", cam["width"], cam["height"], cam["fps"]))
    if hw.camera is None and backend in ("auto", "opencv"):
        if backend == "auto":
            hw.unavailable = [u for u in hw.unavailable if u.name != "camera"]
        hw.camera = _try(hw, "camera", "camera", True,
                         lambda: OpenCVCamera("camera", cam["device"], cam["width"], cam["height"], cam["fps"]))
    return hw


def build_simulation(cfg: dict) -> HardwareSet:
    from robot.simulation.devices import (SimBattery, SimBinSensor, SimCamera, SimEmergencyStop,
                                          SimEncoders, SimImu, SimLidar, SimMotorDriver,
                                          SimRangeSensor, SimServos)
    from robot.simulation.world import SimWorld

    sc = cfg["simulation"]
    ch = cfg["chassis"]
    hc = cfg["hardware"]
    world = SimWorld(sc["world"], ch, seed=sc.get("seed", 7), sensor_noise=sc.get("sensor_noise_m", 0.01),
                     odom_noise=sc.get("odom_noise", 0.02),
                     pickup_failure_rate=sc.get("pickup_failure_rate", 0.0))
    hw = HardwareSet(mode="simulation", world=world)

    def init(dev):
        dev.initialize()
        return dev

    hw.estop = init(SimEmergencyStop("emergency_stop", world))
    hw.motors = init(SimMotorDriver("motor_driver", world))
    hw.encoders = init(SimEncoders("encoders", world, ch["wheel_radius_m"], ch["encoder_ticks_per_rev"]))
    for s in hc["ultrasonic"]["sensors"]:
        hw.ranges.append(init(SimRangeSensor(f"us_{s['name']}", world, s.get("angle_deg", 0), 2.5,
                                             "ultrasonic")))
    for s in hc["tof"]["sensors"]:
        hw.ranges.append(init(SimRangeSensor(f"tof_{s['name']}", world, s.get("angle_deg", 0), 1.2,
                                             "tof", beam_width_deg=4)))
    hw.imu = init(SimImu("imu", world))
    if hc["lidar"]["enabled"]:
        hw.lidar = init(SimLidar("lidar", world))
    else:
        hw.unavailable.append(DeviceHealth("lidar", "lidar", HealthState.DISABLED,
                                           "disabled in config", simulated=True))
    srt = cfg["sorting"]
    hw.servos = init(SimServos("servos", world, hc["servos"]["channels"], srt["gripper_closed"],
                               srt["bin_angles"]))
    cam = cfg["camera"]
    hw.camera = init(SimCamera("camera", world, cam["width"], cam["height"], cam["focal_length_px"],
                               cam["mount_height_m"], cam["mount_pitch_deg"]))
    hw.battery = init(SimBattery("battery", world))
    hw.bin_sensor = init(SimBinSensor("bin_sensor", world))
    return hw
