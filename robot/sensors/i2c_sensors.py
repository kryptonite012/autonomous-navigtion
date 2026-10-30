"""I2C sensors: VL53L0X time-of-flight, MPU6050 IMU and ADS1115 battery ADC."""

from __future__ import annotations

import logging
import math
import struct
import time
from typing import Optional

from robot.hardware.base import (BatteryMonitor, DistanceSensor, HardwareUnavailable,
                                 Imu, ImuReading, RangeReading)

log = logging.getLogger(__name__)

G = 9.80665


def _smbus(bus: int):
    try:
        from smbus2 import SMBus
    except ImportError as exc:
        raise HardwareUnavailable("smbus2 not installed") from exc
    try:
        return SMBus(bus)
    except (FileNotFoundError, PermissionError) as exc:
        raise HardwareUnavailable(f"I2C bus {bus} not available (enable it with raspi-config)") from exc


class MPU6050(Imu):
    PWR_MGMT_1 = 0x6B
    WHO_AM_I = 0x75
    ACCEL_XOUT_H = 0x3B
    GYRO_CONFIG = 0x1B
    ACCEL_CONFIG = 0x1C

    def __init__(self, name: str, bus: int = 1, address: int = 0x68):
        super().__init__(name)
        self.bus_id = bus
        self.address = address
        self._bus = None
        self.gyro_bias = (0.0, 0.0, 0.0)

    def initialize(self) -> None:
        self._bus = _smbus(self.bus_id)
        try:
            who = self._bus.read_byte_data(self.address, self.WHO_AM_I)
        except OSError as exc:
            self._record_error(exc)
            raise HardwareUnavailable(f"MPU6050 not found at 0x{self.address:02x}") from exc
        if who not in (0x68, 0x70, 0x72):  # MPU6050 / 6500 / clones
            raise HardwareUnavailable(f"unexpected WHO_AM_I 0x{who:02x}")
        self._bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x01)  # wake, PLL gyro x
        self._bus.write_byte_data(self.address, self.GYRO_CONFIG, 0x00)  # +-250 deg/s
        self._bus.write_byte_data(self.address, self.ACCEL_CONFIG, 0x00)  # +-2 g
        time.sleep(0.05)
        super().initialize()
        self.calibrate_gyro()

    def calibrate_gyro(self, samples: int = 200) -> None:
        """Average the gyro output while the robot is stationary."""
        sx = sy = sz = 0.0
        for _ in range(samples):
            _, g, _ = self._raw()
            sx, sy, sz = sx + g[0], sy + g[1], sz + g[2]
            time.sleep(0.002)
        self.gyro_bias = (sx / samples, sy / samples, sz / samples)

    def _raw(self):
        data = bytes(self._bus.read_i2c_block_data(self.address, self.ACCEL_XOUT_H, 14))
        ax, ay, az, temp, gx, gy, gz = struct.unpack(">hhhhhhh", data)
        accel = (ax / 16384.0 * G, ay / 16384.0 * G, az / 16384.0 * G)
        gyro = tuple(math.radians(v / 131.0) for v in (gx, gy, gz))
        return accel, gyro, temp / 340.0 + 36.53

    def read(self) -> ImuReading:
        try:
            accel, gyro, temp = self._raw()
            self._record_ok()
        except OSError as exc:
            self._record_error(exc)
            raise
        gyro = tuple(g - b for g, b in zip(gyro, self.gyro_bias))
        return ImuReading(accel, gyro, temp, time.time())

    def shutdown(self) -> None:
        if self._bus:
            self._bus.close()
        super().shutdown()


class VL53L0XSensor(DistanceSensor):
    """VL53L0X via the Adafruit CircuitPython driver.

    Several sensors share address 0x29. Each one has its XSHUT pin held low
    at boot; we release them one by one and assign a new address.
    """

    kind = "tof"
    max_range_m = 1.2

    def __init__(self, name: str, bus: int, xshut: Optional[int], address: int, angle_deg: float = 0.0):
        super().__init__(name)
        self.bus_id = bus
        self.xshut = xshut
        self.address = address
        self.angle_rad = math.radians(angle_deg)
        self._sensor = None
        self._xshut_dev = None

    def initialize(self) -> None:
        try:
            import adafruit_vl53l0x
            import board
            import busio
        except ImportError as exc:
            raise HardwareUnavailable("adafruit-circuitpython-vl53l0x not installed") from exc
        try:
            if self.xshut is not None:
                from gpiozero import DigitalOutputDevice
                self._xshut_dev = DigitalOutputDevice(self.xshut, initial_value=True)
                time.sleep(0.01)
            i2c = busio.I2C(board.SCL, board.SDA)
            try:
                self._sensor = adafruit_vl53l0x.VL53L0X(i2c, address=self.address)
            except (ValueError, OSError):
                self._sensor = adafruit_vl53l0x.VL53L0X(i2c)
                if self.address != 0x29:
                    self._sensor.set_address(self.address)
            self._sensor.measurement_timing_budget = 33000
        except Exception as exc:
            self._record_error(exc)
            raise HardwareUnavailable(f"{self.name}: {exc}") from exc
        super().initialize()

    def read(self) -> RangeReading:
        now = time.time()
        try:
            mm = self._sensor.range
            self._record_ok()
        except Exception as exc:
            self._record_error(exc)
            return RangeReading(self.name, None, self.angle_rad, now, False, self.max_range_m)
        if mm <= 0 or mm >= 8000 or mm / 1000.0 > self.max_range_m:
            return RangeReading(self.name, None, self.angle_rad, now, True, self.max_range_m)
        return RangeReading(self.name, mm / 1000.0, self.angle_rad, now, True, self.max_range_m)

    def shutdown(self) -> None:
        if self._xshut_dev:
            self._xshut_dev.close()
        super().shutdown()


class ADS1115Battery(BatteryMonitor):
    """Battery voltage via an ADS1115 behind a resistor divider."""

    REG_CONV = 0x00
    REG_CFG = 0x01

    def __init__(self, name: str, bus: int, address: int, channel: int, divider_ratio: float):
        super().__init__(name)
        self.bus_id = bus
        self.address = address
        self.channel = channel
        self.ratio = divider_ratio
        self._bus = None

    def initialize(self) -> None:
        self._bus = _smbus(self.bus_id)
        try:
            self._bus.read_i2c_block_data(self.address, self.REG_CFG, 2)
        except OSError as exc:
            self._record_error(exc)
            raise HardwareUnavailable(f"ADS1115 not found at 0x{self.address:02x}") from exc
        super().initialize()

    def voltage(self) -> Optional[float]:
        mux = 0x4 + self.channel                # single ended AINx vs GND
        cfg = (1 << 15) | (mux << 12) | (0x1 << 9) | (1 << 8) | (0x4 << 5) | 0x3  # +-4.096V, 128SPS
        try:
            self._bus.write_i2c_block_data(self.address, self.REG_CFG, [cfg >> 8, cfg & 0xFF])
            time.sleep(0.01)
            hi, lo = self._bus.read_i2c_block_data(self.address, self.REG_CONV, 2)
            self._record_ok()
        except OSError as exc:
            self._record_error(exc)
            return None
        raw = struct.unpack(">h", bytes([hi, lo]))[0]
        return raw * 4.096 / 32768.0 * self.ratio

    def shutdown(self) -> None:
        if self._bus:
            self._bus.close()
        super().shutdown()
