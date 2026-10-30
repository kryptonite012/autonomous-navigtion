"""GPIO based sensors: quadrature encoders, HC-SR04, e-stop and bin beam."""

from __future__ import annotations

import logging
import math
import threading
import time

from robot.hardware.base import (BinSensor, DistanceSensor, EmergencyStopInput,
                                 EncoderPair, HardwareUnavailable, RangeReading)

log = logging.getLogger(__name__)


def _gpiozero():
    try:
        import gpiozero
        return gpiozero
    except ImportError as exc:
        raise HardwareUnavailable("gpiozero not installed") from exc


class QuadratureEncoders(EncoderPair):
    def __init__(self, name: str, left: dict, right: dict):
        super().__init__(name)
        self.left_pins = left
        self.right_pins = right
        self._enc = []

    def initialize(self) -> None:
        gz = _gpiozero()
        try:
            # max_steps=0 -> unbounded counter
            self._enc = [
                gz.RotaryEncoder(self.left_pins["a"], self.left_pins["b"], max_steps=0),
                gz.RotaryEncoder(self.right_pins["a"], self.right_pins["b"], max_steps=0),
            ]
        except Exception as exc:
            self._record_error(exc)
            raise HardwareUnavailable(f"encoders: {exc}") from exc
        super().initialize()

    def ticks(self) -> tuple[int, int]:
        return self._enc[0].steps, self._enc[1].steps

    def shutdown(self) -> None:
        for e in self._enc:
            e.close()
        super().shutdown()


class HCSR04(DistanceSensor):
    """HC-SR04 ultrasonic ranger.

    ECHO is a 5 V signal: it must go through a 1k/2k divider (or a level
    shifter) before reaching the Pi. See docs/hardware/wiring.md.
    """

    kind = "ultrasonic"
    max_range_m = 2.5

    def __init__(self, name: str, trigger: int, echo: int, angle_deg: float = 0.0):
        super().__init__(name)
        self.trigger = trigger
        self.echo = echo
        self.angle_rad = math.radians(angle_deg)
        self._dev = None
        self._lock = threading.Lock()

    def initialize(self) -> None:
        gz = _gpiozero()
        try:
            self._dev = gz.DistanceSensor(echo=self.echo, trigger=self.trigger,
                                          max_distance=self.max_range_m, queue_len=3)
        except Exception as exc:
            self._record_error(exc)
            raise HardwareUnavailable(f"{self.name}: {exc}") from exc
        super().initialize()

    def validate(self) -> bool:
        # A disconnected echo line produces no pulse, gpiozero then reports
        # max range forever. Take a few samples; all-max means suspicious but
        # could also be an open room, so we only report it as degraded later.
        if not self._initialized:
            return False
        r = self.read()
        return r.valid

    def read(self) -> RangeReading:
        now = time.time()
        if not self._initialized:
            return RangeReading(self.name, None, self.angle_rad, now, False, self.max_range_m)
        try:
            with self._lock:
                d = float(self._dev.distance)
            self._record_ok()
        except Exception as exc:
            self._record_error(exc)
            return RangeReading(self.name, None, self.angle_rad, now, False, self.max_range_m)
        if d >= self.max_range_m * 0.99:
            return RangeReading(self.name, None, self.angle_rad, now, True, self.max_range_m)
        return RangeReading(self.name, d, self.angle_rad, now, True, self.max_range_m)

    def shutdown(self) -> None:
        if self._dev:
            self._dev.close()
        super().shutdown()


class EmergencyStopButton(EmergencyStopInput):
    """Normally-closed e-stop contact wired between the GPIO and ground.

    Pressed (or a cut wire) opens the contact, the pull-up drives the pin
    high and we report pressed. The same switch also opens the motor relay
    so stopping does not depend on this software.
    """

    def __init__(self, name: str, pin: int):
        super().__init__(name)
        self.pin = pin
        self._btn = None

    def initialize(self) -> None:
        gz = _gpiozero()
        try:
            self._btn = gz.DigitalInputDevice(self.pin, pull_up=True, bounce_time=0.01)
        except Exception as exc:
            self._record_error(exc)
            raise HardwareUnavailable(f"e-stop: {exc}") from exc
        super().initialize()

    def is_pressed(self) -> bool:
        if not self._initialized:
            return True  # fail safe
        # pull-up: value 1 means the NC contact is open -> pressed
        return bool(self._btn.value)

    def shutdown(self) -> None:
        if self._btn:
            self._btn.close()
        super().shutdown()


class BreakBeamBinSensor(BinSensor):
    """IR break-beam across the bin chute, output is open collector, active low."""

    def __init__(self, name: str, pin: int):
        super().__init__(name)
        self.pin = pin
        self._dev = None
        self._count = 0
        self._lock = threading.Lock()

    def initialize(self) -> None:
        gz = _gpiozero()
        try:
            self._dev = gz.DigitalInputDevice(self.pin, pull_up=True, bounce_time=0.005)
            self._dev.when_deactivated = self._on_break
        except Exception as exc:
            self._record_error(exc)
            raise HardwareUnavailable(f"bin sensor: {exc}") from exc
        super().initialize()

    def _on_break(self) -> None:
        with self._lock:
            self._count += 1

    def event_count(self) -> int:
        with self._lock:
            return self._count

    def shutdown(self) -> None:
        if self._dev:
            self._dev.close()
        super().shutdown()
