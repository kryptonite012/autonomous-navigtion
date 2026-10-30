"""TB6612FNG dual H-bridge driver using gpiozero (lgpio backend on the Pi 5).

Wiring is documented in docs/hardware/wiring.md. The driver's VM pin is fed
from the motor battery, VCC from the Pi 3.3 V rail, grounds are common.
"""

from __future__ import annotations

import logging

from robot.hardware.base import HardwareUnavailable, MotorDriver

log = logging.getLogger(__name__)


def _clamp(v: float) -> float:
    return max(-1.0, min(1.0, float(v)))


class TB6612Driver(MotorDriver):
    def __init__(self, name: str, pins: dict, pwm_frequency_hz: int = 1000,
                 invert_left: bool = False, invert_right: bool = False):
        super().__init__(name)
        self.pins = pins
        self.freq = pwm_frequency_hz
        self.invert_left = invert_left
        self.invert_right = invert_right
        self._left = None
        self._right = None
        self._stby = None

    def initialize(self) -> None:
        try:
            from gpiozero import DigitalOutputDevice, Motor
        except ImportError as exc:
            self._record_error("gpiozero not installed")
            raise HardwareUnavailable("gpiozero not installed") from exc
        p = self.pins
        try:
            self._left = Motor(forward=p["ain1"], backward=p["ain2"], enable=p["pwma"], pwm=True)
            self._right = Motor(forward=p["bin1"], backward=p["bin2"], enable=p["pwmb"], pwm=True)
            self._stby = DigitalOutputDevice(p["stby"], initial_value=False)
        except Exception as exc:  # BadPinFactory, GPIOPinInUse, ...
            self._record_error(exc)
            raise HardwareUnavailable(f"cannot open TB6612 pins: {exc}") from exc
        for m in (self._left, self._right):
            try:
                m.enable_device.frequency = self.freq
            except Exception:
                pass
        self._stby.on()
        super().initialize()
        log.info("TB6612 motor driver ready")

    @staticmethod
    def _apply(motor, value: float) -> None:
        if value > 0:
            motor.forward(value)
        elif value < 0:
            motor.backward(-value)
        else:
            motor.stop()

    def set_duty(self, left: float, right: float) -> None:
        if not self._initialized:
            return
        left, right = _clamp(left), _clamp(right)
        if self.invert_left:
            left = -left
        if self.invert_right:
            right = -right
        try:
            self._apply(self._left, left)
            self._apply(self._right, right)
            self._record_ok()
        except Exception as exc:
            self._record_error(exc)
            self.stop()
            raise

    def stop(self) -> None:
        for m in (self._left, self._right):
            if m is not None:
                try:
                    m.stop()
                except Exception as exc:
                    self._record_error(exc)

    def validate(self) -> bool:
        # The TB6612 has no readback. We can only verify that the pins are
        # held; real motion is confirmed by the encoders (see safety monitor).
        return self._initialized and self._stby is not None and self._stby.value == 1

    def shutdown(self) -> None:
        self.stop()
        if self._stby is not None:
            self._stby.off()
        for dev in (self._left, self._right, self._stby):
            if dev is not None:
                dev.close()
        super().shutdown()
