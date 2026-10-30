"""Extended Kalman filter fusing wheel encoders and the IMU gyro.

State: pose [x, y, theta] in the odometry frame, with covariance P.

Each cycle the motion increment is estimated from two sources:

* encoders -> distance ``ds`` and heading change ``dth_enc``. Their noise
  grows with distance travelled and with rotation (wheel slip corrupts
  rotation far more than distance);
* gyro -> heading change ``dth_gyro = w_z * dt``, noise grows with time
  (rate noise and residual bias), independent of slip.

The two heading increments are fused with inverse-variance weights, then
the pose is propagated with the unicycle model and P is propagated through
its Jacobians (standard odometry EKF prediction). Using the increments as
the control input avoids the one-step lag of a constant-velocity model.

Same interface as ``WheelOdometry`` (``update``, ``pose``, ``v``, ``w``,
``reset``) plus ``P`` and ``pose_std``.
"""

from __future__ import annotations

import math
from typing import Optional

import numpy as np

from robot.motors.kinematics import DiffDriveKinematics, Pose2D, normalize_angle


class OdometryEKF:
    def __init__(self, kin: DiffDriveKinematics, ticks_per_rev: int,
                 k_dist: float = 0.02, k_rot_from_dist: float = 0.05, k_rot: float = 0.08,
                 gyro_rate_std: float = 0.02, gyro_bias_std: float = 0.005):
        self.kin = kin
        self.m_per_tick = 2 * math.pi * kin.r / ticks_per_rev
        self.x = np.zeros(3)
        self.P = np.diag([1e-6, 1e-6, 1e-6])
        self.k_dist = k_dist                    # distance std per metre travelled
        self.k_rot_from_dist = k_rot_from_dist  # heading std (rad) per metre travelled
        self.k_rot = k_rot                      # heading std per radian turned (slip)
        self.gyro_rate_std = gyro_rate_std      # rad/s noise per sample
        self.gyro_bias_std = gyro_bias_std      # rad/s residual bias after start-up calibration
        self.v = 0.0
        self.w = 0.0
        self.gyro_weight = 0.0                  # last fusion weight given to the gyro (diagnostics)
        self._last_ticks: Optional[tuple[int, int]] = None
        self._last_t: Optional[float] = None
        self.updates = 0

    @property
    def pose(self) -> Pose2D:
        return Pose2D(float(self.x[0]), float(self.x[1]), float(self.x[2]))

    def pose_std(self) -> tuple[float, float]:
        """(position std in m, heading std in rad)."""
        return (math.sqrt(max(0.0, self.P[0, 0] + self.P[1, 1])), math.sqrt(max(0.0, self.P[2, 2])))

    def reset(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0) -> None:
        self.x[:] = [x, y, theta]
        self.P = np.diag([1e-6, 1e-6, 1e-6])

    def update(self, ticks: tuple[int, int], t: float, gyro_z: Optional[float] = None) -> Pose2D:
        if self._last_ticks is None or self._last_t is None:
            self._last_ticks, self._last_t = ticks, t
            return self.pose
        dt = t - self._last_t
        if dt <= 1e-4:
            return self.pose
        dl = (ticks[0] - self._last_ticks[0]) * self.m_per_tick
        dr = (ticks[1] - self._last_ticks[1]) * self.m_per_tick
        self._last_ticks, self._last_t = ticks, t

        ds = (dl + dr) / 2.0
        dth_enc = (dr - dl) / self.kin.L
        var_ds = (self.k_dist * abs(ds)) ** 2 + 1e-8
        var_enc = (self.k_rot_from_dist * abs(ds) + self.k_rot * abs(dth_enc)) ** 2 + 1e-8
        if gyro_z is not None and math.isfinite(gyro_z):
            dth_g = gyro_z * dt
            var_g = ((self.gyro_rate_std + self.gyro_bias_std) * dt) ** 2
            self.gyro_weight = var_enc / (var_enc + var_g)
            dth = dth_enc + self.gyro_weight * (dth_g - dth_enc)
            var_dth = var_enc * var_g / (var_enc + var_g)
        else:
            self.gyro_weight = 0.0
            dth, var_dth = dth_enc, var_enc

        th = self.x[2]
        mid = th + dth / 2.0
        c, s = math.cos(mid), math.sin(mid)
        self.x[0] += ds * c
        self.x[1] += ds * s
        self.x[2] = normalize_angle(th + dth)

        # Jacobians of the motion model wrt state and control (ds, dth)
        F = np.array([[1.0, 0.0, -ds * s],
                      [0.0, 1.0, ds * c],
                      [0.0, 0.0, 1.0]])
        G = np.array([[c, -0.5 * ds * s],
                      [s, 0.5 * ds * c],
                      [0.0, 1.0]])
        Q = np.diag([var_ds, var_dth])
        self.P = F @ self.P @ F.T + G @ Q @ G.T
        self.v = ds / dt
        self.w = dth / dt
        self.updates += 1
        return self.pose
