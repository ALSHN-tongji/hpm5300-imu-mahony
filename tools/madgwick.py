"""
Madgwick AHRS — Python replica of user_app/src/madgwick.c

Gradient-descent orientation filter (6-DOF, no magnetometer).
Line-by-line correspondence with the C implementation.

Usage:
    ahrs = MadgwickAHRS(beta=0.05)
    roll, pitch, yaw = ahrs.update(ax, ay, az, gx, gy, gz, dt)
"""

import math
import numpy as np

DEG2RAD = 0.017453292519943295
RAD2DEG = 57.29577951308232


class MadgwickAHRS:
    """Madgwick 6-DOF AHRS with configurable beta."""

    def __init__(self, beta=0.05):
        """
        Args:
            beta: Gradient descent step size.
                  Larger = faster convergence, more noise.
                  Smaller = smoother, slower response.
                  Typical range: 0.01 ~ 0.5.
        """
        self.beta = float(beta)
        self.reset()

    def reset(self):
        """Reset internal state (same as madgwick_init)."""
        self.q0, self.q1, self.q2, self.q3 = 1.0, 0.0, 0.0, 0.0
        self.initialized = False

    def _init_from_accel(self, ax, ay, az):
        """Same as init_from_accel in C."""
        n = 1.0 / math.sqrt(ax * ax + ay * ay + az * az)
        ax *= n
        ay *= n
        az *= n

        roll = math.atan2(ay, az)
        pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))

        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)

        self.q0 = cr * cp
        self.q1 = sr * cp
        self.q2 = cr * sp
        self.q3 = 0.0
        self.initialized = True

    def update(self, ax, ay, az, gx, gy, gz, dt):
        """
        Single Madgwick update step.  Exactly matches madgwick_update() in
        madgwick.c.

        Args:
            ax, ay, az: Accelerometer in g (calibrated).
            gx, gy, gz: Gyroscope in °/s (bias-corrected).
            dt: Time step in seconds.

        Returns:
            dict with keys: roll, pitch, yaw (deg), q0, q1, q2, q3
        """
        if not self.initialized:
            self._init_from_accel(ax, ay, az)
            return self._output()

        # ── Normalize accelerometer (line 56 in C) ──
        recip = 1.0 / math.sqrt(ax * ax + ay * ay + az * az)
        ax *= recip
        ay *= recip
        az *= recip

        # ── Objective function f = g_est(q) - a_meas (lines 65-67 in C) ──
        q0, q1, q2, q3 = self.q0, self.q1, self.q2, self.q3
        f1 = 2.0 * (q1 * q3 - q0 * q2) - ax
        f2 = 2.0 * (q0 * q1 + q2 * q3) - ay
        f3 = (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) - az

        # ── Gradient ∇f = J^T * f (lines 81-84 in C) ──
        g0 = -2.0 * q2 * f1 + 2.0 * q1 * f2 + 2.0 * q0 * f3
        g1 = 2.0 * q3 * f1 + 2.0 * q0 * f2 - 2.0 * q1 * f3
        g2 = -2.0 * q0 * f1 + 2.0 * q3 * f2 - 2.0 * q2 * f3
        g3 = 2.0 * q1 * f1 + 2.0 * q2 * f2

        # ── Normalize gradient (lines 87-91 in C) ──
        g_norm = math.sqrt(g0 * g0 + g1 * g1 + g2 * g2 + g3 * g3)
        if g_norm > 1e-9:
            recip = 1.0 / g_norm
            g0 *= recip
            g1 *= recip
            g2 *= recip
            g3 *= recip

        # ── Quaternion derivative from gyroscope (lines 94-97 in C) ──
        # Note: DEG2RAD applied here, unlike Mahony where it's on wx/wy/wz.
        wq0 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz) * DEG2RAD
        wq1 = 0.5 * (q0 * gx + q2 * gz - q3 * gy) * DEG2RAD
        wq2 = 0.5 * (q0 * gy - q1 * gz + q3 * gx) * DEG2RAD
        wq3 = 0.5 * (q0 * gz + q1 * gy - q2 * gx) * DEG2RAD

        # ── Fused derivative: q_dot = wq - β * ∇f/|∇f| (line 100 in C) ──
        self.q0 += (wq0 - self.beta * g0) * dt
        self.q1 += (wq1 - self.beta * g1) * dt
        self.q2 += (wq2 - self.beta * g2) * dt
        self.q3 += (wq3 - self.beta * g3) * dt

        # ── Normalize quaternion (lines 106-108 in C) ──
        recip = 1.0 / math.sqrt(
            self.q0 * self.q0
            + self.q1 * self.q1
            + self.q2 * self.q2
            + self.q3 * self.q3
        )
        self.q0 *= recip
        self.q1 *= recip
        self.q2 *= recip
        self.q3 *= recip
        if self.q0 < 0:
            self.q0 = -self.q0
            self.q1 = -self.q1
            self.q2 = -self.q2
            self.q3 = -self.q3

        return self._output()

    def _output(self):
        """Same Euler-angle computation as lines 112-114 in C."""
        q0, q1, q2, q3 = self.q0, self.q1, self.q2, self.q3
        return {
            "roll": math.atan2(2.0 * (q0 * q1 + q2 * q3),
                               1.0 - 2.0 * (q1 * q1 + q2 * q2)) * RAD2DEG,
            "pitch": math.asin(2.0 * (q0 * q2 - q3 * q1)) * RAD2DEG,
            "yaw": math.atan2(2.0 * (q0 * q3 + q1 * q2),
                              1.0 - 2.0 * (q2 * q2 + q3 * q3)) * RAD2DEG,
            "q0": self.q0, "q1": self.q1, "q2": self.q2, "q3": self.q3,
        }

    def run_dataset(self, data, progress=False):
        """
        Run Madgwick on a full dataset (NumPy array).

        Args:
            data: ndarray of shape (N, 6 or 7) with columns [ax, ay, az, gx, gy, gz, (dt)].
            progress: If True, print progress every 10000 samples.

        Returns:
            ndarray of shape (N, 3) with columns [roll, pitch, yaw].
        """
        self.reset()
        N = data.shape[0]
        result = np.empty((N, 3), dtype=np.float32)

        for i in range(N):
            ax, ay, az = float(data[i, 0]), float(data[i, 1]), float(data[i, 2])
            gx, gy, gz = float(data[i, 3]), float(data[i, 4]), float(data[i, 5])
            if data.shape[1] >= 7:
                dt = float(data[i, 6])
            else:
                dt = 0.001

            out = self.update(ax, ay, az, gx, gy, gz, dt)
            result[i, 0] = out["roll"]
            result[i, 1] = out["pitch"]
            result[i, 2] = out["yaw"]

            if progress and i % 10000 == 0 and i > 0:
                print(f"  madgwick: {i}/{N}")

        return result
