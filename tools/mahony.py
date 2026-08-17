"""
Mahony AHRS — Python replica of user_app/src/mahony.c

PI-controlled complementary filter on SO(3).
Line-by-line correspondence with the C implementation.

Usage:
    ahrs = MahonyAHRS(kp=1.0, ki=0.1)
    roll, pitch, yaw = ahrs.update(ax, ay, az, gx, gy, gz, dt)
"""

import math
import numpy as np

DEG2RAD = 0.017453292519943295
RAD2DEG = 57.29577951308232


class MahonyAHRS:
    """Mahony 6-DOF AHRS with configurable Kp, Ki."""

    def __init__(self, kp=1.0, ki=0.1, kp_min=0.1, kp_thresh=0.3):
        """
        Args:
            kp: Proportional gain (KP_NOMINAL in C). Higher = faster convergence, more noise.
            ki: Integral gain for gyro bias estimation.
            kp_min: Absolute floor on adaptive confidence factor (same as C KP_MIN).
            kp_thresh: |a| deviation from 1g that drops confidence to 0 (in g).
        """
        self.kp = float(kp)
        self.ki = float(ki)
        self.kp_min = float(kp_min)
        self.kp_thresh = float(kp_thresh)
        self.reset()

    def reset(self):
        """Reset internal state (same as mahony_init)."""
        self.q0, self.q1, self.q2, self.q3 = 1.0, 0.0, 0.0, 0.0
        self.ix, self.iy, self.iz = 0.0, 0.0, 0.0
        self.initialized = False

    def _init_from_accel(self, ax, ay, az):
        """Same as mahony_init_from_accel in C."""
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
        Single Mahony update step.  Exactly matches mahony_update() in mahony.c.

        Args:
            ax, ay, az: Accelerometer in g (calibrated).
            gx, gy, gz: Gyroscope in °/s (bias-corrected).
            dt: Time step in seconds.

        Returns:
            dict with keys: roll, pitch, yaw (deg), q0, q1, q2, q3, bx, by, bz
        """
        if not self.initialized:
            self._init_from_accel(ax, ay, az)
            return self._output()

        # ── Normalize accelerometer (lines 70-72 in C) ──
        a_norm = math.sqrt(ax * ax + ay * ay + az * az)
        recip = 1.0 / a_norm
        ax *= recip
        ay *= recip
        az *= recip

        # ── Adaptive Kp (lines 75-79 in C) ──
        dev = abs(a_norm - 1.0)
        conf = 1.0 - dev / self.kp_thresh
        if conf < self.kp_min:
            conf = self.kp_min
        if conf > 1.0:
            conf = 1.0
        Kp = self.kp * conf

        # ── Estimated gravity direction from quaternion (lines 81-83 in C) ──
        q0, q1, q2, q3 = self.q0, self.q1, self.q2, self.q3
        vx = 2.0 * (q1 * q3 - q0 * q2)
        vy = 2.0 * (q0 * q1 + q2 * q3)
        vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3

        # ── Cross-product error (lines 85-87 in C) ──
        ex = ay * vz - az * vy
        ey = az * vx - ax * vz
        ez = ax * vy - ay * vx

        # ── Integral term with clamping (lines 89-99 in C) ──
        if self.ki > 0.0:
            self.ix += self.ki * ex * dt
            self.iy += self.ki * ey * dt
            self.iz += self.ki * ez * dt
            if self.ix > 20.0:
                self.ix = 20.0
            if self.ix < -20.0:
                self.ix = -20.0
            if self.iy > 20.0:
                self.iy = 20.0
            if self.iy < -20.0:
                self.iy = -20.0
            if self.iz > 20.0:
                self.iz = 20.0
            if self.iz < -20.0:
                self.iz = -20.0

        # ── Corrected angular velocity (lines 101-103 in C) ──
        wx = gx * DEG2RAD + Kp * ex + self.ix
        wy = gy * DEG2RAD + Kp * ey + self.iy
        wz = gz * DEG2RAD + Kp * ez + self.iz

        # ── Quaternion derivative: 0.5 * q ⊗ ω (lines 105-108 in C) ──
        hw = 0.5 * (-q1 * wx - q2 * wy - q3 * wz)
        hx = 0.5 * (q0 * wx + q2 * wz - q3 * wy)
        hy = 0.5 * (q0 * wy - q1 * wz + q3 * wx)
        hz = 0.5 * (q0 * wz + q1 * wy - q2 * wx)

        # ── Integrate quaternion (line 110 in C) ──
        self.q0 += hw * dt
        self.q1 += hx * dt
        self.q2 += hy * dt
        self.q3 += hz * dt

        # ── Normalize quaternion (lines 113-116 in C) ──
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
        """Same Euler-angle computation as lines 119-124 in C."""
        q0, q1, q2, q3 = self.q0, self.q1, self.q2, self.q3
        return {
            "roll": math.atan2(2.0 * (q0 * q1 + q2 * q3),
                               1.0 - 2.0 * (q1 * q1 + q2 * q2)) * RAD2DEG,
            "pitch": math.asin(2.0 * (q0 * q2 - q3 * q1)) * RAD2DEG,
            "yaw": math.atan2(2.0 * (q0 * q3 + q1 * q2),
                              1.0 - 2.0 * (q2 * q2 + q3 * q3)) * RAD2DEG,
            "q0": self.q0, "q1": self.q1, "q2": self.q2, "q3": self.q3,
            "bx": self.ix, "by": self.iy, "bz": self.iz,
        }

    def run_dataset(self, data, progress=False):
        """
        Run Mahony on a full dataset (NumPy array).

        Args:
            data: ndarray of shape (N, 6 or 7) with columns [ax, ay, az, gx, gy, gz, (dt)].
                  If dt column missing, it's computed from consecutive timestamps.
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
            elif i == 0:
                dt = 0.001
            else:
                dt = 0.001  # constant-rate assumption

            out = self.update(ax, ay, az, gx, gy, gz, dt)
            result[i, 0] = out["roll"]
            result[i, 1] = out["pitch"]
            result[i, 2] = out["yaw"]

            if progress and i % 10000 == 0 and i > 0:
                print(f"  mahony: {i}/{N}")

        return result
