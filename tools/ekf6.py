"""
6-axis Error-State Kalman Filter (ES-EKF) for IMU Attitude
============================================================
Noise parameters Q/R derived from measured Allan variance (2026-08-07),
not copied from textbook constants.

State (error-state, 6-D):  δθ(3) + δb(3)
  - δθ = attitude error (rad), corrects quaternion
  - δb  = gyro bias error (rad/s), corrects estimated bias

Noise sources (from Allan on this hardware):
  - ARW = 0.41 °/√hr  →  gyro white noise σ_g
  - BI  = 6.8  °/hr   →  bias random walk σ_bw
  - Accel PSD floor    →  measurement noise σ_a

Usage:
    ekf = EKF6(arw_deg_per_sqrt_hr=0.41, bi_deg_per_hr=6.8)
    roll, pitch, yaw = ekf.update(ax, ay, az, gx, gy, gz, dt)
"""

import math
import numpy as np

DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi


class EKF6:
    """6-axis error-state EKF (no magnetometer).

    Uses measured Allan parameters for Q/R — no hand-tuned constants.
    Optionally uses adaptive R: scales measurement noise by |a| deviation
    from 1g (equivalent to Mahony's adaptive Kp).
    """

    def __init__(self, arw_deg_per_sqrt_hr=0.41, bi_deg_per_hr=6.8,
                 accel_noise_mg=0.3, bias_corr_time_s=500.0,
                 adaptive_r=True, r_thresh=0.3, r_max_scale=100.0):
        """
        Args:
            arw_deg_per_sqrt_hr: Angle Random Walk from Allan (°/√hr).
            bi_deg_per_hr:       Bias Instability from Allan (°/hr).
            accel_noise_mg:      Accel white noise std (mg).
            bias_corr_time_s:    Bias correlation time (s).
            adaptive_r:          Enable adaptive measurement noise.
            r_thresh:            |a| deviation from 1g that triggers
                                 max R scaling (g), same as KP_THRESH.
            r_max_scale:         Maximum R multiplier at large |a| dev.
        """
        # ── Convert Allan parameters to filter noise densities ──
        arw_rad_per_sqrt_s = arw_deg_per_sqrt_hr * DEG2RAD / 60.0
        self.sigma_g = arw_rad_per_sqrt_s  # rad/√s

        bi_rad_per_s = bi_deg_per_hr * DEG2RAD / 3600.0
        tau = max(bias_corr_time_s, 10.0)
        self.sigma_bw = bi_rad_per_s * math.sqrt(3.0 / tau)

        self.sigma_a = accel_noise_mg * 0.001  # mg → g
        self.sigma_a_base = self.sigma_a

        # Adaptive R params
        self.adaptive_r = adaptive_r
        self.r_thresh = r_thresh
        self.r_max_scale = r_max_scale
        self.r_scale_smooth = 1.0  # smoothed R multiplier

        self.arw_used = arw_deg_per_sqrt_hr
        self.bi_used = bi_deg_per_hr
        self.bi_tau = tau

        self.reset()

    def reset(self):
        """Reset state and covariance."""
        # Nominal state
        self.q = np.array([1.0, 0.0, 0.0, 0.0])  # q0, q1, q2, q3
        self.bias = np.zeros(3)  # estimated gyro bias (rad/s)

        # Error state (always reset to zero after injection)
        self.dx = np.zeros(6)

        # Covariance
        self.P = np.eye(6) * 0.01
        # Attitude uncertainty: ~0.1 rad initially
        self.P[0, 0] = self.P[1, 1] = self.P[2, 2] = 0.01
        # Bias uncertainty: ~BI
        bi_init = self.bi_used * DEG2RAD / 3600.0
        self.P[3, 3] = self.P[4, 4] = self.P[5, 5] = bi_init ** 2

        self.initialized = False

    def _init_from_accel(self, ax, ay, az):
        """Initialize attitude from accelerometer (same as Mahony/Madgwick)."""
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

        self.q[0] = cr * cp
        self.q[1] = sr * cp
        self.q[2] = cr * sp
        self.q[3] = 0.0
        self.bias[:] = 0.0
        self.initialized = True

    def update(self, ax, ay, az, gx, gy, gz, dt):
        """Single EKF update step.

        Returns dict with roll, pitch, yaw (deg), q, bias.
        """
        if not self.initialized:
            self._init_from_accel(ax, ay, az)
            return self._output()

        q0, q1, q2, q3 = self.q
        bx, by, bz = self.bias

        # ── Gyro in rad/s, bias-corrected ──
        wx = gx * DEG2RAD - bx
        wy = gy * DEG2RAD - by
        wz = gz * DEG2RAD - bz

        # ═══════════════════════════════════════════════════════
        # 1. PROPAGATE nominal quaternion
        # ═══════════════════════════════════════════════════════
        hw = 0.5 * (-q1 * wx - q2 * wy - q3 * wz) * dt
        hx = 0.5 * (q0 * wx + q2 * wz - q3 * wy) * dt
        hy = 0.5 * (q0 * wy - q1 * wz + q3 * wx) * dt
        hz = 0.5 * (q0 * wz + q1 * wy - q2 * wx) * dt

        q0_new = q0 + hw
        q1_new = q1 + hx
        q2_new = q2 + hy
        q3_new = q3 + hz

        n = 1.0 / math.sqrt(q0_new ** 2 + q1_new ** 2 + q2_new ** 2 + q3_new ** 2)
        self.q[0] = q0_new * n
        self.q[1] = q1_new * n
        self.q[2] = q2_new * n
        self.q[3] = q3_new * n

        # ═══════════════════════════════════════════════════════
        # 2. PROPAGATE error-state covariance
        # ═══════════════════════════════════════════════════════
        F = np.eye(6)
        # Attitude error coupling: δθ̇ = -[ω×]δθ - δb
        F[0, 1] = wz * dt
        F[0, 2] = -wy * dt
        F[1, 0] = -wz * dt
        F[1, 2] = wx * dt
        F[2, 0] = wy * dt
        F[2, 1] = -wx * dt
        F[0, 3] = -dt
        F[1, 4] = -dt
        F[2, 5] = -dt

        # ═══════════════════════════════════════════════════════
        # Motion detection (shared by Q/R adaptation)
        # ═══════════════════════════════════════════════════════
        a_norm = math.sqrt(ax * ax + ay * ay + az * az)
        dev = abs(a_norm - 1.0)

        # motion_factor: 1.0=static → normal Q/R,  0.0=motion → frozen bias / high R
        # Equivalent of Mahony's adaptive Kp confidence
        raw_conf = 1.0 - dev / self.r_thresh
        if raw_conf < 0.0:
            raw_conf = 0.0
        if raw_conf > 1.0:
            raw_conf = 1.0
        # Smooth (EMA, τ=150ms)
        alpha = dt / (0.15 + dt)
        self.r_scale_smooth += alpha * (raw_conf - self.r_scale_smooth)
        motion = self.r_scale_smooth  # 1=static, 0=motion

        # ── Q: freeze bias update during motion ──
        # bias random walk scaled by motion factor (static=full, motion≈0)
        sg2_dt = self.sigma_g ** 2 * dt
        sb2_dt = self.sigma_bw ** 2 * dt * motion  # ← bias frozen in motion
        Q = np.zeros((6, 6))
        Q[0, 0] = Q[1, 1] = Q[2, 2] = sg2_dt + sb2_dt * dt * dt / 3.0
        Q[3, 3] = Q[4, 4] = Q[5, 5] = sb2_dt
        Q[0, 3] = Q[1, 4] = Q[2, 5] = -sb2_dt * dt / 2.0
        Q[3, 0] = Q[4, 1] = Q[5, 2] = -sb2_dt * dt / 2.0

        self.P = F @ self.P @ F.T + Q

        # ═══════════════════════════════════════════════════════
        # 3. MEASUREMENT UPDATE (accelerometer)
        # ═══════════════════════════════════════════════════════
        # Normalize accelerometer
        if a_norm > 1e-6:
            ax_n = ax / a_norm
            ay_n = ay / a_norm
            az_n = az / a_norm
        else:
            ax_n, ay_n, az_n = ax, ay, az

        # Predicted gravity from current quaternion
        q0, q1, q2, q3 = self.q
        gx_pred = 2.0 * (q1 * q3 - q0 * q2)
        gy_pred = 2.0 * (q0 * q1 + q2 * q3)
        gz_pred = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3

        # Innovation: measured - predicted gravity
        y = np.array([ax_n - gx_pred, ay_n - gy_pred, az_n - gz_pred])

        # Measurement Jacobian: H = [ [g_pred ×], 0 ]
        H = np.zeros((3, 6))
        H[0, 1] = -gz_pred
        H[0, 2] = gy_pred
        H[1, 0] = gz_pred
        H[1, 2] = -gx_pred
        H[2, 0] = -gy_pred
        H[2, 1] = gx_pred

        # ── R: amplify during motion, same as Mahony reducing Kp ──
        # R_effective = base_R / motion  →  stationary=motion=1 → normal R
        #                                  motion→0 → huge R → accel ignored
        if self.adaptive_r:
            r_eff = self.sigma_a ** 2 / max(motion, 0.01)
        else:
            r_eff = self.sigma_a ** 2
        R = np.eye(3) * r_eff

        # Kalman gain
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        # Update error state
        dx = K @ y
        self.P = (np.eye(6) - K @ H) @ self.P

        # ═══════════════════════════════════════════════════════
        # 4. INJECT error into nominal state
        # ═══════════════════════════════════════════════════════
        dtheta = dx[0:3]
        dbias = dx[3:6]

        # Quaternion update: q ← q ⊗ exp(δθ/2)
        dt_norm = math.sqrt(dtheta[0] ** 2 + dtheta[1] ** 2 + dtheta[2] ** 2)
        if dt_norm > 1e-12:
            half_angle = dt_norm * 0.5
            s = math.sin(half_angle) / dt_norm
            dq = np.array([math.cos(half_angle),
                           dtheta[0] * s, dtheta[1] * s, dtheta[2] * s])
            # q_new = q ⊗ dq
            q = self.q
            self.q[0] = q[0] * dq[0] - q[1] * dq[1] - q[2] * dq[2] - q[3] * dq[3]
            self.q[1] = q[0] * dq[1] + q[1] * dq[0] + q[2] * dq[3] - q[3] * dq[2]
            self.q[2] = q[0] * dq[2] - q[1] * dq[3] + q[2] * dq[0] + q[3] * dq[1]
            self.q[3] = q[0] * dq[3] + q[1] * dq[2] - q[2] * dq[1] + q[3] * dq[0]

        # Bias update
        self.bias += dbias

        # Normalize quaternion
        n = 1.0 / math.sqrt(self.q[0] ** 2 + self.q[1] ** 2 +
                            self.q[2] ** 2 + self.q[3] ** 2)
        self.q *= n

        return self._output()

    def _output(self):
        q0, q1, q2, q3 = self.q
        return {
            "roll": math.atan2(2.0 * (q0 * q1 + q2 * q3),
                               1.0 - 2.0 * (q1 * q1 + q2 * q2)) * RAD2DEG,
            "pitch": math.asin(2.0 * (q0 * q2 - q3 * q1)) * RAD2DEG,
            "yaw": math.atan2(2.0 * (q0 * q3 + q1 * q2),
                              1.0 - 2.0 * (q2 * q2 + q3 * q3)) * RAD2DEG,
            "q0": self.q[0], "q1": self.q[1],
            "q2": self.q[2], "q3": self.q[3],
            "bx": self.bias[0] / DEG2RAD,
            "by": self.bias[1] / DEG2RAD,
            "bz": self.bias[2] / DEG2RAD,
        }

    def run_dataset(self, data, progress=False):
        """Run EKF on full dataset. Returns (N, 3) [roll, pitch, yaw]."""
        self.reset()
        N = data.shape[0]
        result = np.empty((N, 3), dtype=np.float32)
        for i in range(N):
            ax, ay, az = float(data[i, 0]), float(data[i, 1]), float(data[i, 2])
            gx, gy, gz = float(data[i, 3]), float(data[i, 4]), float(data[i, 5])
            dt = 0.001 if data.shape[1] < 7 else float(data[i, 6])
            if dt <= 0 or dt > 0.01:
                dt = 0.001
            out = self.update(ax, ay, az, gx, gy, gz, dt)
            result[i, 0] = out["roll"]
            result[i, 1] = out["pitch"]
            result[i, 2] = out["yaw"]
            if progress and i % 10000 == 0 and i > 0:
                print(f"  ekf: {i}/{N}")
        return result
