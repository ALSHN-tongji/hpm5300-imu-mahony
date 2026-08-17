/*
 * Mahony AHRS — PI-controlled complementary filter for 6-DOF IMU
 *
 * P term (Kp): proportional correction from accelerometer, controls
 *              convergence rate of roll/pitch
 * I term (Ki): integral of accel error → auto gyro bias estimation.
 *              The key advantage over Madgwick — bias is tracked online,
 *              so imperfect static calibration is compensated over time.
 *
 * Both terms only correct tilt (roll/pitch); yaw is gyro-only.
 */

#ifndef MAHONY_H
#define MAHONY_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float roll, pitch, yaw;    /* Euler angles (°) */
    float q0, q1, q2, q3;      /* quaternion (w,x,y,z) */
    float bx, by, bz;          /* estimated gyro bias (°/s) */
} mahony_state_t;

void mahony_init(void);

/**
 * @param ax, ay, az  Accelerometer in g (calibrated)
 * @param gx, gy, gz  Gyroscope in °/s (bias-corrected)
 * @param dt          Time step in seconds
 * @param out         Output state (may be NULL)
 */
void mahony_update(float ax, float ay, float az,
                   float gx, float gy, float gz,
                   float dt, mahony_state_t *out);

/* DSP-accelerated variant (separate state, same interface).
 * Requires CONFIG_HPM_MATH_DSP=1; falls back to scalar if not set. */
void mahony_dsp_update(float ax, float ay, float az,
                       float gx, float gy, float gz,
                       float dt, mahony_state_t *out);

#ifdef __cplusplus
}
#endif
#endif /* MAHONY_H */
