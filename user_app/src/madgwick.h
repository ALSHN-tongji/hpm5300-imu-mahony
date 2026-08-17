/*
 * Madgwick AHRS — 6-DOF IMU (no magnetometer)
 *
 * Gradient-descent orientation filter. Minimises error between
 * measured accelerometer vector and estimated gravity direction
 * derived from quaternion.
 *
 * Reference: S. Madgwick, "An efficient orientation filter for
 * inertial and inertial/magnetic sensor arrays", 2010.
 *
 * Compared against Mahony in MODE 5 (ALGO_CMP).
 */

#ifndef MADGWICK_H
#define MADGWICK_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float roll, pitch, yaw;    /* Euler angles (°) */
    float q0, q1, q2, q3;      /* quaternion (w,x,y,z) */
} madgwick_state_t;

/**
 * @param beta  Gradient descent step size (typical: 0.03~0.1).
 *              Larger = faster convergence, more noise.
 *              Smaller = smoother, slower response.
 */
void madgwick_init(void);
void madgwick_set_beta(float b);
float madgwick_get_beta(void);

void madgwick_update(float ax, float ay, float az,
                     float gx, float gy, float gz,
                     float dt, madgwick_state_t *out);

#ifdef __cplusplus
}
#endif
#endif
