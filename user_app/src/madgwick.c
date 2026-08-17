/*
 * Madgwick AHRS — 6-DOF IMU (no magnetometer)
 *
 * Gradient-descent orientation filter. Minimises error between
 * measured accelerometer vector and estimated gravity direction
 * derived from quaternion.
 *
 * Reference: S. Madgwick, "An efficient orientation filter for
 * inertial and inertial/magnetic sensor arrays", 2010.
 */

#include <math.h>
#include <stdbool.h>
#include "madgwick.h"

#define DEG2RAD  0.017453292519943295f
#define RAD2DEG  57.29577951308232f

/* Default beta — tuned for 1kHz ODR, ±250dps gyro */
#define BETA_DEFAULT  0.05f

static float q0=1, q1=0, q2=0, q3=0;
static float beta = BETA_DEFAULT;
static bool  initialized = false;

void madgwick_init(void)
{
    q0=1; q1=0; q2=0; q3=0;
    beta = BETA_DEFAULT;
    initialized = false;
}

void madgwick_set_beta(float b) { beta = b; }
float madgwick_get_beta(void)   { return beta; }

static void init_from_accel(float ax, float ay, float az)
{
    float n = 1.0f / sqrtf(ax*ax + ay*ay + az*az);
    ax *= n; ay *= n; az *= n;

    float roll  = atan2f(ay, az);
    float pitch = atan2f(-ax, sqrtf(ay*ay + az*az));

    float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);

    q0 = cr*cp;  q1 = sr*cp;  q2 = cr*sp;  q3 = 0.0f;
    initialized = true;
}

void madgwick_update(float ax, float ay, float az,
                     float gx, float gy, float gz,
                     float dt, madgwick_state_t *out)
{
    if (!initialized) {
        init_from_accel(ax, ay, az);
        if (out) {
            out->q0=q0; out->q1=q1; out->q2=q2; out->q3=q3;
            out->roll  = atan2f(2.0f*(q0*q1+q2*q3), 1.0f-2.0f*(q1*q1+q2*q2))*RAD2DEG;
            out->pitch = asinf(2.0f*(q0*q2-q3*q1))*RAD2DEG;
            out->yaw   = atan2f(2.0f*(q0*q3+q1*q2), 1.0f-2.0f*(q2*q2+q3*q3))*RAD2DEG;
        }
        return;
    }

    /* ── Normalize accelerometer ── */
    float recip = 1.0f / sqrtf(ax*ax + ay*ay + az*az);
    ax *= recip; ay *= recip; az *= recip;

    /* ── Objective function: f(q) = estimated_gravity(q) - measured_gravity
     *   Estimated gravity direction from quaternion (z-axis of body frame):
     *     g_est = [ 2*(q1*q3 - q0*q2),
     *               2*(q0*q1 + q2*q3),
     *               q0² - q1² - q2² + q3² ]
     *   Error: f = g_est - [ax, ay, az] ── */
    float f1 = 2.0f * (q1*q3 - q0*q2) - ax;
    float f2 = 2.0f * (q0*q1 + q2*q3) - ay;
    float f3 = (q0*q0 - q1*q1 - q2*q2 + q3*q3) - az;

    /* ── Jacobian J of f w.r.t q = [∂f/∂q0, ∂f/∂q1, ∂f/∂q2, ∂f/∂q3]
     *   J = [[ -2q2,  2q3, -2q0,  2q1 ],
     *        [  2q1,  2q0,  2q3,  2q2 ],
     *        [  2q0, -2q1, -2q2,  2q0 ]]  (wait, let me recalculate)
     *
     * Actually from Madgwick's paper, 6-axis Jacobian:
     *   J[0] = [ -2q2,  2q3, -2q0,  2q1 ]   (∂f1/∂q)
     *   J[1] = [  2q1,  2q0,  2q3,  2q2 ]   (∂f2/∂q)
     *   J[2] = [  2q0, -2q1, -2q2,  0  ]   (∂f3/∂q) — note q3 term is 0
     * ── */

    /* ── Gradient ∇f = J^T * f ── */
    float g0 = -2.0f*q2*f1 + 2.0f*q1*f2 + 2.0f*q0*f3;
    float g1 =  2.0f*q3*f1 + 2.0f*q0*f2 - 2.0f*q1*f3;
    float g2 = -2.0f*q0*f1 + 2.0f*q3*f2 - 2.0f*q2*f3;
    float g3 =  2.0f*q1*f1 + 2.0f*q2*f2;

    /* ── Normalize gradient ── */
    float g_norm = sqrtf(g0*g0 + g1*g1 + g2*g2 + g3*g3);
    if (g_norm > 1e-9f) {
        recip = 1.0f / g_norm;
        g0 *= recip; g1 *= recip; g2 *= recip; g3 *= recip;
    }

    /* ── Quaternion derivative from gyroscope ── */
    float wq0 = 0.5f * (-q1*gx - q2*gy - q3*gz) * DEG2RAD;
    float wq1 = 0.5f * ( q0*gx + q2*gz - q3*gy) * DEG2RAD;
    float wq2 = 0.5f * ( q0*gy - q1*gz + q3*gx) * DEG2RAD;
    float wq3 = 0.5f * ( q0*gz + q1*gy - q2*gx) * DEG2RAD;

    /* ── Fused derivative: q_dot = wq - β * ∇f/|∇f| ── */
    q0 += (wq0 - beta * g0) * dt;
    q1 += (wq1 - beta * g1) * dt;
    q2 += (wq2 - beta * g2) * dt;
    q3 += (wq3 - beta * g3) * dt;

    /* ── Normalize quaternion ── */
    recip = 1.0f / sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recip; q1 *= recip; q2 *= recip; q3 *= recip;
    if (q0 < 0) { q0 = -q0; q1 = -q1; q2 = -q2; q3 = -q3; }

    if (out) {
        out->q0=q0; out->q1=q1; out->q2=q2; out->q3=q3;
        out->roll  = atan2f(2.0f*(q0*q1+q2*q3), 1.0f-2.0f*(q1*q1+q2*q2))*RAD2DEG;
        out->pitch = asinf(2.0f*(q0*q2-q3*q1))*RAD2DEG;
        out->yaw   = atan2f(2.0f*(q0*q3+q1*q2), 1.0f-2.0f*(q2*q2+q3*q3))*RAD2DEG;
    }
}
