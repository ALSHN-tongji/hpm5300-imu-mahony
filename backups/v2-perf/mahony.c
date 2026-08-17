/*
 * Mahony AHRS — 6-DOF IMU attitude estimation
 *
 * PI-controlled complementary filter on SO(3):
 *   - P term (Kp): cross-product error × Kp → instant tilt correction
 *   - I term (Ki): integrated error × Ki → gyro bias auto-estimation
 *
 * Reference: Mahony et al., "Nonlinear Complementary Filters on
 * the Special Orthogonal Group", IEEE Trans. Automatic Control, 2008.
 */

#include <math.h>
#include <stdbool.h>
#include "mahony.h"

#define DEG2RAD  0.0174532925f
#define RAD2DEG  57.2957795131f

/* Gains tuned for 1kHz filter rate */
#define KP_NOMINAL  1.0f    /* max P gain when accel is trustworthy */
#define KP_MIN      0.1f    /* min P gain floor — never fully trust gyro alone */
#define KP_THRESH   0.3f    /* |a| deviation from 1g to lose confidence (g) */
#define KI          0.1f    /* I gain — always active, bias changes slowly */

static float q0=1, q1=0, q2=0, q3=0;
static float ix=0, iy=0, iz=0;
static bool  initialized = false;

void mahony_init(void)
{
    q0=1; q1=0; q2=0; q3=0;
    ix=0; iy=0; iz=0;
    initialized = false;
}

static void mahony_init_from_accel(float ax, float ay, float az)
{
    float n = 1.0f / sqrtf(ax*ax + ay*ay + az*az);
    ax *= n; ay *= n; az *= n;

    float roll  = atan2f(ay, az);
    float pitch = atan2f(-ax, sqrtf(ay*ay + az*az));

    float cr = cosf(roll * 0.5f),  sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f),  sp = sinf(pitch * 0.5f);

    q0 = cr*cp;  q1 = sr*cp;  q2 = cr*sp;  q3 = 0.0f;
    initialized = true;
}

void mahony_update(float ax, float ay, float az,
                   float gx, float gy, float gz,
                   float dt, mahony_state_t *out)
{
    if (!initialized) {
        mahony_init_from_accel(ax, ay, az);
        if (out) {
            out->q0=q0; out->q1=q1; out->q2=q2; out->q3=q3;
            out->roll  = atan2f(2.0f*(q0*q1+q2*q3), 1.0f-2.0f*(q1*q1+q2*q2))*RAD2DEG;
            out->pitch = asinf(2.0f*(q0*q2-q3*q1))*RAD2DEG;
            out->yaw   = atan2f(2.0f*(q0*q3+q1*q2), 1.0f-2.0f*(q2*q2+q3*q3))*RAD2DEG;
            out->bx = 0; out->by = 0; out->bz = 0;
        }
        return;
    }

    float recip_norm;
    float hw, hx, hy, hz;

    float a_norm = sqrtf(ax*ax + ay*ay + az*az);
    recip_norm = 1.0f / a_norm;
    ax *= recip_norm; ay *= recip_norm; az *= recip_norm;

    /* Adaptive Kp: trust accelerometer less when |a| deviates from 1g */
    float dev  = fabsf(a_norm - 1.0f);
    float conf = 1.0f - dev / KP_THRESH;
    if (conf < KP_MIN) conf = KP_MIN;
    if (conf > 1.0f)   conf = 1.0f;
    float Kp = KP_NOMINAL * conf;

    float vx = 2.0f * (q1*q3 - q0*q2);
    float vy = 2.0f * (q0*q1 + q2*q3);
    float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    float ex = ay*vz - az*vy;
    float ey = az*vx - ax*vz;
    float ez = ax*vy - ay*vx;

    if (KI > 0.0f) {
        ix += KI * ex * dt;
        iy += KI * ey * dt;
        iz += KI * ez * dt;
        if (ix >  20.0f) ix =  20.0f;
        if (ix < -20.0f) ix = -20.0f;
        if (iy >  20.0f) iy =  20.0f;
        if (iy < -20.0f) iy = -20.0f;
        if (iz >  20.0f) iz =  20.0f;
        if (iz < -20.0f) iz = -20.0f;
    }

    float wx = gx * DEG2RAD + Kp * ex + ix;
    float wy = gy * DEG2RAD + Kp * ey + iy;
    float wz = gz * DEG2RAD + Kp * ez + iz;

    hw = 0.5f * (-q1*wx - q2*wy - q3*wz);
    hx = 0.5f * ( q0*wx + q2*wz - q3*wy);
    hy = 0.5f * ( q0*wy - q1*wz + q3*wx);
    hz = 0.5f * ( q0*wz + q1*wy - q2*wx);

    q0 += hw * dt;  q1 += hx * dt;
    q2 += hy * dt;  q3 += hz * dt;

    recip_norm = 1.0f / sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recip_norm; q1 *= recip_norm;
    q2 *= recip_norm; q3 *= recip_norm;
    if (q0 < 0) { q0 = -q0; q1 = -q1; q2 = -q2; q3 = -q3; }

    if (out) {
        out->q0=q0; out->q1=q1; out->q2=q2; out->q3=q3;
        out->roll  = atan2f(2.0f*(q0*q1+q2*q3), 1.0f-2.0f*(q1*q1+q2*q2))*RAD2DEG;
        out->pitch = asinf(2.0f*(q0*q2-q3*q1))*RAD2DEG;
        out->yaw   = atan2f(2.0f*(q0*q3+q1*q2), 1.0f-2.0f*(q2*q2+q3*q3))*RAD2DEG;
        out->bx = ix; out->by = iy; out->bz = iz;
    }
}

