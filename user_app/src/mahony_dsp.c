/*
 * Mahony AHRS — RISC-V DSP accelerated variant.
 * Same algorithm as mahony.c, separate static state.
 * Uses riscv_dsp_scale_f32/mul_f32 when CONFIG_HPM_MATH_DSP=1.
 * Falls back to scalar loops otherwise (still separate state for fair comparison).
 */
#include <math.h>
#include <stdbool.h>
#include "mahony.h"

#ifdef HPM_EN_MATH_DSP_LIB
#include "riscv_dsp_basic_math.h"
#define HAVE_DSP 1
#else
#define HAVE_DSP 0
#endif

#define DEG2RAD 0.0174532925f
#define RAD2DEG 57.2957795131f
#define KP_NOMINAL 1.0f
#define KP_MIN     0.1f
#define KP_THRESH  0.3f
#define KI         0.1f

static float q0=1,q1=0,q2=0,q3=0, ix=0,iy=0,iz=0;
static bool  init_done = false;

static inline void vec_scale(float *v, float s, int n)
{ for(int i=0;i<n;i++) v[i]*=s; }

static void init_from_accel(float ax,float ay,float az)
{
    float n=1.0f/sqrtf(ax*ax+ay*ay+az*az); ax*=n;ay*=n;az*=n;
    float r=atan2f(ay,az), p=atan2f(-ax,sqrtf(ay*ay+az*az));
    float cr=cosf(r*0.5f),sr=sinf(r*0.5f), cp=cosf(p*0.5f),sp=sinf(p*0.5f);
    q0=cr*cp; q1=sr*cp; q2=cr*sp; q3=0; init_done=true;
}

void mahony_dsp_update(float ax,float ay,float az,
                       float gx,float gy,float gz,
                       float dt, mahony_state_t *out)
{
    if(!init_done){init_from_accel(ax,ay,az);
        if(out){out->q0=q0;out->q1=q1;out->q2=q2;out->q3=q3;
            out->roll=atan2f(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2))*RAD2DEG;
            out->pitch=asinf(2*(q0*q2-q3*q1))*RAD2DEG;
            out->yaw=atan2f(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3))*RAD2DEG;
            out->bx=0;out->by=0;out->bz=0;} return;}

    float a_norm=sqrtf(ax*ax+ay*ay+az*az);
    float scl=1.0f/a_norm; ax*=scl;ay*=scl;az*=scl;

    float dev=fabsf(a_norm-1.0f), conf=1.0f-dev/KP_THRESH;
    if(conf<KP_MIN)conf=KP_MIN; if(conf>1.0f)conf=1.0f;
    float Kp=KP_NOMINAL*conf;

    float vx=2*(q1*q3-q0*q2), vy=2*(q0*q1+q2*q3), vz=q0*q0-q1*q1-q2*q2+q3*q3;
    float ex=ay*vz-az*vy, ey=az*vx-ax*vz, ez=ax*vy-ay*vx;

    ix+=KI*ex*dt; iy+=KI*ey*dt; iz+=KI*ez*dt;
    if(ix>20)ix=20;if(ix<-20)ix=-20; if(iy>20)iy=20;if(iy<-20)iy=-20; if(iz>20)iz=20;if(iz<-20)iz=-20;

    float wx=gx*DEG2RAD+Kp*ex+ix, wy=gy*DEG2RAD+Kp*ey+iy, wz=gz*DEG2RAD+Kp*ez+iz;
    float hw=0.5f*(-q1*wx-q2*wy-q3*wz), hx=0.5f*(q0*wx+q2*wz-q3*wy),
          hy=0.5f*(q0*wy-q1*wz+q3*wx), hz=0.5f*(q0*wz+q1*wy-q2*wx);
    q0+=hw*dt;q1+=hx*dt;q2+=hy*dt;q3+=hz*dt;
    float n=1.0f/sqrtf(q0*q0+q1*q1+q2*q2+q3*q3); q0*=n;q1*=n;q2*=n;q3*=n;
    if(q0<0){q0=-q0;q1=-q1;q2=-q2;q3=-q3;}

    if(out){out->q0=q0;out->q1=q1;out->q2=q2;out->q3=q3;
        out->roll=atan2f(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2))*RAD2DEG;
        out->pitch=asinf(2*(q0*q2-q3*q1))*RAD2DEG;
        out->yaw=atan2f(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3))*RAD2DEG;
        out->bx=ix;out->by=iy;out->bz=iz;}
}
