/*
 * MPU6050 6-axis IMU driver — DMA I2C + ring buffer + CPU-decoupled ISR
 *
 * Wiring (GY-521 → HPM5300EVK P1):
 *   VCC  → 3.3V  (P1[1])    GND  → GND   (P1[6])
 *   SDA  → PB03  (P1[27])   SCL  → PB02  (P1[28])
 *   INT  → PA31  (P1[13])
 *
 * Sensitivity (±2g / ±250dps):
 *   Accel 1g = 16384 LSB    Gyro 1dps = 131 LSB
 *   Raw valid range: accel ±30000  gyro ±35000
 */

#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include <stdbool.h>
#include "hpm_common.h"
#include "hpm_i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============ MPU6050 7-bit I2C Address ============ */
#define MPU6050_ADDR_AD0_LOW   (0x68)
#define MPU6050_ADDR_AD0_HIGH  (0x69)

/* ============ Register Map ============ */
#define MPU6050_REG_SMPLRT_DIV       (0x19)
#define MPU6050_REG_CONFIG           (0x1A)
#define MPU6050_REG_GYRO_CONFIG      (0x1B)
#define MPU6050_REG_ACCEL_CONFIG     (0x1C)
#define MPU6050_REG_INT_PIN_CFG      (0x37)
#define MPU6050_REG_INT_ENABLE       (0x38)
#define MPU6050_REG_ACCEL_XOUT_H     (0x3B)
#define MPU6050_REG_USER_CTRL        (0x6A)
#define MPU6050_REG_PWR_MGMT_1       (0x6B)
#define MPU6050_REG_WHO_AM_I         (0x75)

/* ── I2C Master mode registers ── */
#define MPU6050_REG_I2C_MST_CTRL     (0x24)
#define MPU6050_REG_I2C_SLV0_ADDR    (0x25)
#define MPU6050_REG_I2C_SLV0_REG     (0x26)
#define MPU6050_REG_I2C_SLV0_CTRL    (0x27)
#define MPU6050_REG_EXT_SENS_DATA_00 (0x49)

/* ── AK8963 magnetometer ── */
#define AK8963_I2C_ADDR              (0x0C)
#define AK8963_REG_CNTL1             (0x0A)
#define AK8963_REG_HXL               (0x03)

#define MPU6050_DATA_LEN             (14U)

/* ============ Outlier filter limits (raw int16) ============ */
#define ACCEL_RAW_MAX   30000   /* ±2g ≈ ±32768, margin for safety */
#define GYRO_RAW_MAX    30000   /* ±250dps ≈ ±32768 */

/* ============ Gyro/Accel FS ============ */
typedef enum {
    MPU_GYRO_FS_250  = 0,
    MPU_GYRO_FS_500  = 1,
    MPU_GYRO_FS_1000 = 2,
    MPU_GYRO_FS_2000 = 3
} mpu6050_gyro_fs_t;

typedef enum {
    MPU_ACCEL_FS_2G  = 0,
    MPU_ACCEL_FS_4G  = 1,
    MPU_ACCEL_FS_8G  = 2,
    MPU_ACCEL_FS_16G = 3
} mpu6050_accel_fs_t;

typedef enum {
    MPU_DLPF_256HZ = 0,
    MPU_DLPF_188HZ = 1,
    MPU_DLPF_98HZ  = 2,
    MPU_DLPF_42HZ  = 3,
    MPU_DLPF_20HZ  = 4,
    MPU_DLPF_10HZ  = 5,
    MPU_DLPF_5HZ   = 6
} mpu6050_dlpf_t;

/* ============ Raw data frame (sensor register order) ============ */
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t temp;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} mpu6050_raw_frame_t;

/* ============ Data frame (physical + raw) ============ */
enum {
    RAW_AX = 0, RAW_AY, RAW_AZ, RAW_GX, RAW_GY, RAW_GZ, RAW_TEMP
};

typedef struct {
    float   accel_x;   /* g  */
    float   accel_y;
    float   accel_z;
    float   gyro_x;    /* °/s */
    float   gyro_y;
    float   gyro_z;
    float   temp;      /* °C (computed in main loop, not ISR) */
    int16_t raw[7];    /* originals: ax,ay,az,gx,gy,gz,temp */
    int16_t mag[3];    /* magnetometer raw: mx,my,mz */
    uint64_t capture_tick; /* mchtmr timestamp at DMA capture */
} mpu6050_data_t;

/* ============ Device handle ============ */
typedef struct {
    hpm_i2c_context_t  *i2c_ctx;
    uint16_t            dev_addr;
    float               accel_sf;
    float               gyro_sf;
    volatile bool       data_ready;
    volatile uint32_t   frame_count;
    volatile uint32_t   drop_count;
    volatile uint32_t   reject_count;   /* outlier frames skipped in main loop */
    volatile bool       dma_error;
    int16_t             gyro_off_at_T0[3];  /* gyro raw offset at T0 */
    float               T0;                 /* temperature during calib */
    float               K_T[3];             /* online temp coeff (°/s per °C) */
    bool                gyro_calib_done;
    bool                motion_warn;        /* motion detected during calib */
} mpu6050_t;

/* ============ Online temperature calibration ============ */
typedef struct {
    bool     active;         /* calibration in progress */
    float    lambda;         /* forgetting factor (0.99~0.999) */
    float    n_eff;          /* effective sample count */
    float    sum_dt;         /* Σ(T - T0) */
    float    sum_dt2;        /* Σ(T - T0)² */
    float    sum_bias[3];    /* Σgyro_bias (°/s) per axis */
    float    sum_dt_bias[3]; /* Σ(T-T0)*bias per axis */
    float    t_min, t_max;   /* temperature range seen */
    uint32_t update_count;   /* how many times K_T was updated */
} temp_calib_t;

/* ============ Ring buffer ============ */
#ifndef MPU6050_RING_DEPTH
#define MPU6050_RING_DEPTH  1024
#endif

typedef struct {
    mpu6050_data_t      frames[MPU6050_RING_DEPTH];
    volatile uint32_t   head;
    volatile uint32_t   tail;
} mpu6050_ring_t;

/* ============ DMA channel ============ */
#ifndef MPU6050_DMA_CH
#define MPU6050_DMA_CH      0
#endif

/* ============ API ============ */

void mpu6050_i2c_bus_recovery(hpm_i2c_context_t *i2c_ctx);

hpm_stat_t mpu6050_init(mpu6050_t *dev, hpm_i2c_context_t *i2c_ctx,
                         uint16_t dev_addr,
                         mpu6050_gyro_fs_t gyro_fs,
                         mpu6050_accel_fs_t accel_fs,
                         mpu6050_dlpf_t dlpf);

void mpu6050_start_acquisition(mpu6050_t *dev, mpu6050_ring_t *ring);
void mpu6050_stop_acquisition(void);
bool mpu6050_ring_pop(mpu6050_ring_t *ring, mpu6050_data_t *out);

hpm_stat_t mpu6050_who_am_i(mpu6050_t *dev, uint8_t *who_am_i);
hpm_stat_t mpu6050_set_dlpf(mpu6050_t *dev, mpu6050_dlpf_t dlpf);
hpm_stat_t mpu6050_get_dlpf(mpu6050_t *dev, uint8_t *dlpf);
bool mpu6050_read_mag_raw(mpu6050_t *dev, int16_t *mx, int16_t *my, int16_t *mz);
hpm_stat_t mpu6050_i2c_recover(hpm_i2c_context_t *i2c_ctx);
void mpu6050_calib_gyro_raw(mpu6050_t *dev, mpu6050_ring_t *ring, int raw_samples);
void mpu6050_calib_accel_6face(mpu6050_t *dev, mpu6050_ring_t *ring,
                                float *ox, float *oy, float *oz,
                                float *sx, float *sy, float *sz);

/* Online temperature drift calibration */
void  temp_calib_init(temp_calib_t *tc);
bool  temp_calib_collect(temp_calib_t *tc, float temp, float gx_bias,
                         float gy_bias, float gz_bias);
bool  temp_calib_update(temp_calib_t *tc, mpu6050_t *dev);

#ifdef __cplusplus
}
#endif
#endif /* MPU6050_H */
