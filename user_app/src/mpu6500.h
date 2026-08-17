#ifndef MPU6500_H
#define MPU6500_H
#include <stdint.h>
#include <stdbool.h>
#include "hpm_common.h"
#include "hpm_i2c.h"
#ifdef __cplusplus
extern "C" {
#endif

#define MPU6500_ADDR (0x68)
#define MPU6500_REG_SMPLRT_DIV 0x19
#define MPU6500_REG_CONFIG 0x1A
#define MPU6500_REG_GYRO_CONFIG 0x1B
#define MPU6500_REG_ACCEL_CONFIG 0x1C
#define MPU6500_REG_INT_PIN_CFG 0x37
#define MPU6500_REG_INT_ENABLE 0x38
#define MPU6500_REG_ACCEL_XOUT_H 0x3B
#define MPU6500_REG_PWR_MGMT_1 0x6B
#define MPU6500_REG_WHO_AM_I 0x75
#define MPU6500_DATA_LEN 14

typedef enum{MPU_GYRO_FS_250=0,MPU_GYRO_FS_500=1,MPU_GYRO_FS_1000=2,MPU_GYRO_FS_2000=3}mpu6500_gyro_fs_t;
typedef enum{MPU_ACCEL_FS_2G=0,MPU_ACCEL_FS_4G=1,MPU_ACCEL_FS_8G=2,MPU_ACCEL_FS_16G=3}mpu6500_accel_fs_t;
typedef enum{MPU_DLPF_256HZ=0,MPU_DLPF_188HZ=1,MPU_DLPF_98HZ=2,MPU_DLPF_42HZ=3,MPU_DLPF_20HZ=4,MPU_DLPF_10HZ=5,MPU_DLPF_5HZ=6}mpu6500_dlpf_t;
enum{RAW_AX=0,RAW_AY,RAW_AZ,RAW_GX,RAW_GY,RAW_GZ};

typedef struct{float accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,temp;int16_t raw[6];}mpu6500_data_t;
#ifndef MPU6500_RING_DEPTH
#define MPU6500_RING_DEPTH 512
#endif
typedef struct{mpu6500_data_t frames[MPU6500_RING_DEPTH];volatile uint32_t head,tail;}mpu6500_ring_t;
typedef struct{hpm_i2c_context_t*i2c_ctx;uint16_t dev_addr;float accel_sf,gyro_sf;
    volatile uint32_t frame_count,drop_count;volatile bool dma_error;
    int16_t gyro_off[3];float T0;bool calib_done;}mpu6500_t;

hpm_stat_t mpu6500_init(mpu6500_t*d,hpm_i2c_context_t*i,uint16_t a,mpu6500_gyro_fs_t g,mpu6500_accel_fs_t f,mpu6500_dlpf_t l);
void mpu6500_start_acquisition(mpu6500_ring_t*r);
void mpu6500_stop_acquisition(void);
bool mpu6500_ring_pop(mpu6500_ring_t*r,mpu6500_data_t*o);
void mpu6500_calib_gyro(mpu6500_t*d,mpu6500_ring_t*r,int n);
void mpu6500_i2c_bus_recovery(hpm_i2c_context_t*c);
bool mpu6500_trigger_read(void);
uint8_t mpu6500_read_reg(mpu6500_t*d,uint8_t r);

#ifdef __cplusplus
}
#endif
#endif
