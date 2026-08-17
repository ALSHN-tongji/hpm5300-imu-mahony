/*
 * IMU — MPU6050 DMA self-loop + Mahony AHRS
 * DMA runs continuously, INT-gated push, every frame through Mahony.
 */

#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include "board.h"
#include "mpu6050.h"
#include "mahony.h"
#include "hpm_mchtmr_drv.h"
#include "hpm_uart_drv.h"

#define I2C_DMAMUX_CH  DMA_SOC_CHN_TO_DMAMUX_CHN(BOARD_APP_I2C_DMA, 0)
#define DOWNSAMPLE_N   20
#define WATCHDOG_MS    500
#define WATCHDOG_TICKS  (24 * 1000 * WATCHDOG_MS)

static bool uart_tx_congested(void) {
    return uart_get_data_count_in_tx_fifo(BOARD_CONSOLE_UART_BASE) > (UART_SOC_FIFO_SIZE / 2);
}
static int print_nb(const char *fmt, ...) {
    if (uart_tx_congested()) return 0;
    char buf[256]; va_list a; va_start(a, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, a); va_end(a);
    if (len <= 0 || len >= (int)sizeof(buf)) return 0;
    for (int i = 0; i < len; i++) {
        if (buf[i] == '\n') { if (uart_send_byte(BOARD_CONSOLE_UART_BASE, '\r') != status_success) return 0; }
        if (uart_send_byte(BOARD_CONSOLE_UART_BASE, (uint8_t)buf[i]) != status_success) return 0;
    }
    return len;
}

int main(void) {
    hpm_stat_t stat; hpm_i2c_context_t i2c_ctx; mpu6050_t imu; mpu6050_ring_t ring; mpu6050_data_t s;
    uint32_t tick = 0, err = 0, ds = 0; uint64_t wd = 0;

    board_init(); board_init_led_pins();
    printf("# MPU6050 DMA self-loop + Mahony\n");

    board_init_i2c_clock(BOARD_APP_I2C_BASE); init_i2c_pins(BOARD_APP_I2C_BASE);
    hpm_i2c_get_default_init_context(&i2c_ctx); i2c_ctx.base = BOARD_APP_I2C_BASE;
    i2c_ctx.init_config.speed = i2c_speed_400khz;
    mpu6050_i2c_bus_recovery(&i2c_ctx);
    stat = hpm_i2c_initialize(&i2c_ctx);
    if (stat != status_success) { printf("# [ERR] I2C\n"); while (1) {} }

    stat = mpu6050_init(&imu, &i2c_ctx, MPU6050_ADDR_AD0_LOW, MPU_GYRO_FS_250, MPU_ACCEL_FS_2G, MPU_DLPF_42HZ);
    if (stat != status_success) { printf("# [ERR] IMU\n"); while (1) {} }
    printf("# [I2C] OK [IMU] OK\n# Ax,Ay,Az,Gx,Gy,Gz,Roll,Pitch,Yaw,q0,q1,q2,q3\n");

    mpu6050_calib_gyro_raw(&imu, &ring, 5000);
    if (!imu.gyro_calib_done) {
        imu.gyro_off_at_T0[0] = imu.gyro_off_at_T0[1] = imu.gyro_off_at_T0[2] = 0;
        imu.T0 = 25.0f;
    }

    /* 6-face accel calibration: uncomment to recalibrate, then copy result to main.c */
    // float OX, OY, OZ, SX, SY, SZ;
    // mpu6050_i2c_recover(&i2c_ctx);
    // mpu6050_calib_accel_6face(&imu, &ring, &OX, &OY, &OZ, &SX, &SY, &SZ);

    mahony_init(); imu.frame_count = 0; imu.dma_error = false;
    mpu6050_i2c_recover(&i2c_ctx);  /* clean bus after calib's DMA abort */
    mpu6050_start_acquisition(&imu, &ring);
    wd = mchtmr_get_count(HPM_MCHTMR);

    while (1) {
        bool got = false;
        while (mpu6050_ring_pop(&ring, &s)) {
            got = true;
            float gx = (s.raw[RAW_GX] - imu.gyro_off_at_T0[0]) / imu.gyro_sf;
            float gy = (s.raw[RAW_GY] - imu.gyro_off_at_T0[1]) / imu.gyro_sf;
            float gz = (s.raw[RAW_GZ] - imu.gyro_off_at_T0[2]) / imu.gyro_sf;
            /* Accel calibration */
            const float OX=331.1f, OY=-651.1f, OZ=-50.6f;
            const float SX=0.999533f, SY=0.997731f, SZ=0.985919f;
            float ax = SX * (s.raw[RAW_AX] - OX) / imu.accel_sf;
            float ay = SY * (s.raw[RAW_AY] - OY) / imu.accel_sf;
            float az = SZ * (s.raw[RAW_AZ] - OZ) / imu.accel_sf;

            static uint64_t ml;
            float dt = (ml == 0) ? 0.001f : (float)(s.capture_tick - ml) / (24.0f * 1000000.0f);
            if (dt > 0.005f) dt = 0.001f;
            ml = s.capture_tick;

            mahony_state_t ms; mahony_update(ax, ay, az, gx, gy, gz, dt, &ms);

            ds++; if (ds >= DOWNSAMPLE_N) { ds = 0;
                print_nb("%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%.4f,%.4f,%.4f,%.4f\n",
                         ax, ay, az, gx, gy, gz, ms.roll, ms.pitch, ms.yaw, ms.q0, ms.q1, ms.q2, ms.q3);
                tick++; board_led_toggle(); }
        }
        if (got) { wd = mchtmr_get_count(HPM_MCHTMR); }
        else if (mchtmr_get_count(HPM_MCHTMR) - wd > WATCHDOG_TICKS) {
            err++; printf("# !TIMEOUT,err=%lu\n", err);
            mpu6050_stop_acquisition(); mpu6050_i2c_bus_recovery(&i2c_ctx);
            hpm_i2c_initialize(&i2c_ctx);
            dmamux_config(BOARD_APP_I2C_DMAMUX, I2C_DMAMUX_CH, BOARD_APP_I2C_DMA_SRC, true);
            mpu6050_start_acquisition(&imu, &ring);
            imu.frame_count = 0; wd = mchtmr_get_count(HPM_MCHTMR); imu.dma_error = false;
        }
    }
}
