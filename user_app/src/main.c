/*
 * IMU — MPU6050 DMA self-loop + Mahony AHRS
 *
 * Test modes (set TEST_MODE before build):
 *   0 = NORMAL   original output + #CYC cycle stats every 200 frames
 *   1 = PERF     CSV: mahony+frame min/avg/max cycles per 200-frame window
 *   2 = DRIFT    static drift test: t_s,Roll,Pitch,Yaw,ax,ay,az,gx,gy,gz @ 1Hz
 *   3 = DSP_CMP  alternate float Mahony vs DSP Mahony each frame
 *   4 = SAT      saturation sweep: variable ODR via software decimation
 *   5 = ALGO_CMP Mahony vs Madgwick comparison (same input, alternate frames)
 *   6 = RAW_DATA raw calibrated sensor data, no AHRS (for offline replay/sweep)
 *   7 = DLPF_SWP auto-cycle all 7 DLPF bandwidths (30s each), raw data output
 */

#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include "board.h"
#include "mpu6050.h"
#include "mahony.h"
#include "madgwick.h"
#include "perf_counter.h"
#include "hpm_mchtmr_drv.h"
#include "hpm_uart_drv.h"

#define TEST_MODE  0
#define DLPF_SETTING    MPU_DLPF_98HZ
#define ADAPTIVE_DLPF   1

#if ADAPTIVE_DLPF && TEST_MODE != 7
#define DLPF_STATIC     MPU_DLPF_20HZ
#define DLPF_NORMAL     MPU_DLPF_42HZ
#define DLPF_FAST       MPU_DLPF_256HZ
#define GYRO_STATIC_THR  5.0f     /* °/s — below this, assume static */
#define GYRO_FAST_THR    100.0f   /* °/s — above this, assume fast maneuver */
#define DLPF_COOLDOWN_S  1.0f     /* min seconds between DLPF switches */
static mpu6050_dlpf_t  dlpf_state = DLPF_SETTING;
static uint64_t        dlpf_last_switch = 0;
static float           gyro_mag_lp = 0;  /* low-pass filtered gyro magnitude */

static void adaptive_dlpf_update(mpu6050_t *dev, float gx, float gy, float gz)
{
    float mag = sqrtf(gx*gx + gy*gy + gz*gz);
    /* 1st-order LP with 0.5s time constant @ 1kHz */
    gyro_mag_lp += 0.002f * (mag - gyro_mag_lp);

    uint64_t now = mchtmr_get_count(HPM_MCHTMR);
    if ((now - dlpf_last_switch) < (uint64_t)(DLPF_COOLDOWN_S * 24000000ULL))
        return;

    mpu6050_dlpf_t target;
    if (gyro_mag_lp < GYRO_STATIC_THR)       target = DLPF_STATIC;
    else if (gyro_mag_lp > GYRO_FAST_THR)    target = DLPF_FAST;
    else                                      target = DLPF_NORMAL;

    if (target != dlpf_state) {
        dlpf_state = target;
        dlpf_last_switch = now;
        mpu6050_set_dlpf(dev, target);
    }
}
#endif /* ADAPTIVE_DLPF */

#define I2C_DMAMUX_CH  DMA_SOC_CHN_TO_DMAMUX_CHN(BOARD_APP_I2C_DMA, 0)
#define DOWNSAMPLE_N   20
#define WATCHDOG_MS    500
#define WATCHDOG_TICKS  (24 * 1000 * WATCHDOG_MS)

/* ── Cycle stats ── */
static perf_stat_t g_mahony, g_frame;
#if TEST_MODE == 3
static perf_stat_t g_mahony_dsp, g_frame_dsp;
#endif
#if TEST_MODE == 5
static perf_stat_t g_madgwick, g_frame_mg;
#endif

/* ── MODE 7: DLPF auto-sweep ── */
#if TEST_MODE == 7
static const mpu6050_dlpf_t dlpf_list[] = {
    MPU_DLPF_256HZ, MPU_DLPF_188HZ, MPU_DLPF_98HZ,
    MPU_DLPF_42HZ,  MPU_DLPF_20HZ,  MPU_DLPF_10HZ,
    MPU_DLPF_5HZ
};
static const int dlpf_hz[] = {256, 188, 98, 42, 20, 10, 5};
#define DLPF_COUNT      7
#define DLPF_DURATION_S 30
static int      dlpf_idx = 0;
static uint64_t dlpf_t0  = 0;   /* MCHTMR tick at section start */
static bool     dlpf_first = true;
#endif

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

/* MODE 7: safe DLPF switch. The blocking imu_write_reg inside
 * mpu6050_set_dlpf() is documented "init only — not used during ISR-driven
 * acquisition"; calling it while the DMA ISR drives the I2C bus makes the
 * write silently fail, so the DLPF never actually changes. Stop acquisition
 * -> re-init I2C to polling -> write + read back -> re-arm DMA -> restart. */
#if TEST_MODE == 7
static void dlpf_switch(mpu6050_t *dev, hpm_i2c_context_t *i2c_ctx,
                        mpu6050_ring_t *ring, mpu6050_dlpf_t dlpf, int hz)
{
    mpu6050_stop_acquisition();
    board_delay_ms(2);                  /* let in-flight DMA ISR drain */
    hpm_i2c_initialize(i2c_ctx);

    /* Retry: the blocking write races with the just-stopped DMA ISR and
     * intermittently returns NACK/timeout. A couple retries make it solid. */
    hpm_stat_t stat;
    int attempt;
    for (attempt = 0; attempt < 3; attempt++) {
        stat = mpu6050_set_dlpf(dev, dlpf);
        if (stat == status_success) break;
        board_delay_ms(1);
    }
    uint8_t rb = 0;
    if (stat == status_success) mpu6050_get_dlpf(dev, &rb);

    dmamux_config(BOARD_APP_I2C_DMAMUX, I2C_DMAMUX_CH, BOARD_APP_I2C_DMA_SRC, true);
    mpu6050_start_acquisition(dev, ring);
    board_delay_ms(10);                 /* let DLPF filter settle */

    print_nb("#DLPF=%d\n", hz);
    print_nb("#DLPFWRITE hz=%d write=%s readback=0x%02X\n", hz,
             stat == status_success ? "OK" : "FAIL", rb);
}
#endif

int main(void) {
    hpm_stat_t stat; hpm_i2c_context_t i2c_ctx; mpu6050_t imu; mpu6050_ring_t ring; mpu6050_data_t s;
    uint32_t tick = 0, err = 0, ds = 0; uint64_t wd = 0;

    board_init(); board_init_led_pins();
    perf_init();
    perf_stat_reset(&g_mahony);
    perf_stat_reset(&g_frame);

    printf("# MPU6050 DMA self-loop + Mahony\n");

    board_init_i2c_clock(BOARD_APP_I2C_BASE); init_i2c_pins(BOARD_APP_I2C_BASE);
    hpm_i2c_get_default_init_context(&i2c_ctx); i2c_ctx.base = BOARD_APP_I2C_BASE;
    i2c_ctx.init_config.speed = i2c_speed_400khz;
    mpu6050_i2c_bus_recovery(&i2c_ctx);
    stat = hpm_i2c_initialize(&i2c_ctx);
    if (stat != status_success) { printf("# [ERR] I2C\n"); while (1) {} }

    stat = mpu6050_init(&imu, &i2c_ctx, MPU6050_ADDR_AD0_LOW, MPU_GYRO_FS_250, MPU_ACCEL_FS_2G, DLPF_SETTING);
    if (stat != status_success) { printf("# [ERR] IMU\n"); while (1) {} }

#if TEST_MODE == 1 || TEST_MODE == 3
    printf("# MODE=PERF  CPU=480MHz  cycles->us /480\n");
#if TEST_MODE == 3
    printf("# mahony_min,mahony_avg,mahony_max,mahony_us_avg,"
           "dsp_min,dsp_avg,dsp_max,dsp_us_avg\n");
#else
    printf("# mahony_min,mahony_avg,mahony_max,mahony_us_min,mahony_us_avg,mahony_us_max,"
           "frame_min,frame_avg,frame_max,frame_us_min,frame_us_avg,frame_us_max\n");
#endif
#elif TEST_MODE == 2
    printf("# MODE=DRIFT  static drift test\n# t_s,Roll,Pitch,Yaw,ax,ay,az,gx,gy,gz\n");
#elif TEST_MODE == 5
    printf("# MODE=ALGO_CMP  Mahony vs Madgwick  CPU=480MHz\n");
    printf("# mahony_min,mahony_avg,mahony_max,mahony_us_avg,"
           "madgwick_min,madgwick_avg,madgwick_max,madgwick_us_avg\n");
#elif TEST_MODE == 4
    printf("# MODE=SAT  saturation sweep\n# decim,rate_hz,mahony_min,mahony_avg,mahony_max,mahony_us_avg\n");
#elif TEST_MODE == 6
    printf("# MODE=RAW_DATA  calibrated sensor output, no AHRS\n");
    printf("# tick_24mhz,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps\n");
#elif TEST_MODE == 7
    printf("# MODE=DLPF_SWP  auto-cycle 7 DLPF settings, 30s each\n");
    printf("# DLPF sections marked with #DLPF=<hz> headers\n");
#else
    printf("# [I2C] OK [IMU] OK\n# Ax,Ay,Az,Gx,Gy,Gz,Roll,Pitch,Yaw,q0,q1,q2,q3\n");
#endif

    mpu6050_calib_gyro_raw(&imu, &ring, 5000);
    if (!imu.gyro_calib_done) {
        imu.gyro_off_at_T0[0] = imu.gyro_off_at_T0[1] = imu.gyro_off_at_T0[2] = 0;
        imu.T0 = 25.0f;
    }

    mahony_init(); madgwick_init(); imu.frame_count = 0; imu.dma_error = false;
    mpu6050_i2c_recover(&i2c_ctx);
    mpu6050_start_acquisition(&imu, &ring);
    wd = mchtmr_get_count(HPM_MCHTMR);

#if TEST_MODE == 2
    uint32_t drift_last = 0;
    uint64_t drift_t0 = mchtmr_get_count(HPM_MCHTMR);
#endif
#if TEST_MODE == 4
    int sat_factors[] = {1, 2, 4, 8, 10};
    int sat_idx = 0;
    uint32_t sat_n = 0;
#endif

    while (1) {
        bool got = false;

        while (mpu6050_ring_pop(&ring, &s)) {
            got = true;
            uint64_t frame_t0 = perf_now();
            float gx = (s.raw[RAW_GX] - imu.gyro_off_at_T0[0]) / imu.gyro_sf;
            float gy = (s.raw[RAW_GY] - imu.gyro_off_at_T0[1]) / imu.gyro_sf;
            float gz = (s.raw[RAW_GZ] - imu.gyro_off_at_T0[2]) / imu.gyro_sf;
            const float OX=331.1f, OY=-651.1f, OZ=-50.6f;
            const float SX=0.999533f, SY=0.997731f, SZ=0.985919f;
            float ax = SX * (s.raw[RAW_AX] - OX) / imu.accel_sf;
            float ay = SY * (s.raw[RAW_AY] - OY) / imu.accel_sf;
            float az = SZ * (s.raw[RAW_AZ] - OZ) / imu.accel_sf;

            static uint64_t ml;
            float dt = (ml == 0) ? 0.001f : (float)(s.capture_tick - ml) / (24.0f * 1000000.0f);
            if (dt > 0.005f) dt = 0.001f;
            ml = s.capture_tick;

#if ADAPTIVE_DLPF && TEST_MODE != 7
            adaptive_dlpf_update(&imu, gx, gy, gz);
#endif

#if TEST_MODE == 4
            static int sat_skip = 0;
            if (++sat_skip < sat_factors[sat_idx]) continue;
            sat_skip = 0;
#endif

            mahony_state_t ms;
            uint64_t mahony_t0 = perf_now();
#if TEST_MODE == 3
            static int dsp_tog = 0;
            if (dsp_tog)  mahony_dsp_update(ax, ay, az, gx, gy, gz, dt, &ms);
            else          mahony_update(ax, ay, az, gx, gy, gz, dt, &ms);
#elif TEST_MODE == 5
            static int algo_tog = 0;
            madgwick_state_t mgs;
            if (algo_tog) {
                madgwick_update(ax, ay, az, gx, gy, gz, dt, &mgs);
                ms.roll=mgs.roll; ms.pitch=mgs.pitch; ms.yaw=mgs.yaw;
                ms.q0=mgs.q0; ms.q1=mgs.q1; ms.q2=mgs.q2; ms.q3=mgs.q3;
            } else {
                mahony_update(ax, ay, az, gx, gy, gz, dt, &ms);
            }
#else
            mahony_update(ax, ay, az, gx, gy, gz, dt, &ms);
#endif
            uint64_t mahony_el = perf_now() - mahony_t0;
            uint64_t frame_el  = perf_now() - frame_t0;

#if TEST_MODE == 3
            if (dsp_tog) {
                perf_stat_record(&g_mahony_dsp, mahony_el);
                perf_stat_record(&g_frame_dsp,  frame_el);
            } else {
                perf_stat_record(&g_mahony, mahony_el);
                perf_stat_record(&g_frame,  frame_el);
            }
            dsp_tog = !dsp_tog;
#elif TEST_MODE == 5
            if (algo_tog) {
                perf_stat_record(&g_madgwick, mahony_el);
                perf_stat_record(&g_frame_mg,  frame_el);
            } else {
                perf_stat_record(&g_mahony, mahony_el);
                perf_stat_record(&g_frame,  frame_el);
            }
            algo_tog = !algo_tog;
#else
            perf_stat_record(&g_mahony, mahony_el);
            perf_stat_record(&g_frame,  frame_el);
#endif

            ds++;
#if TEST_MODE == 6
            print_nb("%llu,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f\n",
                     (unsigned long long)s.capture_tick, ax, ay, az, gx, gy, gz);
            tick++; board_led_toggle();
            continue;
#endif
#if TEST_MODE == 7
            {
                uint64_t now = mchtmr_get_count(HPM_MCHTMR);
                if (dlpf_first) {
                    dlpf_first = false;
                    dlpf_t0 = now;
                    dlpf_switch(&imu, &i2c_ctx, &ring, dlpf_list[0], dlpf_hz[0]);
                }
                if ((now - dlpf_t0) >= (uint64_t)DLPF_DURATION_S * 24000000ULL) {
                    dlpf_idx++;
                    if (dlpf_idx >= DLPF_COUNT) {
                        print_nb("#DLPF_SWEEP_DONE\n");
                        while (1) {}
                    }
                    dlpf_t0 = now;
                    dlpf_switch(&imu, &i2c_ctx, &ring, dlpf_list[dlpf_idx], dlpf_hz[dlpf_idx]);
                }
                print_nb("%llu,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f\n",
                         (unsigned long long)s.capture_tick, ax, ay, az, gx, gy, gz);
                tick++; board_led_toggle();
                continue;
            }
#endif
#if TEST_MODE == 2
            uint64_t now = mchtmr_get_count(HPM_MCHTMR);
            if ((now - drift_last) >= 24000000ULL) {
                drift_last = now;
                print_nb("%.1f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f\n",
                         (float)(now-drift_t0)/24000000.0f, ms.roll, ms.pitch, ms.yaw,
                         ax, ay, az, gx, gy, gz);
                tick++; board_led_toggle();
            }
#elif TEST_MODE == 1 || TEST_MODE == 3 || TEST_MODE == 5
            if (ds >= DOWNSAMPLE_N) { ds = 0;
                if (tick > 0 && tick % 200 == 0) {
#if TEST_MODE == 3
                    float ma  = g_mahony.count     ? (float)(g_mahony.sum/g_mahony.count)         : 0;
                    float dma = g_mahony_dsp.count  ? (float)(g_mahony_dsp.sum/g_mahony_dsp.count) : 0;
                    print_nb("%lu,%.1f,%lu,%.2f,%lu,%.1f,%lu,%.2f\n",
                        (unsigned long)g_mahony.min, ma, (unsigned long)g_mahony.max,
                        PERF_CYCLE_TO_US(ma),
                        (unsigned long)g_mahony_dsp.min, dma, (unsigned long)g_mahony_dsp.max,
                        PERF_CYCLE_TO_US(dma));
                    perf_stat_reset(&g_mahony); perf_stat_reset(&g_frame);
                    perf_stat_reset(&g_mahony_dsp); perf_stat_reset(&g_frame_dsp);
#elif TEST_MODE == 5
                    float ma  = g_mahony.count     ? (float)(g_mahony.sum/g_mahony.count)         : 0;
                    float mga = g_madgwick.count   ? (float)(g_madgwick.sum/g_madgwick.count)     : 0;
                    print_nb("%lu,%.1f,%lu,%.2f,%lu,%.1f,%lu,%.2f\n",
                        (unsigned long)g_mahony.min, ma, (unsigned long)g_mahony.max,
                        PERF_CYCLE_TO_US(ma),
                        (unsigned long)g_madgwick.min, mga, (unsigned long)g_madgwick.max,
                        PERF_CYCLE_TO_US(mga));
                    perf_stat_reset(&g_mahony); perf_stat_reset(&g_frame);
                    perf_stat_reset(&g_madgwick); perf_stat_reset(&g_frame_mg);
#else
                    float ma = g_mahony.count ? (float)(g_mahony.sum/g_mahony.count) : 0;
                    float fa = g_frame.count  ? (float)(g_frame.sum/g_frame.count)   : 0;
                    print_nb("%lu,%.1f,%lu,%.2f,%.2f,%.2f,%lu,%.1f,%lu,%.2f,%.2f,%.2f\n",
                        (unsigned long)g_mahony.min, ma, (unsigned long)g_mahony.max,
                        PERF_CYCLE_TO_US(g_mahony.min), PERF_CYCLE_TO_US(ma), PERF_CYCLE_TO_US(g_mahony.max),
                        (unsigned long)g_frame.min, fa, (unsigned long)g_frame.max,
                        PERF_CYCLE_TO_US(g_frame.min), PERF_CYCLE_TO_US(fa), PERF_CYCLE_TO_US(g_frame.max));
                    perf_stat_reset(&g_mahony); perf_stat_reset(&g_frame);
#endif
                }
                tick++; board_led_toggle(); }
#elif TEST_MODE == 4
            if (++sat_n >= 500) { sat_n = 0;
                float ma = g_mahony.count ? (float)(g_mahony.sum/g_mahony.count) : 0;
                print_nb("%d,%d,%lu,%.1f,%lu,%.2f\n",
                    sat_factors[sat_idx], 1000/sat_factors[sat_idx],
                    (unsigned long)g_mahony.min, ma, (unsigned long)g_mahony.max,
                    PERF_CYCLE_TO_US(ma));
                perf_stat_reset(&g_mahony); perf_stat_reset(&g_frame);
                if (++sat_idx >= 5) { sat_idx = 0; printf("# --- sweep restart ---\n"); }
            }
            if (ds >= DOWNSAMPLE_N) { ds = 0; tick++; board_led_toggle(); }
#else
            /* MODE 0: identical output to v1 + cycle stats every 200 frames */
            if (ds >= DOWNSAMPLE_N) { ds = 0;
                if (tick > 0 && tick % 200 == 0) {
                    float ma = g_mahony.count ? (float)(g_mahony.sum/g_mahony.count) : 0;
                    float fa = g_frame.count  ? (float)(g_frame.sum/g_frame.count)   : 0;
                    print_nb("#CYC m:min=%lu avg=%.1f max=%lu (%luus) | f:min=%lu avg=%.1f max=%lu (%luus)\n",
                        (unsigned long)g_mahony.min, ma, (unsigned long)g_mahony.max,
                        (unsigned long)(g_mahony.max/480),
                        (unsigned long)g_frame.min, fa, (unsigned long)g_frame.max,
                        (unsigned long)(g_frame.max/480));
                    perf_stat_reset(&g_mahony); perf_stat_reset(&g_frame);
                }
                print_nb("%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%.4f,%.4f,%.4f,%.4f\n",
                         ax, ay, az, gx, gy, gz, ms.roll, ms.pitch, ms.yaw, ms.q0, ms.q1, ms.q2, ms.q3);
                tick++; board_led_toggle(); }
#endif
        }

        if (got) { wd = mchtmr_get_count(HPM_MCHTMR); }
        else if (mchtmr_get_count(HPM_MCHTMR) - wd > WATCHDOG_TICKS) {
            err++; printf("# !TIMEOUT,err=%lu\n", err);
            mpu6050_stop_acquisition(); mpu6050_i2c_bus_recovery(&i2c_ctx);
            hpm_i2c_initialize(&i2c_ctx);
            dmamux_config(BOARD_APP_I2C_DMAMUX, I2C_DMAMUX_CH, BOARD_APP_I2C_DMA_SRC, true);
#if TEST_MODE == 4
            sat_idx = 0; sat_n = 0;
#endif
            mpu6050_start_acquisition(&imu, &ring);
            imu.frame_count = 0; wd = mchtmr_get_count(HPM_MCHTMR); imu.dma_error = false;
        }
    }
}
