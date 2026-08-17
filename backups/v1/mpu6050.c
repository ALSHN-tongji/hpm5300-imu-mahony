/*
 * Copyright (c) 2024-2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * MPU6050 IMU driver — DMA self-loop + INT-gated data validation
 *
 * Architecture:
 *   DMA runs continuously (TX→RX→TX→RX...), keeping the I2C bus
 *   active at all times — this eliminates the idle-gap bus glitches
 *   that plague pure INT-driven schemes.
 *
 *   On every RX completion, the MPU6050 INT pin (PA31) is checked:
 *     INT=LOW  → new 1kHz sample → push to ring buffer
 *     INT=HIGH → duplicate read   → discard
 *
 *   MPU6050 ODR=1kHz, INT latched low until data read, cleared by read.
 *   Effective data rate: exactly 1kHz, aligned to MPU6050's internal clock.
 */

#include "board.h"
#include "mpu6050.h"
#include "hpm_dmav2_drv.h"
#include "hpm_dmamux_drv.h"
#include "hpm_gpio_drv.h"
#include "hpm_l1c_drv.h"
#include "hpm_mchtmr_drv.h"

#define I2C_DMAMUX_CH  DMA_SOC_CHN_TO_DMAMUX_CHN(BOARD_APP_I2C_DMA, MPU6050_DMA_CH)

static uint8_t dma_tx_buf[1] ATTR_PLACE_AT_NONCACHEABLE;
static uint8_t dma_rx_buf[MPU6050_DATA_LEN] ATTR_PLACE_AT_NONCACHEABLE;

/* ── ISR-driven acquisition state ── */
static mpu6050_t     *g_imu_dev;
static mpu6050_ring_t *g_imu_ring;
static uint16_t       g_mag_addr = 0;  /* AK8963 I2C addr, 0 = not found */

typedef enum {
    IMU_DMA_IDLE,
    IMU_DMA_TX,
    IMU_DMA_RX
} imu_dma_phase_t;
static volatile imu_dma_phase_t g_dma_phase = IMU_DMA_IDLE;

/* ========================================================================
 * I2C bus recovery
 * ======================================================================== */
void mpu6050_i2c_bus_recovery(hpm_i2c_context_t *i2c_ctx)
{
    (void)i2c_ctx;
    const uint32_t scl_ioc  = IOC_PAD_PB02;
    const uint32_t sda_ioc  = IOC_PAD_PB03;
    const uint32_t port     = GPIO_GET_PORT_INDEX(scl_ioc);
    const uint8_t  scl_pin  = GPIO_GET_PIN_INDEX(scl_ioc);
    const uint8_t  sda_pin  = GPIO_GET_PIN_INDEX(sda_ioc);

    HPM_IOC->PAD[scl_ioc].FUNC_CTL = IOC_PB02_FUNC_CTL_GPIO_B_02;
    HPM_IOC->PAD[scl_ioc].PAD_CTL  = IOC_PAD_PAD_CTL_OD_SET(1)
                                   | IOC_PAD_PAD_CTL_PE_SET(1)
                                   | IOC_PAD_PAD_CTL_PS_SET(1);

    HPM_IOC->PAD[sda_ioc].FUNC_CTL = IOC_PB03_FUNC_CTL_GPIO_B_03;
    HPM_IOC->PAD[sda_ioc].PAD_CTL  = IOC_PAD_PAD_CTL_OD_SET(1)
                                   | IOC_PAD_PAD_CTL_PE_SET(1)
                                   | IOC_PAD_PAD_CTL_PS_SET(1);

    gpio_set_pin_output_with_initial(HPM_GPIO0, port, scl_pin, 1);
    gpio_set_pin_input(HPM_GPIO0, port, sda_pin);

    for (int i = 0; i < 9; i++) {
        gpio_write_pin(HPM_GPIO0, port, scl_pin, 0);
        board_delay_us(10);
        gpio_write_pin(HPM_GPIO0, port, scl_pin, 1);
        board_delay_us(10);
        if (gpio_read_pin(HPM_GPIO0, port, sda_pin)) break;
    }

    gpio_set_pin_output(HPM_GPIO0, port, sda_pin);
    gpio_write_pin(HPM_GPIO0, port, sda_pin, 0);
    board_delay_us(5);
    gpio_write_pin(HPM_GPIO0, port, scl_pin, 1);
    board_delay_us(5);
    gpio_write_pin(HPM_GPIO0, port, sda_pin, 1);
    board_delay_us(5);

    i2c_reset(BOARD_APP_I2C_BASE);
    init_i2c0_pins();
}

/* ========================================================================
 * Blocking helpers (init only — not used during ISR-driven acquisition)
 * ======================================================================== */
static hpm_stat_t imu_write_reg(I2C_Type *i2c_ptr, uint16_t dev_addr,
                                 uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_write(i2c_ptr, dev_addr, buf, 2);
}

static hpm_stat_t imu_read_regs(I2C_Type *i2c_ptr, uint16_t dev_addr,
                                 uint8_t reg, uint8_t *data, uint32_t len)
{
    uint8_t reg_buf[1] = { reg };
    hpm_stat_t stat = i2c_master_write(i2c_ptr, dev_addr, reg_buf, 1);
    if (stat != status_success) return stat;
    return i2c_master_read(i2c_ptr, dev_addr, data, len);
}

static float gyro_sensitivity(mpu6050_gyro_fs_t fs)
{
    switch (fs) {
    case MPU_GYRO_FS_250:   return 131.0f;
    case MPU_GYRO_FS_500:   return 65.5f;
    case MPU_GYRO_FS_1000:  return 32.8f;
    case MPU_GYRO_FS_2000:  return 16.4f;
    default:                return 131.0f;
    }
}

static float accel_sensitivity(mpu6050_accel_fs_t fs)
{
    switch (fs) {
    case MPU_ACCEL_FS_2G:   return 16384.0f;
    case MPU_ACCEL_FS_4G:   return 8192.0f;
    case MPU_ACCEL_FS_8G:   return 4096.0f;
    case MPU_ACCEL_FS_16G:  return 2048.0f;
    default:                return 16384.0f;
    }
}

static void parse_raw(const uint8_t *buf, mpu6050_raw_frame_t *raw)
{
    raw->accel_x = (int16_t)((buf[0]  << 8) | buf[1]);
    raw->accel_y = (int16_t)((buf[2]  << 8) | buf[3]);
    raw->accel_z = (int16_t)((buf[4]  << 8) | buf[5]);
    raw->temp    = (int16_t)((buf[6]  << 8) | buf[7]);
    raw->gyro_x  = (int16_t)((buf[8]  << 8) | buf[9]);
    raw->gyro_y  = (int16_t)((buf[10] << 8) | buf[11]);
    raw->gyro_z  = (int16_t)((buf[12] << 8) | buf[13]);
}

static void push_to_ring(const mpu6050_raw_frame_t *raw,
                         mpu6050_t *dev, mpu6050_ring_t *ring)
{
    /* Soft dedup: ~2.5kHz DMA reads but MPU6050 ODR is 1kHz. */
    static mpu6050_raw_frame_t last;
    static bool               skip_one;
    if (raw->accel_x == last.accel_x && raw->accel_y == last.accel_y &&
        raw->accel_z == last.accel_z && raw->gyro_x  == last.gyro_x  &&
        raw->gyro_y  == last.gyro_y  && raw->gyro_z  == last.gyro_z) {
        if (!skip_one) { skip_one = true;  return; }
    }
    skip_one = false;
    last = *raw;

    uint32_t head = ring->head;
    uint32_t next = (head + 1) % MPU6050_RING_DEPTH;

    if (next == ring->tail) {
        dev->drop_count++;
        return;
    }

    mpu6050_data_t *dst = &ring->frames[head];
    dst->raw[RAW_AX] = raw->accel_x;
    dst->raw[RAW_AY] = raw->accel_y;
    dst->raw[RAW_AZ] = raw->accel_z;
    dst->raw[RAW_GX] = raw->gyro_x;
    dst->raw[RAW_GY] = raw->gyro_y;
    dst->raw[RAW_GZ] = raw->gyro_z;
    dst->raw[RAW_TEMP] = raw->temp;
    dst->capture_tick = mchtmr_get_count(HPM_MCHTMR);

    ring->head = next;
}

/* ========================================================================
 * DMA helpers
 * ======================================================================== */
static hpm_stat_t dma_setup_i2c_tx(uint32_t src, uint32_t size)
{
    dma_handshake_config_t cfg;
    dma_default_handshake_config(BOARD_APP_I2C_DMA, &cfg);
    cfg.ch_index      = MPU6050_DMA_CH;
    cfg.dst           = (uint32_t)&BOARD_APP_I2C_BASE->DATA;
    cfg.dst_fixed     = true;
    cfg.src           = src;
    cfg.src_fixed     = false;
    cfg.data_width    = DMA_TRANSFER_WIDTH_BYTE;
    cfg.size_in_byte  = size;
    cfg.interrupt_mask = DMA_INTERRUPT_MASK_HALF_TC;
    return dma_setup_handshake(BOARD_APP_I2C_DMA, &cfg, true);
}

static hpm_stat_t dma_setup_i2c_rx(uint32_t dst, uint32_t size)
{
    dma_handshake_config_t cfg;
    dma_default_handshake_config(BOARD_APP_I2C_DMA, &cfg);
    cfg.ch_index      = MPU6050_DMA_CH;
    cfg.dst           = dst;
    cfg.dst_fixed     = false;
    cfg.src           = (uint32_t)&BOARD_APP_I2C_BASE->DATA;
    cfg.src_fixed     = true;
    cfg.data_width    = DMA_TRANSFER_WIDTH_BYTE;
    cfg.size_in_byte  = size;
    cfg.interrupt_mask = DMA_INTERRUPT_MASK_HALF_TC;
    return dma_setup_handshake(BOARD_APP_I2C_DMA, &cfg, true);
}

/* ── Start TX phase (called from ISR and from start_acquisition) ── */
static void imu_dma_start_tx(I2C_Type *i2c, uint16_t dev_addr)
{
    g_dma_phase = IMU_DMA_TX;
    dma_tx_buf[0] = MPU6050_REG_ACCEL_XOUT_H;
    dma_setup_i2c_tx(
        core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)dma_tx_buf), 1);
    i2c_master_start_dma_write(i2c, dev_addr, 1);
}

/* ── Start RX phase (called from ISR only) ── */
static void imu_dma_start_rx(I2C_Type *i2c, uint16_t dev_addr)
{
    g_dma_phase = IMU_DMA_RX;
    dma_setup_i2c_rx(
        core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)dma_rx_buf),
        MPU6050_DATA_LEN);
    i2c_master_start_dma_read(i2c, dev_addr, MPU6050_DATA_LEN);
}

/* ── DMA ISR: self-loop transport + INT-gated data validation ── */
SDK_DECLARE_EXT_ISR_M(IRQn_HDMA, imu_dma_isr)
void imu_dma_isr(void)
{
    uint32_t status = dma_check_transfer_status(BOARD_APP_I2C_DMA, MPU6050_DMA_CH);

    if (status & DMA_CHANNEL_STATUS_TC) {
        if (g_dma_phase != IMU_DMA_IDLE) {
            I2C_Type *i2c  = g_imu_dev->i2c_ctx->base;
            uint16_t  addr = g_imu_dev->dev_addr;

            i2c_clear_status(i2c, i2c_get_status(i2c));
            i2c_dma_disable(i2c);

            if (g_dma_phase == IMU_DMA_TX) {
                imu_dma_start_rx(i2c, addr);
            } else {
                /* RX complete: push every frame, soft-dedup in push_to_ring */
                mpu6050_raw_frame_t raw;
                parse_raw(dma_rx_buf, &raw);
                push_to_ring(&raw, g_imu_dev, g_imu_ring);
                g_imu_dev->frame_count++;
                /* Chain next TX — bus never idle = no glitches */
                imu_dma_start_tx(i2c, addr);
            }
        }
    }

    if (status & (DMA_CHANNEL_STATUS_ERROR | DMA_CHANNEL_STATUS_ABORT)) {
        g_imu_dev->dma_error = true;
        g_dma_phase = IMU_DMA_IDLE;
    }
}

/* ========================================================================
 * Public API
 * ======================================================================== */

hpm_stat_t mpu6050_init(mpu6050_t *dev, hpm_i2c_context_t *i2c_ctx,
                         uint16_t dev_addr,
                         mpu6050_gyro_fs_t gyro_fs,
                         mpu6050_accel_fs_t accel_fs,
                         mpu6050_dlpf_t dlpf)
{
    hpm_stat_t stat;
    I2C_Type *i2c = i2c_ctx->base;

    dev->i2c_ctx    = i2c_ctx;
    dev->dev_addr   = dev_addr;
    dev->accel_sf   = accel_sensitivity(accel_fs);
    dev->gyro_sf    = gyro_sensitivity(gyro_fs);
    dev->data_ready          = false;
    dev->frame_count          = 0;
    dev->drop_count           = 0;
    dev->reject_count         = 0;
    dev->dma_error            = false;
    dev->gyro_off_at_T0[0]    = 0;
    dev->gyro_off_at_T0[1]    = 0;
    dev->gyro_off_at_T0[2]    = 0;
    dev->T0                   = 25.0f;
    dev->gyro_calib_done      = false;
    dev->motion_warn          = false;
    dev->K_T[0] = 0.01f; dev->K_T[1] = 0.01f; dev->K_T[2] = 0.03f;
    g_imu_dev                 = dev;

    /* 1. Wake up */
    stat = imu_write_reg(i2c, dev_addr, MPU6050_REG_PWR_MGMT_1, 0x00);
    if (stat != status_success) return stat;
    board_delay_ms(100);

    /* 2. Sample rate: 0 = 1kHz */
    stat = imu_write_reg(i2c, dev_addr, MPU6050_REG_SMPLRT_DIV, 0x00);
    if (stat != status_success) return stat;

    /* 3. DLPF */
    stat = imu_write_reg(i2c, dev_addr, MPU6050_REG_CONFIG, (uint8_t)dlpf);
    if (stat != status_success) return stat;

    /* 4. Gyro FS */
    stat = imu_write_reg(i2c, dev_addr, MPU6050_REG_GYRO_CONFIG,
                         (uint8_t)(gyro_fs << 3));
    if (stat != status_success) return stat;

    /* 5. Accel FS */
    stat = imu_write_reg(i2c, dev_addr, MPU6050_REG_ACCEL_CONFIG,
                         (uint8_t)(accel_fs << 3));
    if (stat != status_success) return stat;

    /* 6. INT pin — active low, latched until data read */
    stat = imu_write_reg(i2c, dev_addr, MPU6050_REG_INT_PIN_CFG, 0x30);
    if (stat != status_success) return stat;
    stat = imu_write_reg(i2c, dev_addr, MPU6050_REG_INT_ENABLE, 0x01);
    if (stat != status_success) return stat;

    /* 7. INT pin GPIO */
    init_imu_int_pin();

    /* 8. DMAMUX */
    dmamux_config(BOARD_APP_I2C_DMAMUX, I2C_DMAMUX_CH,
                  BOARD_APP_I2C_DMA_SRC, true);

    /* 9. ID check */
    {
        uint8_t wai = 0;
        imu_read_regs(i2c, dev_addr, MPU6050_REG_WHO_AM_I, &wai, 1);
        printf("# [ID] WHO_AM_I=0x%02X (%s)\n", wai,
               wai == 0x71 ? "MPU-9250" : wai == 0x68 ? "MPU-6050" : "UNKNOWN");
    }

    return status_success;
}

void mpu6050_start_acquisition(mpu6050_t *dev, mpu6050_ring_t *ring)
{
    dev->dma_error = false;
    ring->head     = 0;
    ring->tail     = 0;
    g_imu_ring     = ring;

    intc_m_enable_irq_with_priority(IRQn_HDMA, 1);
    imu_dma_start_tx(dev->i2c_ctx->base, dev->dev_addr);
}

void mpu6050_stop_acquisition(void)
{
    g_imu_ring = NULL;
    dma_disable_channel(BOARD_APP_I2C_DMA, MPU6050_DMA_CH);
    i2c_dma_disable(BOARD_APP_I2C_BASE);
    g_dma_phase = IMU_DMA_IDLE;
}

hpm_stat_t mpu6050_who_am_i(mpu6050_t *dev, uint8_t *who_am_i)
{
    return imu_read_regs(dev->i2c_ctx->base, dev->dev_addr,
                          MPU6050_REG_WHO_AM_I, who_am_i, 1);
}

bool mpu6050_read_mag_raw(mpu6050_t *dev, int16_t *mx, int16_t *my, int16_t *mz)
{
    if (!g_mag_addr) return false;
    uint8_t buf[6];
    hpm_stat_t stat = imu_read_regs(dev->i2c_ctx->base, dev->dev_addr,
                                    MPU6050_REG_EXT_SENS_DATA_00, buf, 6);
    if (stat != status_success) return false;
    *mx = (int16_t)((uint16_t)buf[1] << 8 | buf[0]);
    *my = (int16_t)((uint16_t)buf[3] << 8 | buf[2]);
    *mz = (int16_t)((uint16_t)buf[5] << 8 | buf[4]);
    return true;
}

hpm_stat_t mpu6050_i2c_recover(hpm_i2c_context_t *i2c_ctx)
{
    dma_disable_channel(BOARD_APP_I2C_DMA, MPU6050_DMA_CH);
    i2c_dma_disable(BOARD_APP_I2C_BASE);

    mpu6050_i2c_bus_recovery(i2c_ctx);

    hpm_stat_t stat = hpm_i2c_initialize(i2c_ctx);
    if (stat != status_success) return stat;

    dmamux_config(BOARD_APP_I2C_DMAMUX,
                  DMA_SOC_CHN_TO_DMAMUX_CHN(BOARD_APP_I2C_DMA, MPU6050_DMA_CH),
                  BOARD_APP_I2C_DMA_SRC, true);

    return status_success;
}

/* ========================================================================
 * Calibration
 * ======================================================================== */
void mpu6050_calib_gyro_raw(mpu6050_t *dev, mpu6050_ring_t *ring, int raw_samples)
{
    mpu6050_data_t s;
    int64_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
    double  temp_sum = 0;
    int     n = 0;
    int16_t gx_min = 32767, gx_max = -32768;
    int16_t gy_min = 32767, gy_max = -32768;
    int16_t gz_min = 32767, gz_max = -32768;

    printf("# Gyro calib: %d samples, keep STILL...\n", raw_samples);

    printf("#  calib: probing...\n");
    ring->head = ring->tail = 0;
    dev->dma_error = false;
    mpu6050_start_acquisition(dev, ring);

    {
        uint64_t probe_start = mchtmr_get_count(HPM_MCHTMR);
        while (!mpu6050_ring_pop(ring, &s)) {
            if (mchtmr_get_count(HPM_MCHTMR) - probe_start > 24 * 1000 * 2000) {
                printf("#  calib: FAILED — sensor not responding\n");
                mpu6050_stop_acquisition();
                return;
            }
        }
        int16_t gx = s.raw[RAW_GX], gy = s.raw[RAW_GY], gz = s.raw[RAW_GZ];
        gx_sum += gx; gy_sum += gy; gz_sum += gz;
        temp_sum += s.raw[RAW_TEMP] / 340.0 + 36.53;
        gx_min = gx_max = gx;
        gy_min = gy_max = gy;
        gz_min = gz_max = gz;
        n = 1;
    }
    printf("#  calib: OK, collecting...\n");

    uint64_t wd_last_tick = mchtmr_get_count(HPM_MCHTMR);
    int      wd_last_n    = n;
    while (n < raw_samples) {
        while (mpu6050_ring_pop(ring, &s)) {
            int16_t gx = s.raw[RAW_GX], gy = s.raw[RAW_GY], gz = s.raw[RAW_GZ];
            gx_sum += gx; gy_sum += gy; gz_sum += gz;
            temp_sum += s.raw[RAW_TEMP] / 340.0 + 36.53;
            if (gx < gx_min) gx_min = gx;
            if (gx > gx_max) gx_max = gx;
            if (gy < gy_min) gy_min = gy;
            if (gy > gy_max) gy_max = gy;
            if (gz < gz_min) gz_min = gz;
            if (gz > gz_max) gz_max = gz;
            n++;
        }
        if (n != wd_last_n) {
            wd_last_n    = n;
            wd_last_tick = mchtmr_get_count(HPM_MCHTMR);
        } else if (mchtmr_get_count(HPM_MCHTMR) - wd_last_tick > 24 * 1000 * 500) {
            printf("#  calib: timeout, recovering...\n");
            mpu6050_stop_acquisition();
            mpu6050_i2c_recover(dev->i2c_ctx);
            ring->head = ring->tail = 0;
            dev->dma_error = false;
            mpu6050_start_acquisition(dev, ring);
            wd_last_tick = mchtmr_get_count(HPM_MCHTMR);
        }
        if ((n % 200) == 0) {
            printf("#  calib: %d/%d\n", n, raw_samples);
        }
    }

    mpu6050_stop_acquisition();

    int16_t spread_x = (int16_t)(gx_max - gx_min);
    int16_t spread_y = (int16_t)(gy_max - gy_min);
    int16_t spread_z = (int16_t)(gz_max - gz_min);
    printf("#  calib: spread(raw)=%d,%d,%d  (%.2f,%.2f,%.2f °/s)\n",
           spread_x, spread_y, spread_z,
           spread_x/dev->gyro_sf, spread_y/dev->gyro_sf, spread_z/dev->gyro_sf);

    if (spread_x > 200 || spread_y > 200 || spread_z > 200) {
        printf("# CALIB FAILED — motion detected!\n");
        dev->motion_warn = true;
    }

    dev->gyro_off_at_T0[0] = (int16_t)(gx_sum / raw_samples);
    dev->gyro_off_at_T0[1] = (int16_t)(gy_sum / raw_samples);
    dev->gyro_off_at_T0[2] = (int16_t)(gz_sum / raw_samples);
    dev->T0            = (float)(temp_sum / raw_samples);
    dev->gyro_calib_done = true;

    printf("# T0 = %.2f C\n", dev->T0);
    printf("# Gyro off@T0 (raw): %d, %d, %d\n",
           dev->gyro_off_at_T0[0], dev->gyro_off_at_T0[1], dev->gyro_off_at_T0[2]);
    printf("# Gyro off@T0 (°/s): %.3f, %.3f, %.3f\n",
           dev->gyro_off_at_T0[0] / dev->gyro_sf,
           dev->gyro_off_at_T0[1] / dev->gyro_sf,
           dev->gyro_off_at_T0[2] / dev->gyro_sf);
}

/* ========================================================================
 * 6-face accelerometer calibration
 * ======================================================================== */

#define CALIB_6FACE_SAMPLES  300

static void calib_collect_face(mpu6050_ring_t *ring, int n,
                                double *sum_x, double *sum_y, double *sum_z)
{
    mpu6050_data_t s;
    int cnt = 0;
    *sum_x = *sum_y = *sum_z = 0.0;
    while (cnt < n) {
        while (mpu6050_ring_pop(ring, &s) && cnt < n) {
            *sum_x += s.raw[RAW_AX];
            *sum_y += s.raw[RAW_AY];
            *sum_z += s.raw[RAW_AZ];
            cnt++;
        }
    }
}

void mpu6050_calib_accel_6face(mpu6050_t *dev, mpu6050_ring_t *ring,
                                float *ox, float *oy, float *oz,
                                float *sx, float *sy, float *sz)
{
    double mean[6][3];
    double sf = dev->accel_sf;

    printf("\n# 6-FACE ACCEL CALIBRATION\n");
    printf("# Put board on ANY 6 different sides, one at a time.\n");
    printf("# Order doesn't matter. Keep STILL.\n");

    mpu6050_start_acquisition(dev, ring);

    for (int f = 0; f < 6; f++) {
        printf("\n# -- Face %d/6: Place board on a NEW side, then press ENTER --\n", f+1);
        getchar();

        mpu6050_data_t dummy;
        while (mpu6050_ring_pop(ring, &dummy)) {}

        double sx, sy, sz;
        calib_collect_face(ring, CALIB_6FACE_SAMPLES, &sx, &sy, &sz);
        mean[f][0] = sx / CALIB_6FACE_SAMPLES;
        mean[f][1] = sy / CALIB_6FACE_SAMPLES;
        mean[f][2] = sz / CALIB_6FACE_SAMPLES;

        double aa = (mean[f][0] > 0 ? mean[f][0] : -mean[f][0]);
        double bb = (mean[f][1] > 0 ? mean[f][1] : -mean[f][1]);
        double cc = (mean[f][2] > 0 ? mean[f][2] : -mean[f][2]);
        char axis = (aa > bb && aa > cc) ? 'X' : (bb > cc ? 'Y' : 'Z');
        char dir  = (axis == 'X') ? (mean[f][0] > 0 ? '+' : '-')
                  : (axis == 'Y') ? (mean[f][1] > 0 ? '+' : '-')
                  :                 (mean[f][2] > 0 ? '+' : '-');
        printf("#   -> Detected: %c%c  (%.0f, %.0f, %.0f)\n",
               axis, dir, mean[f][0], mean[f][1], mean[f][2]);
    }

    mpu6050_stop_acquisition();

    int bad = 0;
    double raw_max[3], raw_min[3];
    for (int a = 0; a < 3; a++) {
        raw_max[a] = mean[0][a]; raw_min[a] = mean[0][a];
        for (int f = 1; f < 6; f++) {
            if (mean[f][a] > raw_max[a]) raw_max[a] = mean[f][a];
            if (mean[f][a] < raw_min[a]) raw_min[a] = mean[f][a];
        }
        double spread = raw_max[a] - raw_min[a];
        if (spread < 1.4 * sf) {
            printf("# !! Axis %c: spread=%.0f (expected ~%.0f) -- missing a face?\n",
                   'X'+a, spread, 2.0*sf);
            bad = 1;
        }
    }
    if (bad) {
        printf("# Calibration FAILED -- re-run and cover ALL 6 sides.\n");
        *ox = *oy = *oz = 0;
        *sx = *sy = *sz = 1.0f;
        return;
    }

    *ox = (float)((raw_max[0] + raw_min[0]) / 2.0);
    *oy = (float)((raw_max[1] + raw_min[1]) / 2.0);
    *oz = (float)((raw_max[2] + raw_min[2]) / 2.0);
    *sx = (float)(2.0 * sf / (raw_max[0] - raw_min[0]));
    *sy = (float)(2.0 * sf / (raw_max[1] - raw_min[1]));
    *sz = (float)(2.0 * sf / (raw_max[2] - raw_min[2]));

    printf("\n# CALIBRATION RESULT\n");
    printf("# OX=%.1f OY=%.1f OZ=%.1f\n", *ox, *oy, *oz);
    printf("# SX=%.6f SY=%.6f SZ=%.6f\n", *sx, *sy, *sz);
    printf("# Copy to main.c lines 77-78:\n");
    printf("# const float OX=%.1ff, OY=%.1ff, OZ=%.1ff;\n", *ox, *oy, *oz);
    printf("# const float SX=%ff, SY=%ff, SZ=%ff;\n", *sx, *sy, *sz);
}

bool mpu6050_ring_pop(mpu6050_ring_t *ring, mpu6050_data_t *out)
{
    intc_m_disable_irq(IRQn_HDMA);
    uint32_t head = ring->head;
    uint32_t tail = ring->tail;
    intc_m_enable_irq(IRQn_HDMA);

    if (head == tail) {
        return false;
    }
    if (out) {
        *out = ring->frames[tail];
    }
    ring->tail = (tail + 1) % MPU6050_RING_DEPTH;
    return true;
}

/* ========================================================================
 * Online temperature drift calibration
 * ======================================================================== */

#define TC_LAMBDA       0.995f
#define TC_MIN_SPREAD    3.0f
#define TC_MIN_SAMPLES   200.0f

void temp_calib_init(temp_calib_t *tc)
{
    tc->active  = true;
    tc->lambda  = TC_LAMBDA;
    tc->n_eff   = 0.0f;
    tc->sum_dt  = 0.0f;
    tc->sum_dt2 = 0.0f;
    for (int i = 0; i < 3; i++) {
        tc->sum_bias[i]    = 0.0f;
        tc->sum_dt_bias[i] = 0.0f;
    }
    tc->t_min =  999.0f;
    tc->t_max = -999.0f;
    tc->update_count = 0;
}

bool temp_calib_collect(temp_calib_t *tc, float temp, float gx_bias,
                         float gy_bias, float gz_bias)
{
    if (!tc->active) return false;

    float dt = temp;
    float bias[3] = { gx_bias, gy_bias, gz_bias };
    float lam = tc->lambda;

    tc->n_eff   = lam * tc->n_eff   + 1.0f;
    tc->sum_dt  = lam * tc->sum_dt  + dt;
    tc->sum_dt2 = lam * tc->sum_dt2 + dt * dt;

    for (int i = 0; i < 3; i++) {
        tc->sum_bias[i]    = lam * tc->sum_bias[i]    + bias[i];
        tc->sum_dt_bias[i] = lam * tc->sum_dt_bias[i] + dt * bias[i];
    }

    if (dt < tc->t_min) tc->t_min = dt;
    if (dt > tc->t_max) tc->t_max = dt;

    return true;
}

bool temp_calib_update(temp_calib_t *tc, mpu6050_t *dev)
{
    if (!tc->active) return false;

    float spread = tc->t_max - tc->t_min;
    if (tc->n_eff < TC_MIN_SAMPLES)  return false;
    if (spread  < TC_MIN_SPREAD)     return false;

    float n  = tc->n_eff;
    float sx = tc->sum_dt;
    float sy, sxy;

    for (int i = 0; i < 3; i++) {
        sy  = tc->sum_bias[i];
        sxy = tc->sum_dt_bias[i];

        float denom = n * tc->sum_dt2 - sx * sx;
        if (denom < 0.001f) continue;

        float slope = (n * sxy - sx * sy) / denom;
        dev->K_T[i] = slope;

        float intercept = (sy - slope * sx) / n;
        dev->gyro_off_at_T0[i] = (int16_t)(intercept * dev->gyro_sf);
    }

    tc->update_count++;
    printf("# T-calib #%lu: K_T=%.4f,%.4f,%.4f °/s/°C  (spread=%.1f°C, n=%.0f)\n",
           tc->update_count,
           dev->K_T[0], dev->K_T[1], dev->K_T[2],
           spread, (double)n);

    return true;
}
