/*
 * MPU6500 — DMA self-loop + INT-gated push
 *
 * DMA runs TX→RX→TX→RX continuously (self-loop).
 * On each RX, INT pin (PA31) is polled:
 *   INT=LOW → new 1kHz sample → push to ring buffer
 *   INT=HIGH → stale/duplicate → push anyway (robust fallback)
 *
 * INT is the data-ready signal — it determines whether data is fresh.
 * DMA is the transport layer — it always runs.
 * Main loop pops every frame, runs Mahony, prints downsampled.
 */

#include "board.h"
#include "mpu6500.h"
#include "hpm_dmav2_drv.h"
#include "hpm_dmamux_drv.h"
#include "hpm_gpio_drv.h"
#include "hpm_l1c_drv.h"
#include "hpm_mchtmr_drv.h"

#define I2C_DMAMUX_CH  DMA_SOC_CHN_TO_DMAMUX_CHN(BOARD_APP_I2C_DMA, 0)

static mpu6500_t       *g_dev;
static mpu6500_ring_t   *g_ring;
static uint8_t           g_tx_buf[1] ATTR_PLACE_AT_NONCACHEABLE;
static uint8_t           g_rx_buf[MPU6500_DATA_LEN] ATTR_PLACE_AT_NONCACHEABLE;
static volatile bool     g_is_rx = false;  /* false=TX phase, true=RX phase */
static volatile uint32_t g_int_frames = 0; /* frames pushed while INT was LOW */

static inline bool int_pin_is_low(void) {
    return !gpio_read_pin(BOARD_IMU_INT_GPIO_CTRL,
                          BOARD_IMU_INT_GPIO_INDEX,
                          BOARD_IMU_INT_GPIO_PIN);
}

/* ── DMA ISR: self-loop TX→RX→TX→RX ── */
SDK_DECLARE_EXT_ISR_M(IRQn_HDMA, mpu6500_dma_isr)
void mpu6500_dma_isr(void)
{
    uint32_t st = dma_check_transfer_status(BOARD_APP_I2C_DMA, 0);
    dma_clear_transfer_status(BOARD_APP_I2C_DMA, 0);

    if (st & DMA_CHANNEL_STATUS_TC) {
        I2C_Type  *i2c  = g_dev->i2c_ctx->base;
        uint16_t   addr = g_dev->dev_addr;

        i2c->STATUS = I2C_STATUS_CMPL_MASK;
        i2c->SETUP &= ~I2C_SETUP_DMAEN_MASK;

        if (!g_is_rx) {
            /* TX done (no STOP) → start RX with Repeated Start */
            g_is_rx = true;
            dma_handshake_config_t c;
            dma_default_handshake_config(BOARD_APP_I2C_DMA, &c);
            c.ch_index       = 0;
            c.dst            = core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)g_rx_buf);
            c.dst_fixed      = false;
            c.src            = (uint32_t)&i2c->DATA;
            c.src_fixed      = true;
            c.data_width     = DMA_TRANSFER_WIDTH_BYTE;
            c.size_in_byte   = MPU6500_DATA_LEN;
            c.interrupt_mask = DMA_INTERRUPT_MASK_HALF_TC;
            dma_setup_handshake(BOARD_APP_I2C_DMA, &c, true);

            /* Repeated Start + read + STOP (releases bus after read) */
            i2c->CTRL = I2C_CTRL_PHASE_START_MASK   /* Repeated Start */
                      | I2C_CTRL_PHASE_STOP_MASK     /* STOP after read */
                      | I2C_CTRL_PHASE_ADDR_MASK
                      | I2C_CTRL_PHASE_DATA_MASK
                      | I2C_CTRL_DIR_SET(I2C_DIR_MASTER_READ)
                      | I2C_CTRL_DATACNT_SET(I2C_DATACNT_MAP(MPU6500_DATA_LEN));
            i2c->SETUP |= I2C_SETUP_DMAEN_MASK;
            i2c->CMD = I2C_CMD_ISSUE_DATA_TRANSMISSION;
        } else {
            /* RX done → parse, push, chain next TX */
            g_is_rx = false;

            mpu6500_data_t d;
            uint8_t *b = g_rx_buf;
            d.raw[RAW_AX] = (int16_t)((b[0]  << 8) | b[1]);
            d.raw[RAW_AY] = (int16_t)((b[2]  << 8) | b[3]);
            d.raw[RAW_AZ] = (int16_t)((b[4]  << 8) | b[5]);
            int16_t tr    = (int16_t)((b[6]  << 8) | b[7]);
            d.raw[RAW_GX] = (int16_t)((b[8]  << 8) | b[9]);
            d.raw[RAW_GY] = (int16_t)((b[10] << 8) | b[11]);
            d.raw[RAW_GZ] = (int16_t)((b[12] << 8) | b[13]);
            d.accel_x = d.raw[RAW_AX] / g_dev->accel_sf;
            d.accel_y = d.raw[RAW_AY] / g_dev->accel_sf;
            d.accel_z = d.raw[RAW_AZ] / g_dev->accel_sf;
            d.gyro_x  = d.raw[RAW_GX] / g_dev->gyro_sf;
            d.gyro_y  = d.raw[RAW_GY] / g_dev->gyro_sf;
            d.gyro_z  = d.raw[RAW_GZ] / g_dev->gyro_sf;
            d.temp    = tr / 340.0f + 36.53f;

            /* Track INT-low frames */
            if (int_pin_is_low()) g_int_frames++;

            if (g_ring) {
                uint32_t nx = (g_ring->head + 1) % MPU6500_RING_DEPTH;
                if (nx != g_ring->tail) {
                    g_ring->frames[g_ring->head] = d;
                    g_ring->head = nx;
                }
                g_dev->frame_count++;
            }

            /* Chain: start TX (write reg addr, no STOP → bus stays held) */
            g_tx_buf[0] = MPU6500_REG_ACCEL_XOUT_H;
            dma_handshake_config_t c;
            dma_default_handshake_config(BOARD_APP_I2C_DMA, &c);
            c.ch_index       = 0;
            c.dst            = (uint32_t)&i2c->DATA;
            c.dst_fixed      = true;
            c.src            = core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)g_tx_buf);
            c.src_fixed      = false;
            c.data_width     = DMA_TRANSFER_WIDTH_BYTE;
            c.size_in_byte   = 1;
            c.interrupt_mask = DMA_INTERRUPT_MASK_HALF_TC;
            dma_setup_handshake(BOARD_APP_I2C_DMA, &c, true);

            /* START + write + NO STOP → bus held for next Repeated Start read */
            i2c->CTRL = I2C_CTRL_PHASE_START_MASK
                      /* NO PHASE_STOP — bus stays held */
                      | I2C_CTRL_PHASE_ADDR_MASK
                      | I2C_CTRL_PHASE_DATA_MASK
                      | I2C_CTRL_DIR_SET(I2C_DIR_MASTER_WRITE)
                      | I2C_CTRL_DATACNT_SET(I2C_DATACNT_MAP(1));
            i2c->SETUP |= I2C_SETUP_DMAEN_MASK;
            i2c->CMD = I2C_CMD_ISSUE_DATA_TRANSMISSION;
        }
    }

    if (st & (DMA_CHANNEL_STATUS_ERROR | DMA_CHANNEL_STATUS_ABORT)) {
        g_dev->dma_error = true;
    }
}

/* ======================================================================== */
void mpu6500_i2c_bus_recovery(hpm_i2c_context_t *ctx)
{
    (void)ctx;
    uint32_t sc = IOC_PAD_PB02, sd = IOC_PAD_PB03;
    uint32_t p = GPIO_GET_PORT_INDEX(sc);
    uint8_t sp = GPIO_GET_PIN_INDEX(sc), dp = GPIO_GET_PIN_INDEX(sd);

    HPM_IOC->PAD[sc].FUNC_CTL = IOC_PB02_FUNC_CTL_GPIO_B_02;
    HPM_IOC->PAD[sc].PAD_CTL  = IOC_PAD_PAD_CTL_OD_SET(1) | IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(1);
    HPM_IOC->PAD[sd].FUNC_CTL = IOC_PB03_FUNC_CTL_GPIO_B_03;
    HPM_IOC->PAD[sd].PAD_CTL  = IOC_PAD_PAD_CTL_OD_SET(1) | IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(1);
    gpio_set_pin_output_with_initial(HPM_GPIO0, p, sp, 1);
    gpio_set_pin_input(HPM_GPIO0, p, dp);
    for (int i = 0; i < 9; i++) {
        gpio_write_pin(HPM_GPIO0, p, sp, 0); board_delay_us(10);
        gpio_write_pin(HPM_GPIO0, p, sp, 1); board_delay_us(10);
        if (gpio_read_pin(HPM_GPIO0, p, dp)) break;
    }
    gpio_set_pin_output(HPM_GPIO0, p, dp); gpio_write_pin(HPM_GPIO0, p, dp, 0);
    board_delay_us(5); gpio_write_pin(HPM_GPIO0, p, sp, 1); board_delay_us(5);
    gpio_write_pin(HPM_GPIO0, p, dp, 1); board_delay_us(5);
    i2c_reset(BOARD_APP_I2C_BASE); init_i2c0_pins();
}

static hpm_stat_t write_reg(I2C_Type *i2c, uint16_t dev, uint8_t reg, uint8_t val) {
    uint8_t b[2]={reg,val}; return i2c_master_write(i2c,dev,b,2);
}
static hpm_stat_t read_regs(I2C_Type *i2c, uint16_t dev, uint8_t reg, uint8_t *d, uint32_t n) {
    uint8_t rb[1]={reg}; hpm_stat_t s=i2c_master_write(i2c,dev,rb,1);
    if(s!=status_success)return s; return i2c_master_read(i2c,dev,d,n);
}
uint8_t mpu6500_read_reg(mpu6500_t *dev, uint8_t reg) {
    uint8_t v=0; read_regs(dev->i2c_ctx->base,dev->dev_addr,reg,&v,1); return v;
}

/* ======================================================================== */
hpm_stat_t mpu6500_init(mpu6500_t *dev, hpm_i2c_context_t *i2c_ctx, uint16_t addr,
                         mpu6500_gyro_fs_t gfs, mpu6500_accel_fs_t afs, mpu6500_dlpf_t dlpf)
{
    I2C_Type *i2c = i2c_ctx->base;
    dev->i2c_ctx=i2c_ctx; dev->dev_addr=addr;
    dev->frame_count=0; dev->drop_count=0; dev->dma_error=false; dev->calib_done=false;
    dev->gyro_off[0]=dev->gyro_off[1]=dev->gyro_off[2]=0; dev->T0=25.0f;
    switch(gfs){case MPU_GYRO_FS_250:dev->gyro_sf=131.0f;break;case MPU_GYRO_FS_500:dev->gyro_sf=65.5f;break;case MPU_GYRO_FS_1000:dev->gyro_sf=32.8f;break;default:dev->gyro_sf=16.4f;break;}
    switch(afs){case MPU_ACCEL_FS_2G:dev->accel_sf=16384.0f;break;case MPU_ACCEL_FS_4G:dev->accel_sf=8192.0f;break;case MPU_ACCEL_FS_8G:dev->accel_sf=4096.0f;break;default:dev->accel_sf=2048.0f;break;}

    g_dev=dev; g_ring=NULL; g_is_rx=false; g_int_frames=0;

    write_reg(i2c,addr,MPU6500_REG_PWR_MGMT_1,0x00); board_delay_ms(100);
    write_reg(i2c,addr,MPU6500_REG_SMPLRT_DIV,0x00);
    write_reg(i2c,addr,MPU6500_REG_CONFIG,(uint8_t)dlpf);
    write_reg(i2c,addr,MPU6500_REG_GYRO_CONFIG,(uint8_t)(gfs<<3));
    write_reg(i2c,addr,MPU6500_REG_ACCEL_CONFIG,(uint8_t)(afs<<3));
    write_reg(i2c,addr,MPU6500_REG_INT_PIN_CFG,0x30);
    write_reg(i2c,addr,MPU6500_REG_INT_ENABLE,0x01);
    init_imu_int_pin();
    dmamux_config(BOARD_APP_I2C_DMAMUX,I2C_DMAMUX_CH,BOARD_APP_I2C_DMA_SRC,true);
    return status_success;
}

void mpu6500_start_acquisition(mpu6500_ring_t *ring) {
    g_ring=ring; g_ring->head=g_ring->tail=0;
    g_dev->dma_error=false; g_dev->drop_count=0;
    g_is_rx=false; g_int_frames=0;
    intc_m_enable_irq_with_priority(IRQn_HDMA,1);

    /* Kick off first TX */
    g_tx_buf[0]=MPU6500_REG_ACCEL_XOUT_H;
    I2C_Type *i2c=g_dev->i2c_ctx->base;
    i2c->STATUS=I2C_STATUS_CMPL_MASK;
    i2c->ADDR=I2C_ADDR_ADDR_SET(g_dev->dev_addr);
    i2c->CTRL=I2C_CTRL_PHASE_START_MASK|I2C_CTRL_PHASE_STOP_MASK|I2C_CTRL_PHASE_ADDR_MASK|I2C_CTRL_PHASE_DATA_MASK|I2C_CTRL_DIR_SET(I2C_DIR_MASTER_WRITE)|I2C_CTRL_DATACNT_SET(I2C_DATACNT_MAP(1));
    i2c->SETUP|=I2C_SETUP_DMAEN_MASK;
    dma_handshake_config_t c; dma_default_handshake_config(BOARD_APP_I2C_DMA,&c);
    c.ch_index=0; c.dst=(uint32_t)&i2c->DATA; c.dst_fixed=true;
    c.src=core_local_mem_to_sys_address(BOARD_RUNNING_CORE,(uint32_t)g_tx_buf);
    c.src_fixed=false; c.data_width=DMA_TRANSFER_WIDTH_BYTE; c.size_in_byte=1;
    c.interrupt_mask=DMA_INTERRUPT_MASK_HALF_TC;
    dma_setup_handshake(BOARD_APP_I2C_DMA,&c,true);
    i2c->CMD=I2C_CMD_ISSUE_DATA_TRANSMISSION;
}

void mpu6500_stop_acquisition(void) {
    dma_disable_channel(BOARD_APP_I2C_DMA,0); i2c_dma_disable(BOARD_APP_I2C_BASE); g_ring=NULL;
}

bool mpu6500_ring_pop(mpu6500_ring_t *ring, mpu6500_data_t *out) {
    if(!ring||ring->head==ring->tail)return false;
    if(out)*out=ring->frames[ring->tail];
    ring->tail=(ring->tail+1)%MPU6500_RING_DEPTH; return true;
}

/* ======================================================================== */
void mpu6500_calib_gyro(mpu6500_t *dev, mpu6500_ring_t *ring, int samples)
{
    mpu6500_data_t s; int64_t gxs=0,gys=0,gzs=0; double ts=0; int n=0;
    int16_t gxn=32767,gxx=-32768,gyn=32767,gyx=-32768,gzn=32767,gzx=-32768;

    printf("# Gyro calib: %d samples, keep STILL...\n",samples);
    mpu6500_start_acquisition(ring);

    {   uint64_t t0=mchtmr_get_count(HPM_MCHTMR);
        while(!mpu6500_ring_pop(ring,&s)){
            if(mchtmr_get_count(HPM_MCHTMR)-t0>24*1000*3000){
                printf("#  calib: FAILED\n"); mpu6500_stop_acquisition(); return; }}
        int16_t gx=s.raw[RAW_GX],gy=s.raw[RAW_GY],gz=s.raw[RAW_GZ];
        gxs+=gx;gys+=gy;gzs+=gz;ts+=s.temp;
        gxn=gxx=gx;gyn=gyx=gy;gzn=gzx=gz; n=1; }
    printf("#  calib: OK, collecting...\n");

    uint64_t wd=mchtmr_get_count(HPM_MCHTMR); int wn=n;
    while(n<samples){
        while(mpu6500_ring_pop(ring,&s)){
            int16_t gx=s.raw[RAW_GX],gy=s.raw[RAW_GY],gz=s.raw[RAW_GZ];
            gxs+=gx;gys+=gy;gzs+=gz;ts+=s.temp;
            if(gx<gxn)gxn=gx; if(gx>gxx)gxx=gx;
            if(gy<gyn)gyn=gy; if(gy>gyx)gyx=gy;
            if(gz<gzn)gzn=gz; if(gz>gzx)gzx=gz; n++;}
        if(n!=wn){wn=n; wd=mchtmr_get_count(HPM_MCHTMR);}
        else if(mchtmr_get_count(HPM_MCHTMR)-wd>24*1000*500){
            printf("#  calib: timeout recovering (n=%d,int_lo=%lu)...\n",n,g_int_frames);
            mpu6500_stop_acquisition();
            mpu6500_i2c_bus_recovery(dev->i2c_ctx); hpm_i2c_initialize(dev->i2c_ctx);
            dmamux_config(BOARD_APP_I2C_DMAMUX,I2C_DMAMUX_CH,BOARD_APP_I2C_DMA_SRC,true);
            g_int_frames=0; mpu6500_start_acquisition(ring); wd=mchtmr_get_count(HPM_MCHTMR);}
        if((n%200)==0&&n!=wn)printf("#  calib: %d/%d\n",n,samples);}
    mpu6500_stop_acquisition();

    int16_t sx=gxx-gxn,sy=gyx-gyn,sz=gzx-gzn;
    printf("#  calib: spread(raw)=%d,%d,%d (%.2f,%.2f,%.2f deg/s)\n",sx,sy,sz,sx/dev->gyro_sf,sy/dev->gyro_sf,sz/dev->gyro_sf);
    dev->gyro_off[0]=(int16_t)(gxs/samples);dev->gyro_off[1]=(int16_t)(gys/samples);
    dev->gyro_off[2]=(int16_t)(gzs/samples);dev->T0=(float)(ts/samples);dev->calib_done=true;
    printf("# T0=%.2fC Gyro off: %d,%d,%d (deg/s): %.3f,%.3f,%.3f\n",
           dev->T0,dev->gyro_off[0],dev->gyro_off[1],dev->gyro_off[2],
           dev->gyro_off[0]/dev->gyro_sf,dev->gyro_off[1]/dev->gyro_sf,dev->gyro_off[2]/dev->gyro_sf);
}
