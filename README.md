# HPM5300EVK + MPU6050 (GY-521) IMU 数据采集与可视化

## 硬件准备

### 购买清单

| 物品 | 型号/关键词 | 参考价格 |
|------|-----------|---------|
| IMU 模块 | **GY-521 (MPU6050)** 淘宝搜索 | 3-5 元 |
| 杜邦线 母对母 | 10cm 或 20cm, 4根 | 1-2 元 |

### 接线图

```
GY-521 模块        HPM5300EVK P1 接口 (40Pin 树莓派兼容)

  VCC  ──────────  Pin 1  (3.3V)    ← 红色线
  GND  ──────────  Pin 6  (GND)     ← 黑色线
  SCL  ──────────  Pin 28 (I2C0_SCL) ← PB02, 黄色线
  SDA  ──────────  Pin 27 (I2C0_SDA) ← PB03, 绿色线
  INT  ──────────  Pin 13 (PA31)     ← 蓝色线（数据就绪中断）
```

> **P1 接口位置**：开发板中间偏上的 40Pin 双排排针，丝印标注 "P1"，兼容树莓派 40Pin 引脚定义。

---

## 架构

```
[DMA ISR — 后台自主循环, CPU 0% 参与]
  TX(写0x3B) → TC中断 → RX(读14字节) → TC中断
  → parse → soft dedup → push ring buffer(1024) → 下一TX
  速度: 1kHz (MPU6050 ODR, DMA 总线连续 = 无 idle-gap glitch)

[主循环 — 纯消费者]
  ring_pop → 陀螺零偏减去(启动校准5000帧) → 加速度6面校准
  → 自适应DLPF(硬件带宽 20/42/256Hz) → Mahony AHRS(Kp=2.0, Ki=0.1)
  → 降采样(÷20) → printf 13列CSV
```

**DMA ISR 驱动**（不是 polling）：`mpu6050_start_acquisition()` 启动第一次 TX 后，DMA 完成中断自动接管 TX→RX→push→TX 链，CPU 只从 ring buffer 取数据。

**AHRS 选型**：Mahony 全面领先 Madgwick（8/5 实验对比）——漂移 1.8 vs 19.4 °/hr，CPU 37 vs 43 μs，且独有陀螺偏置 I 项在线估计。EKF7 和 Madgwick 代码仍保留用于对比测试（MODE 5）。

---

## 编译与烧录

### 步骤 1：打开 SDK 环境

双击 `D:\sdk_env_v1.12.1\start_cmd.cmd`，打开 HPM SDK 命令行环境。

### 步骤 2：使用 start_gui 生成工程

在 SDK 命令行中：

```cmd
start_gui.exe
```

配置参数：
- **Board Path**: `D:\sdk_env_v1.12.1\imu_project\user_board`
- **Application Path**: `D:\sdk_env_v1.12.1\imu_project\user_app`  
- **Build Type**: `flash_xip`
- 点击 **Generate** 生成工程

> 项目会生成到 `D:\sdk_env_v1.12.1\hpm_prj\` 目录下。

### 步骤 3：编译

在 SDK 命令行中进入生成的项目目录：

```cmd
cd D:\sdk_env_v1.12.1\hpm_prj\imu_app_hpm5300evk_flash_xip_debug
ninja
```

### 步骤 4：烧录

用 USB 线连接 HPM5300EVK 的 DEBUG USB-C 口到电脑，然后在 SDK 命令行：

```cmd
cd D:\sdk_env_v1.12.1\hpm_sdk\boards\openocd
..\..\..\tools\openocd\openocd.exe -c "set HPM_SDK_BASE D:/sdk_env_v1.12.1/hpm_sdk; set BOARD hpm5300evk; set PROBE ft2232;" -f hpm5300_all_in_one.cfg -c "program D:/sdk_env_v1.12.1/hpm_prj/imu_app_hpm5300evk_flash_xip_debug/demo.elf verify reset exit"
```

---

## 运行与验证

### 串口验证

1. 烧录完成后，开发板自动复位运行
2. 用串口助手（波特率 **115200**）连接开发板 UART0 对应的 COM 口
3. 应该看到：

```
# ======================================
# IMU — EKF7 + Adaptive LPF + Online Temp Calib
# ======================================
# [I2C] OK  [IMU] INIT OK  (DLPF=42Hz)
# Gyro calib: 1500 samples, keep STILL...
# T0 = xx.xx C
# Gyro off@T0 (raw): xxx, xxx, xxx
# Ax,Ay,Az,Gx,Gy,Gz,Roll,Pitch,Yaw,q0,q1,q2,q3
0.001,-0.058,1.019,0.24,-0.11,0.28,0.12,-3.34,45.2,0.9995,0.0010,-0.0291,0.0062
...
```

> 如果看到 `[ERR] No device at 0x68! Check wiring.` → 检查接线，确认 SDA/SCL 没有接反。

### 输出格式（13 列）

```
Ax,Ay,Az,Gx,Gy,Gz,Roll,Pitch,Yaw,q0,q1,q2,q3
```

| 列 | 含义 | 单位 |
|----|------|------|
| Ax, Ay, Az | 加速度（校准后） | g |
| Gx, Gy, Gz | 陀螺（LPF 后） | °/s |
| Roll, Pitch, Yaw | 欧拉角 | ° |
| q0, q1, q2, q3 | 四元数 (w, x, y, z) | — |

### Python 可视化

安装 Python 依赖：

```bash
pip install pyserial matplotlib numpy pygame pyopengl
```

#### 3D 姿态查看器 (`imu_3d_viewer.py`)

```bash
python tools\imu_3d_viewer.py COM3
# 或回放 CSV 日志：
python tools\imu_3d_viewer.py data.csv
```

四元数驱动的 3D 方盒渲染（无万向节死锁），显示姿态、传感器读数、四元数 HUD。

控制：`R` 重置视角  `F` 全屏  `G` 网格开关  `A` 自动旋转  `ESC` 退出

#### Web 版工具（零依赖，浏览器打开即用）

```cmd
cd tools
python -m http.server 8000
```
然后浏览器访问 `http://localhost:8000`（**必须 localhost**，file:// 无法访问串口）：
- **`imu_3d_viewer.html`** — 3D 姿态查看器，拖入 CSV 回放，或点"连接串口"实时
- **`imu_algo_compare.html`** — 算法对比调试器，MODE 6 原始数据输入，浏览器内现算 Mahony vs Madgwick + 调 Kp/Ki/β

---

## 项目结构

```
imu_project/
  user_board/              ← 板级配置 (HPM5300EVK)
    board.c,h              ← 板级驱动
    clock.c,h              ← 时钟配置 + I2C/SPI/UART 时钟
    pinmux.c,h             ← 引脚复用 (含 I2C0 init)
    user_board.yaml        ← SOC/Flash 配置
    CMakeLists.txt
  user_app/
    CMakeLists.txt         ← 构建配置 (CONFIG_HPM_I2C=1)
    src/
      main.c               ← 主循环: DMA ISR消费 + Mahony + 自适应DLPF + 8种MODE
      main_diag.c          ← 最小诊断固件 (UART + LED)
      mpu6050.c/h          ← MPU6050 驱动: DMA ISR采集 + 校准 + 温漂在线标定
      mahony.c/h           ← Mahony AHRS: PI互补滤波 + 自适应Kp (主路径)
      madgwick.c/h         ← Madgwick AHRS: 梯度下降 (对比用, MODE 5)
      mahony_dsp.c         ← DSP加速版Mahony (对比用, MODE 3)
      ekf7.c/h             ← 7态EKF (遗留, 不再调用)
      perf_counter.h       ← cycle 计时代码
    linkers/gcc/
      user_linker.ld       ← HPM5361 flash_xip 链接脚本
  tools/                   ← 详见"Python 工具"小节
  results/                 ← 实验结果 CSV + PNG
  backups/                 ← 历史版本快照
  README.md                ← 本文件
  DEMO_RUNBOOK.md          ← 演示流程脚本
```

---

## 关键参数

| 参数 | 值 | 说明 |
|------|-----|------|
| I2C 速度 | 400 kHz | |
| MPU6050 ODR | 1 kHz | 经 soft dedup 后有效 1kHz |
| DLPF（初始） | 98 Hz | 上电默认，通用甜点 |
| 自适应 DLPF | 20/42/256 Hz | 陀螺幅度驱动：<5°/s→20Hz，5~100°/s→42Hz，>100°/s→256Hz |
| 自适应 DLPF 冷却 | 1.0 s | 防止 DLPF 频繁跳变 |
| Mahony Kp | 2.0 | P 增益（固定时 Kp=1.0 在此硬件上会漂移发散） |
| Mahony Ki | 0.1 | I 增益，陀螺零偏在线估计 |
| 自适应 Kp | ADAPTIVE_KP=1 | EMA 平滑 τ=150ms，检测 \|a\| 偏离 1g 时降 Kp |
| 陀螺校准 | 5000 帧, ISR 驱动 | 上电静止采集 |
| 野值过滤 | accel±30000, gyro±30000 (raw) | 整帧丢弃 |
| Ring buffer | 1024 帧 | mpu6050.h: MPU6050_RING_DEPTH |
| 降采样 | ÷20 (~50Hz output) | DOWNSAMPLE_N=20 |
| 串口 | 115200 | printf_nb 非阻塞 + UART TX FIFO 拥塞检测 |

### 加速度校准参数（main.c 硬编码）

```c
const float OX=331.1f, OY=-651.1f, OZ=-50.6f;
const float SX=0.999533f, SY=0.997731f, SZ=0.985919f;
// 校准公式: a_cal = S * (raw - O) / accel_sf
// 用 calib_accel.py 重新跑 6 面标定可更新这组值
```

### 测试模式 (`main.c`: `#define TEST_MODE`)

| MODE | 名称 | 用途 |
|:--:|------|------|
| 0 | NORMAL | 常规 13 列输出 + 每 200 帧 #CYC 统计 |
| 1 | PERF | CSV: mahony+frame cycles/200 帧窗口 |
| 2 | DRIFT | 静态漂移测试: 1Hz 输出 t,Roll,Pitch,Yaw,ax-gz |
| 3 | DSP_CMP | 逐帧交替 float Mahony vs DSP Mahony |
| 4 | SAT | 饱和点扫描: 软件降采样 ÷1/2/4/8/10 |
| 5 | ALGO_CMP | Mahony vs Madgwick 逐帧对比 |
| 6 | RAW_DATA | 原始校准传感器数据（无 AHRS，离线分析用） |
| 7 | DLPF_SWP | DLPF 自动轮询: 7 档 × 30s |

---

## Python 工具一览

| 工具 | 输入 | 输出 | 说明 |
|------|------|------|------|
| `record_serial.py` | COM 口 | CSV | 串口数据录制 |
| `imu_3d_viewer.py` | COM/CSV | PyGame 窗口 | 3D 姿态实时/回放 |
| `calib_accel.py` | 6面CSV | O/S 参数 | 加速度计最小二乘标定 |
| `calib_gyro_temp.py` | 温度+陀螺CSV | K_T 系数 | 陀螺温漂线性拟合 |
| `allan_variance.py` | 长时静态CSV | Allan偏差图 + ARW/BI | 噪声物理极限 |
| `psd_analysis.py` | 长时静态CSV | PSD图 + 噪声地板 | Welch 频域分析 |
| `param_sweep.py` | 静态+动态CSV | 热力图 + 最优参数 | Mahony Kp/Ki 扫描 |
| `dlpf_compare.py` | MODE 7 CSV | 噪声-带宽对比图 | DLPF 7档实测 |
| `stress_analysis.py` | shake/highrate/drift CSV | RMS/P-P/漂移率 | 鲁棒边界测试 |
| `algo_compare.py` | CSV | Mahony vs Madgwick | 算法性能对比 |
| `perf_test.py` | MODE 1 CSV | μs/frame | CPU cycle 基准 |

---

## 故障排查

| 现象 | 可能原因 | 解决方法 |
|------|---------|---------|
| 串口无输出 | 烧录失败 / 串口号不对 | 检查 OpenOCD 烧录日志，确认 COM 口 |
| `No device at 0x68` | 接线错误 / IMU 未供电 | 检查 VCC/GND, SDA/SCL 是否接反 |
| 串口有输出但全是 0 | MPU6050 未初始化 | 检查 WHO_AM_I 是否为 0x68 |
| Python 找不到串口 | COM 口占用 / 权限 | 先关掉串口助手再运行 Python |
| `pyserial` 未安装 | 缺少 Python 包 | `pip install pyserial` |
| I2C 超时 (!TIMEOUT) | GY-521 10kΩ 上拉太弱 | SDA/SCL 各并 2.2kΩ 到 VCC |

---

## I2C 备选方案

如果不想用 I2C，HPM5300EVK P1 接口也提供 **SPI1**：

| 功能 | P1 Pin | GPIO |
|------|--------|------|
| MOSI | Pin 19 | PA29 |
| MISO | Pin 21 | PA28 |
| SCLK | Pin 23 | PA27 |
| CS0  | Pin 24 | PA26 |

适合 SPI 接口的 IMU（如 ICM-20948、BMI160），修改 `main.c` 中的初始化逻辑即可。
