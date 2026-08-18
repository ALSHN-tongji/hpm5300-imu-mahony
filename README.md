## 基于先楫 HPM5300EVK + MPU6050 (GY-521) 的六轴 IMU 姿态解算系统
---

## 项目简介

本项目基于 RISC-V 内核 HPM5300 MCU（480 MHz），完成一套完整 6 轴 IMU 姿态解算固件。采用 **I2C + DMA 自循环采集 + 环形缓冲区** 实现 CPU 无参与传感器数据搬运；完成加速度六面标定、陀螺零偏与温度漂移在线补偿；通过 Allan 方差量化传感器噪声；实现 Mahony / Madgwick / ES-EKF 多算法对照；设计自适应 DLPF、自适应 Kp 双层优化策略；配套零依赖 Web 3D 可视化调试平台，实现实时观测、离线 CSV 回放、网页端调参，完成完整嵌入式传感器融合实验闭环。

硬件 I2C 总线速率 400 kHz，DMA 采样可达 2.5 kHz，固件 UART 输出 115200 波特，50 Hz 姿态有效输出。本项目 **仅使用 6 轴（加速度计 + 陀螺仪，无磁力计）**。

## ✨ 核心亮点

1. **I2C + DMA 自循环采集架构**
   DMA 完成 I2C 读写搬运，中断上下文自动循环采集，CPU 不参与数据搬运；搭配 1024 帧深度 SPSC 环形缓冲区，生产者（DMA 中断）— 消费者（主循环解算）解耦，临界区保护保证数据一致性。

2. **传感器完整标定链路**
   - 加速度计六面最小二乘标定，每面采集 300 帧取平均，利用 ±1g 重力基准求解灵敏度与三轴偏置；
   - 上电陀螺静止零偏自动标定（5000 帧），运动状态自动识别；
   - 陀螺温度漂移递归线性回归在线补偿，遗忘因子 `λ = 0.995`，温度散布 ≥ 3°C 触发更新。

3. **自适应双层滤波策略**
   - **自适应 DLPF 三档切换**：依据陀螺角速度自动切换 20 Hz / 42 Hz / 256 Hz，兼顾静态低噪声与大机动低延迟，1 s 冷却时间 + 0.5 s 低通平滑，防止档位振荡；
   - **Mahony 自适应 Kp 增益**：通过加速度模值判断外部加速度干扰，EMA 平滑（`τ = 150 ms`）动态降低 Kp，晃动时降低加速度计校正权重，提升动态姿态表现。

4. **算法定量评估**
   对 Mahony 完成 56 组 Kp/Ki 参数全因子扫描，得出稳定约束条件 `Kp/Ki ≥ 20`；对比 Mahony / Madgwick / ES-EKF；Allan 方差分析（2 h 静态数据）得到 ARW = 0.41 °/√hr、BI = 6.8 °/hr，为 EKF 噪声矩阵提供实测依据。

5. **固件内置多组测试模式**
   固件通过宏切换 8 种测试模式（MODE 0-7），支持性能统计、长时漂移、DLPF 自动遍历、算法对照、原始数据导出，无需修改业务代码即可开展多维度实验。

6. **Web 端一体化调试工具（零依赖单 HTML）**
   分为 **3D 姿态查看器** 与 **双算法对比调试器** 两大模块，WebSerial 实时接收串口数据；支持录制导出 CSV；支持离线回放长时间漂移记录；JS 移植 AHRS 算法，网页直接修改滤波参数重算姿态，无需重新编译烧录固件。

---

## 📦 硬件准备

### 购买清单

| 物品 | 型号/关键词 | 参考价格 |
|------|-----------|---------|
| IMU 模块 | **GY-521 (MPU6050)** | 3 ~ 5 元 |
| 杜邦线 母对母 | 10 cm / 20 cm，4 ~ 5 根 | 1 ~ 2 元 |

### 系统硬件配置

| 组件 | 型号/参数 | 说明 |
|------|----------|------|
| 微控制器 | HPM5300 (RISC-V) | 480 MHz, I2C×4, DMA×8 |
| IMU 传感器 | MPU6050 (GY-521) | 3 轴陀螺 + 3 轴加计 + 温度 |
| 陀螺量程 | ±250 °/s | 灵敏度 131 LSB/(°/s) |
| 加计量程 | ±2 g | 灵敏度 16384 LSB/g |
| 通信接口 | I2C @400 kHz | DMA 自循环 ~2.5 kHz |
| DLPF | 42 Hz (默认) | 自适应 20/42/256 Hz |
| 输出 | UART 115200 | 13 列 CSV, 50 Hz 有效输出 |

### 接线定义

```
GY-521 模块          HPM5300EVK P1 接口 (40Pin 树莓派兼容)

  VCC  ──────────  Pin 1  (3.3V)      ← 红色线
  GND  ──────────  Pin 6  (GND)       ← 黑色线
  SCL  ──────────  Pin 28 (I2C0_SCL)  ← PB02，黄色线
  SDA  ──────────  Pin 27 (I2C0_SDA)  ← PB03，绿色线
  INT  ──────────  Pin 13 (PA31)      ← 蓝色线（数据就绪中断）
```

> **P1 接口位置**：开发板中间偏上的 40 Pin 双排排针，丝印 `P1`，兼容树莓派引脚定义。
>
> ⚠️ 硬件提示：GY-521 自带 10 kΩ 上拉，400 kHz 高速 I2C 建议 SDA/SCL 各并联 **2.2 kΩ 上拉到 3.3 V**，避免总线锁死（详见[故障排查](#-故障排查)）。

### 备选 SPI 接口

适配 ICM-20948 等 SPI 接口的 IMU：

| 功能 | P1 Pin | GPIO |
|------|--------|------|
| MOSI | Pin 19 | PA29 |
| MISO | Pin 21 | PA28 |
| SCLK | Pin 23 | PA27 |
| CS0  | Pin 24 | PA26 |

---

## ⚙️ 编译与烧录

### 步骤 1：打开 SDK 环境

双击 `D:\sdk_env_v1.12.1\start_cmd.cmd`，打开 HPM SDK 命令行环境。

### 步骤 2：使用 start_gui 生成工程

```cmd
start_gui.exe
```

配置参数：

- **Board Path**：`D:\sdk_env_v1.12.1\hpm_sdk\imu_project\user_board`
- **Application Path**：`D:\sdk_env_v1.12.1\hpm_sdk\imu_project\user_app`
- **Build Type**：`flash_xip`
- 点击 **Generate** 生成工程

> 项目会生成到 `D:\sdk_env_v1.12.1\hpm_prj\` 目录下。

### 步骤 3：编译

```cmd
cd D:\sdk_env_v1.12.1\hpm_prj\imu_app_hpm5300evk_flash_xip_debug
ninja
```

### 步骤 4：烧录

用 USB 线连接 HPM5300EVK 的 DEBUG USB-C 口到电脑：

```cmd
cd D:\sdk_env_v1.12.1\hpm_sdk\boards\openocd
..\..\..\tools\openocd\openocd.exe -c "set HPM_SDK_BASE D:/sdk_env_v1.12.1/hpm_sdk; set BOARD hpm5300evk; set PROBE ft2232;" -f hpm5300_all_in_one.cfg -c "program D:/sdk_env_v1.12.1/hpm_prj/imu_app_hpm5300evk_flash_xip_debug/demo.elf verify reset exit"
```

---

## 🚀 运行与验证

### 串口验证

1. 烧录完成后，开发板自动复位运行
2. 用串口助手（波特率 **115200**）连接开发板 UART0 对应的 COM 口
3. 应看到如下输出：

```
# MPU6050 DMA self-loop + Mahony
# [ID] WHO_AM_I=0x70 (UNKNOWN)
# [I2C] OK [IMU] OK
# Ax,Ay,Az,Gx,Gy,Gz,Roll,Pitch,Yaw,q0,q1,q2,q3
# Gyro calib: 5000 samples, keep STILL...
#  calib: probing...
#  calib: OK, collecting...
...
```

> 如果看到 `[ERR] No device at 0x68! Check wiring.` → 检查接线，确认 SDA/SCL 没有接反。

### 输出格式（13 列 CSV）

| 列 | 含义 | 单位 |
|----|------|------|
| Ax, Ay, Az | 加速度（校准后） | g |
| Gx, Gy, Gz | 陀螺（LPF 后） | °/s |
| Roll, Pitch, Yaw | 欧拉角 | ° |
| q0, q1, q2, q3 | 四元数 (w, x, y, z) | — |

---

## 🏗️ 系统架构

软件基于 HPM SDK，使用 C 语言开发，`start_gui` 生成工程，Ninja + GCC RISC-V 编译，OpenOCD 烧录，SEGGER Embedded Studio 调试。系统采用三层架构：底层 DMA ISR 数据采集、中层 Mahony 姿态解算、上层测试模式与数据输出。

```
[DMA ISR — 后台自主循环，CPU 0% 参与]
  TX(写 0x3B) → TC 中断 → RX(读 14 字节) → TC 中断
  → parse → soft dedup → push ring buffer(1024) → 下一 TX
  速度：~2.5 kHz (MPU6050 ODR，DMA 总线连续 = 无 idle-gap glitch)
  核心 ISR 代码不到 40 行

[Ring Buffer — SPSC 生产者-消费者模型]
  ISR 将解析后的数据帧（3 轴加速度 + 3 轴陀螺 + 温度 + 时间戳）
  推入 1024 帧深度环形缓冲区，主循环通过 mpu6050_ring_pop() 取数解算。
  pop 操作使用临界区保护（关 DMA 中断 → 读 head/tail → 开中断），
  缓冲区满时直接丢弃并记录 drop_count，不阻塞 ISR。

[主循环 — 纯消费者]
  ring_pop → 陀螺零偏减去(启动校准 5000 帧) → 加速度 6 面校准
  → 自适应 DLPF(硬件带宽 20/42/256 Hz) → Mahony AHRS(Kp=2.0, Ki=0.1)
  → 降采样(÷20) → printf 13 列 CSV
```

**DMA ISR 驱动**（非 polling）：`mpu6050_start_acquisition()` 启动首次 TX 后，DMA 完成中断自动接管 TX → RX → push → TX 链，CPU 只从 ring buffer 取数据。

### Mahony 互补滤波算法

Mahony 算法的 P 项（增益 Kp）通过加速度计测量的重力方向与四元数估计方向之间的叉积误差，对陀螺角速度进行瞬时校正；I 项（增益 Ki）对叉积误差积分，自动估计并补偿陀螺的时变零偏。每帧计算耗时约 37.4 μs（17,934 cycles @480 MHz），CPU 占用率仅 3.7%。

```c
/* Mahony AHRS — PI complementary filter on quaternion */
float ex = ay*vz - az*vy;                  // cross product error
float ey = az*vx - ax*vz;
float ez = ax*vy - ay*vx;
ix += KI * ex * dt;  iy += KI * ey * dt;  iz += KI * ez * dt;  // I term
float wx = gx + Kp*ex + ix;                // corrected angular rate
float wy = gy + Kp*ey + iy;
float wz = gz + Kp*ez + iz;
q0 += 0.5f*(-q1*wx - q2*wy - q3*wz) * dt; // quaternion integration
q1 += 0.5f*( q0*wx + q2*wz - q3*wy) * dt;
q2 += 0.5f*( q0*wy - q1*wz + q3*wx) * dt;
q3 += 0.5f*( q0*wz + q1*wy - q2*wx) * dt;
// normalize quaternion (not shown)
```

**AHRS 选型**：Mahony 全面领先 Madgwick（漂移 1.8 vs 19.4 °/hr，CPU 37 vs 43 μs），且独有陀螺偏置 I 项在线估计。EKF7 和 Madgwick 代码保留用于对比测试（MODE 5）。

### 传感器标定

```c
float ax = SX * (raw_ax - OX) / accel_sf;  // 6-face calibrated accel
float ay = SY * (raw_ay - OY) / accel_sf;
float az = SZ * (raw_az - OZ) / accel_sf;
gx_comp = (raw_gx - gyro_off_T0)/gyro_sf - K_T[0]*(T - T0);  // temp comp
```

- **加速度计**：6 面最小二乘标定，利用 ±1g 重力基准求解各轴偏置和灵敏度
- **陀螺仪**：上电自动采集 5000 帧静止数据做零偏标定，同时检测数据散布判断运动干扰
- **温度漂移**：递归线性回归在线标定，遗忘因子 λ=0.995，温度散布 ≥ 3°C 触发更新

---

## 📊 实验结果

### 算法对比：Mahony vs Madgwick

在相同硬件条件下，Mahony 经 56 组 Kp/Ki 参数扫描确定最优值 Kp=2.0、Ki=0.1；Madgwick 经 8 组 β 扫描确定 β=0.05。

| 指标 | Mahony (Kp=2.0) | Madgwick (β=0.05) | 结论 |
|------|:---:|:---:|------|
| CPU 耗时 | **37.4 μs** | 43.3 μs | Mahony 快 16% |
| CPU @1kHz | **3.7%** | 4.3% | 均充裕 |
| 静态漂移 | **1.8 °/hr** | 19.4 °/hr | Mahony 优 10.8 倍 |
| 静态噪声 RMS | 0.68° | **0.024°** | Madgwick 优（梯度下降自带低通） |
| 动态平滑 RMS | **9.55°** | 21.4° | Mahony 优 55% |
| 陀螺零偏估计 | **有 (I 项)** | 无 | Mahony 独有 |

> **结论**：Mahony 在漂移、动态响应、CPU 效率上全面领先；其 I 项提供的陀螺零偏自动估计能力是 Madgwick 不具备的关键优势。Madgwick 的梯度下降法自带低通效应，静态噪声更低，但以牺牲动态响应为代价。综合选型：**Mahony**。

### 传感器噪声参数辨识（Allan 方差）

采集 2 小时静态数据（约 180 万帧，248 Hz 有效采样率）：

| 参数 | 陀螺仪 | 加速度计 | 单位 |
|------|:---:|:---:|------|
| 角度/速度随机游走 (ARW/VRW) | **0.41** | 0.092 | °/√hr / m/s/√hr |
| 零偏不稳定性 (BI) | **6.8** | 83 | °/hr / μg |
| BI 对应积分时间 τ | 142~668 | 8~15 | s |

> **核心发现**：Mahony 实测静态漂移 1.8 °/hr，低于陀螺开环 BI（6.8 °/hr）达 3.8 倍。闭环融合（Mahony I 项 + 加速度计重力参考）将系统漂移压到了传感器自身硬件极限以下。BI 表征的是开环零偏漂移下限，闭环融合可通过外部绝对参考突破此天花板。

### DLPF 7 档带宽-噪声-延迟实测

使用 MODE 7 自动遍历全部 7 档 DLPF（每档 30 s）：

| DLPF | 延迟 (ms) | Gx 噪声 (°/s) | vs 98Hz | 评价 |
|------|:---:|:---:|:---:|------|
| 256 Hz | 1.0 | 0.130 | +83% | 大机动专用 |
| 188 Hz | 1.9 | 0.115 | +62% | — |
| 98 Hz | 2.8 | 0.071 | 基准 | 延迟敏感场景 |
| **42 Hz ★** | **4.8** | **0.052** | **-27%** | **通用甜点** |
| 20 Hz | 8.3 | 0.036 | -49% | 静态低噪声 |
| 10 Hz | 13.4 | 0.026 | -63% | — |
| 5 Hz | 18.6 | 0.022 | -69% | 最低噪声 |

> **关键发现**：MPU6050 七档 DLPF 的陀螺噪声随带宽严格单调下降（256 Hz → 5 Hz，降幅 83%），符合白噪声 ∝ √BW 定律。42 Hz 以 27% 噪声优势、仅 2 ms 延迟代价胜出，确定为通用甜点。

### 性能基准

| 指标 | 数值 | 说明 |
|------|------|------|
| 纯算法耗时 | 37.4 μs (17,934 cycles) | MODE 1 PERF 模式测量 |
| 完整帧处理 | ~40 μs | 含坐标变换 + 统计 |
| CPU @1kHz | 3.7% | 远低于饱和点 |
| 静态漂移 | 1.8 °/hr | MODE 2 长时测试 |
| 动态 RMS (晃动) | 9.55° | Mahony Kp=2.0 |

### 参数优化（Kp/Ki 扫描）

56 组全因子扫描的核心发现：

- **稳定性条件**：`Kp/Ki ≥ 20` 才能维持长期稳定
- **Kp=1.0 隐患**：短时测试漂移仅 +0.37 °/hr，但长时间运行后 I 项积累失控，漂移恶化至 -3478 °/hr
- **Kp=2.0、Ki=0.1**（比值 = 20）经 2 小时长时验证漂移稳定在 1.8 °/hr
- **启示**：短时测试不足以验证算法的长期稳定性，I 项的累积效应需要长时间窗口才能暴露

---

## 📁 项目结构

```
imu_project/
├── user_board/                  ← 板级配置 (HPM5300EVK)
│   ├── board.c / board.h        ← 板级驱动
│   ├── clock.c / clock.h        ← 时钟配置 + I2C/SPI/UART 时钟
│   ├── pinmux.c / pinmux.h      ← 引脚复用 (含 I2C0 init)
│   ├── user_board.yaml          ← SOC/Flash 配置
│   └── CMakeLists.txt
├── user_app/
│   ├── CMakeLists.txt           ← 构建配置 (CONFIG_HPM_I2C=1)
│   ├── src/
│   │   ├── main.c               ← 主循环：DMA ISR 消费 + Mahony + 自适应 DLPF + 8 种 MODE
│   │   ├── main_diag.c          ← 最小诊断固件 (UART + LED)
│   │   ├── mpu6050.c / .h       ← MPU6050 驱动：DMA ISR 采集 + 校准 + 温漂在线标定
│   │   ├── mahony.c / .h        ← Mahony AHRS：PI 互补滤波 + 自适应 Kp (主路径)
│   │   ├── madgwick.c / .h      ← Madgwick AHRS：梯度下降 (对比用，MODE 5)
│   │   ├── mahony_dsp.c         ← DSP 加速版 Mahony (对比用，MODE 3)
│   │   ├── ekf7.c / .h          ← 7 态 EKF (遗留，不再调用)
│   │   └── perf_counter.h       ← cycle 计时代码
│   └── linkers/gcc/
│       └── user_linker.ld       ← HPM5361 flash_xip 链接脚本
├── tools/                       ← 详见 Python 工具小节
├── results/                     ← 实验结果 CSV + PNG
├── web_viewer/                  ←  Web可视化工具
└── README.md                    ← 本文件
```

---

## 📊 关键参数

| 参数 | 值 | 说明 |
|------|-----|------|
| I2C 速度 | 400 kHz | |
| MPU6050 ODR | 1 kHz | 经 soft dedup 后有效 1 kHz |
| DLPF（初始） | 42 Hz | 上电默认，通用甜点 |
| 自适应 DLPF | 20 / 42 / 256 Hz | 陀螺幅度驱动：`<5°/s` → 20 Hz，`5~100°/s` → 42 Hz，`>100°/s` → 256 Hz |
| 自适应 DLPF 冷却 | 1.0 s + 0.5 s LP 平滑 | 防止 DLPF 频繁跳变 |
| Mahony Kp | 2.0 | P 增益（固定 Kp=1.0 在此硬件上会漂移发散） |
| Mahony Ki | 0.1 | I 增益，陀螺零偏在线估计 |
| 稳定条件 | Kp/Ki ≥ 20 | 56 组扫描得出 |
| 自适应 Kp | `ADAPTIVE_KP=1` | EMA 平滑 `τ=150ms`，检测 `\|a\|` 偏离 1g 时降 Kp |
| 陀螺校准 | 5000 帧，ISR 驱动 | 上电静止采集 |
| 野值过滤 | `accel ±30000`，`gyro ±30000` (raw) | 整帧丢弃 |
| Ring buffer | 1024 帧 | `mpu6050.h: MPU6050_RING_DEPTH` |
| 降采样 | `÷20`（~50 Hz output） | `DOWNSAMPLE_N=20` |
| 串口 | 115200 | `printf_nb` 非阻塞 + UART TX FIFO 拥塞检测 |

### 加速度校准参数（`main.c` 硬编码）

```c
const float OX = 331.1f, OY = -651.1f, OZ = -50.6f;
const float SX = 0.999533f, SY = 0.997731f, SZ = 0.985919f;
// 校准公式: a_cal = S * (raw - O) / accel_sf
// Z 轴灵敏度偏差 ~1.7%，属 MEMS 制造公差范围
// 用 calib_accel.py 重新跑 6 面标定可更新这组值
```

### 测试模式（`main.c`：`#define TEST_MODE`）

| MODE | 名称 | 用途 |
|:----:|------|------|
| 0 | NORMAL | 常规 13 列输出 + 每 200 帧 `#CYC` 统计 |
| 1 | PERF | CSV：mahony + frame cycles / 200 帧窗口 |
| 2 | DRIFT | 静态漂移测试：1 Hz 输出 t, Roll, Pitch, Yaw, ax-gz |
| 3 | DSP_CMP | 逐帧交替 float Mahony vs DSP Mahony |
| 4 | SAT | 饱和点扫描：软件降采样 ÷1 / 2 / 4 / 8 / 10 |
| 5 | ALGO_CMP | Mahony vs Madgwick 逐帧对比 |
| 6 | RAW_DATA | 原始校准传感器数据（无 AHRS，离线分析用） |
| 7 | DLPF_SWP | DLPF 自动轮询：7 档 × 30 s |

---

## 🐍 Python 工具一览

安装依赖：

```bash
pip install pyserial matplotlib numpy pygame pyopengl
```

| 工具 | 输入 | 输出 | 说明 |
|------|------|------|------|
| `record_serial.py` | COM 口 | CSV | 串口数据录制 |
| `imu_3d_viewer.py` | COM / CSV | PyGame 窗口 | 3D 姿态实时 / 回放 |
| `calib_accel.py` | 6 面 CSV | O / S 参数 | 加速度计最小二乘标定 |
| `calib_gyro_temp.py` | 温度 + 陀螺 CSV | `K_T` 系数 | 陀螺温漂线性拟合 |
| `allan_variance.py` | 长时静态 CSV | Allan 偏差图 + ARW / BI | 噪声物理极限 |
| `psd_analysis.py` | 长时静态 CSV | PSD 图 + 噪声地板 | Welch 频域分析 |
| `param_sweep.py` | 静态 + 动态 CSV | 热力图 + 最优参数 | Mahony Kp / Ki 扫描 |
| `dlpf_compare.py` | MODE 7 CSV | 噪声-带宽对比图 | DLPF 7 档实测 |
| `stress_analysis.py` | shake / highrate / drift CSV | RMS / P-P / 漂移率 | 鲁棒边界测试 |
| `algo_compare.py` | CSV | Mahony vs Madgwick | 算法性能对比 |
| `perf_test.py` | MODE 1 CSV | μs / frame | CPU cycle 基准 |

### 3D 姿态查看器

```bash
python tools\imu_3d_viewer.py COM3
# 或回放 CSV 日志：
python tools\imu_3d_viewer.py data.csv
```

四元数驱动的 3D 方盒渲染（无万向节死锁），显示姿态、传感器读数、四元数 HUD。

控制：`R` 重置视角 · `F` 全屏 · `G` 网格开关 · `A` 自动旋转 · `ESC` 退出

### Web 版工具（零依赖，浏览器打开即用）

```cmd
cd tools
python -m http.server 8000
```

浏览器访问 `http://localhost:8000`（**必须 localhost**，`file://` 无法访问串口）：

- **`imu_3d_viewer.html`** — 3D 姿态查看器
  - **在线模式**：WebSerial 实时接收 115200 串口数据，渲染无万向锁三维模型，同步展示设备实时姿态；支持一键录制导出 CSV
  - **离线模式**：导入长时间静态采集 CSV，提供进度拖拽、倍速回放、循环播放，完整复现数小时尺度下姿态缓慢漂移曲线
- **`imu_algo_compare.html`** — 双算法对比调试器
  - 内置 JS 移植版 Mahony / Madgwick，读取同一段 MODE 6 原始数据并行运算
  - 页面左右分栏渲染双三维模型 + 实时 Roll/Pitch/Yaw 漂移对比曲线
  - 实时输出稳态噪声 RMS、每小时漂移速率量化指标
  - 支持网页端实时修改 Kp/Ki/β 参数，300 ms 自动重算，无需重新编译烧录固件
---

## 🔧 故障排查

| 现象 | 可能原因 | 解决方法 |
|------|---------|---------|
| 串口无输出 | 烧录失败 / 串口号不对 | 检查 OpenOCD 烧录日志，确认 COM 口 |
| `No device at 0x68` | 接线错误 / IMU 未供电 | 检查 VCC / GND，SDA / SCL 是否接反 |
| 串口有输出但全是 0 | MPU6050 未初始化 | 检查 `WHO_AM_I` 是否为 `0x68` |
| Python 找不到串口 | COM 口占用 / 权限 | 先关掉串口助手再运行 Python |
| `pyserial` 未安装 | 缺少 Python 包 | `pip install pyserial` |
| I2C 超时 `!TIMEOUT` | GY-521 10 kΩ 上拉太弱 | SDA / SCL 各并联 2.2 kΩ 到 VCC |

### 典型调试案例

以下是开发过程中遇到的 3 个典型嵌入式工程问题及完整排查过程：

#### 问题一：I2C 总线间歇性超时锁死

**现象**：系统运行数秒至数分钟后串口输出 `!TIMEOUT`，I2C 总线停止响应，需手动复位。

**根因**：用逻辑分析仪捕获波形后发现 GY-521 模块的 I2C 上拉电阻为 10 kΩ，对 400 kHz 快速模式和 3.3 V 电平而言上拉强度不足，信号上升沿过缓（>300 ns），SDA 线在空闲期间被从机意外拉低，主机无法发起新的 START 条件。

**解决**：
1. SDA / SCL 各并联 2.2 kΩ 电阻至 VCC，将上升沿压缩至 <100 ns
2. 固件加入 **I2C Bus Recovery** 机制——检测到 500 ms 以上无数据后，自动执行 9 周期 SCL 时钟脉冲 + GPIO 复位 SDA + I2C 控制器复位的完整恢复序列
3. 更换上拉后 TIMEOUT 故障彻底消失，Bus Recovery 仅作为最后防线

#### 问题二：FT2232 调试器无法连接

**现象**：Segger Embedded Studio（SES）报 "DTM version -1" 错误，无法通过 JTAG/SWD 连接芯片。

**根因**：Windows Update 在夜间自动更新时将 FT2232 的专用 WinUSB 驱动替换为通用 "USB Serial Converter" 驱动，导致调试器不被 SES 识别。

**解决**：
1. 使用 **Zadig** 工具将 FT2232 接口 0 驱动重新替换为 WinUSB
2. 按住 HPM5300EVK 上的 BOOT 键再上电可强制进入 ISP 模式绕过连接问题
3. **预防措施**：在 Windows 设备安装设置中关闭 "自动下载制造商应用" 选项

#### 问题三：printf 阻塞导致陀螺数据累积偏移

**现象**：静止状态下 Roll/Pitch 缓慢漂移，每 10 秒约偏 1-2°，漂移速率不恒定。

**根因**：`printf()` 在 UART TX FIFO 满时会阻塞等待，115200 波特率下每行 13 列数据约 150 字符，连续输出时 FIFO 频繁填满，导致主循环被阻塞。阻塞期间 DMA ISR 持续向 ring buffer 推数据，恢复后一口气消费大量积压帧，dt 计算失准，导致 Mahony 积分误差累积。

**解决**：
1. 实现非阻塞打印函数 `print_nb()` — 发送前检查 TX FIFO 空闲空间，满则直接丢弃
2. 输出降采样改为 1:20（50 Hz 有效输出率），稳态下 FIFO 不饱和
3. 修复后主循环周期稳定在约 1 ms，姿态漂移恢复正常

---

## 🎓 实习总结

通过本次实习，完成了一套基于 HPM5300 + MPU6050 的完整 IMU 姿态解算系统：

- **系统设计**：DMA ISR 自循环 I2C 采集架构实现了数据采集与姿态解算的完全硬件解耦——DMA 在中断上下文中以 2.5 kHz 速率自动循环，CPU 专注于姿态解算，占用率仅 3.7%。1024 帧环形缓冲区作为 SPSC 桥梁，通过临界区保护确保数据一致性。
- **算法工程化**：Mahony 算法经 56 组参数扫描确定了 Kp=2.0、Ki=0.1 的最优配置和 Kp/Ki ≥ 20 的稳定条件；自适应 DLPF（三档切换）+ 自适应 Kp（EMA 平滑）的软硬双层联合优化，在保持静态低噪声的同时兼顾动态快速响应，晃动场景下姿态误差降低 33%-63%。
- **工程调试**：解决了 I2C 上拉不足导致总线锁死、Windows 自动更新冲掉 FT2232 驱动、printf 阻塞引发积分漂移等 4 个核心问题，每个都经历了"现象观察 → 工具测量 → 根因定位 → 方案验证"的完整排查闭环。
- **芯片平台**：深入学习了 HPM5300 的 I2C DMA 握手模式、DMAMUX 通道配置、MCHTMR 高精度定时器、IOMUX 引脚复用、UART FIFO 状态检测等外设开发，积累了 RISC-V 架构 MCU 的实战经验。

### 未来扩展方向

- 扩展磁力计融合（9-DOF）以解决 Yaw 长期漂移
- 移植更高速 SPI 接口传感器（如 ICM-20948）
- 将标定参数存储至片内 Flash 实现免重复标定
