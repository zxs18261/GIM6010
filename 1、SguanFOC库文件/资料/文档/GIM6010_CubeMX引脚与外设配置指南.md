# GIM6010 CubeMX 引脚与外设配置指南

> 说明 GIM6010 工程每个引脚在 CubeMX 中应配置的模式、参数及原因。  
> 工程文件：`GIM6010_V0.0.ioc` | MCU：STM32G431CBT6 (LQFP-48)  
> 文档路径：`E:\github\GIM6010\SguanFOC库文件\资料\文档`

---

## 目录

1. [配置总览](#1-配置总览)
2. [引脚模式总表](#2-引脚模式总表)
3. [RCC 时钟](#3-rcc-时钟)
4. [TIM1 — 三相 PWM + ADC 触发（重点）](#4-tim1--三相-pwm--adc-触发重点)
5. [TIM4 — 低频环 / RGB（重点）](#5-tim4--低频环--rgb重点)
6. [ADC1 / ADC2 — 电流采样（重点）](#6-adc1--adc2--电流采样重点)
7. [SPI1 — 外部 SPI 扩展](#7-spi1--外部-spi-扩展)
8. [SPI3 — MT6816 磁编码器（重点）](#8-spi3--mt6816-磁编码器重点)
9. [USART1 — 调试串口（重点）](#9-usart1--调试串口重点)
10. [FDCAN1 — CAN 通信](#10-fdcan1--can-通信)
11. [DMA 配置](#11-dma-配置)
12. [NVIC 中断优先级](#12-nvic-中断优先级)
13. [GPIO 独立引脚](#13-gpio-独立引脚)
14. [CubeMX 操作步骤速查](#14-cubemx-操作步骤速查)
15. [当前工程待修正项](#15-当前工程待修正项)
16. [修订记录](#16-修订记录)

---

## 1. 配置总览

### 1.1 必须启用的外设 IP

| IP | 用途 | FOC 优先级 |
|----|------|:----------:|
| **RCC** | HSE 8MHz → PLL 170MHz | 必须 |
| **TIM1** | 三相 PWM + ADC 触发 | **P0** |
| **ADC1** | Iv/Iw 注入 + VBUS/NTC | **P0** |
| **ADC2** | IBUS 常规采样 | P1 |
| **SPI3** | MT6816 编码器 | **P0** |
| **USART1** | 调试 / JustFloat | **P0** |
| **DMA** | USART TX/RX | P1 |
| **TIM4** | SguanFOC 低频环 1kHz | **P0** |
| **FDCAN1** | 电机 CAN 协议 | P2（后期） |
| **SPI1** | CN47 外部 SPI | P3（可选） |
| **SYS** | Debug SWD | 必须 |

### 1.2 配置逻辑关系

```
TIM1 (20kHz PWM)
  ├── PA8/9/10, PB13/14/15 → FD6288 → 三相 MOSFET
  ├── TIM1_CH4 (无引脚) → TRGO OC4REF → ADC1 注入触发
  └── TIM1 更新中断 → SguanFOC_High_Loop()

ADC1 注入 (PB1 Iv, PB0 Iw) ← TIM1_CC4 硬件触发

SPI3 + PA15 CSN → MT6816 角度

USART1 + DMA → 上位机调试

TIM4 更新中断 → SguanFOC_Low_Loop()（需改 Period）
```

---

## 2. 引脚模式总表

| 引脚 | GPIO | CubeMX 信号 / 模式 | 用户标签 | 连接硬件 | 为什么这样配 |
|:----:|:----:|:-------------------|:---------|:---------|:-------------|
| 5 | PF0 | **RCC_OSC_IN** / HSE-External-Oscillator | — | 8MHz 晶振 | 外部高精度时钟，PLL 基准 |
| 6 | PF1 | **RCC_OSC_OUT** / HSE-External-Oscillator | — | 8MHz 晶振 | HSE 输出 |
| 7 | NRST | 系统默认 | — | 复位按键 | 硬件复位，无需在 Pinout 改 |
| 12 | PA2 | **ADC1_IN3** / IN3-Single-Ended | ADC_NTC1 | NTC2 热敏电阻 | 板载温度慢速采样 |
| 13 | PA3 | **ADC1_IN4** / IN4-Single-Ended | ADC_NTC2 | P1 外接 NTC | 外部温度慢速采样 |
| 14 | PA4 | **GPIO_Output** | CSN | CN47 SPI 片选 | SPI1 软件片选，手动拉低 |
| 15 | PA5 | **SPI1_SCK** / Full-Duplex Master | SCK | CN47 | 外部 SPI 时钟 |
| 16 | PA6 | **SPI1_MISO** / Full-Duplex Master | MISO | CN47 | 外部 SPI 读 |
| 17 | PA7 | **SPI1_MOSI** / Full-Duplex Master | MOSI | CN47 | 外部 SPI 写 |
| 18 | PB0 | **ADC1_IN15** / IN15-Single-Ended | ADC_IW | U19 运放 | **W 相电流**，注入采样 |
| 19 | PB1 | **ADC1_IN12** / IN12-Single-Ended | ADC_IV | U15 运放 | **V 相电流**，注入采样 |
| 20 | PB2 | **ADC2_IN12** / IN12-Single-Ended | ADC_IBUS | U16 运放 | 母线电流，软件触发 |
| 22 | PB11 | **ADC1_IN14** / IN14-Single-Ended | ADC_VBUS | 电阻分压 | 母线电压监测 |
| 26 | PB13 | **TIM1_CH1N** / PWM Gen1 CH1 CH1N | LIN_W | FD6288 LIN3 | W 相下桥 PWM |
| 27 | PB14 | **TIM1_CH2N** / PWM Gen2 CH2 CH2N | LIN_V | FD6288 LIN2 | V 相下桥 PWM |
| 28 | PB15 | **TIM1_CH3N** / PWM Gen3 CH3 CH3N | LIN_U | FD6288 LIN1 | U 相下桥 PWM |
| 29 | PA8 | **TIM1_CH1** / PWM Gen1 CH1 CH1N | HIN_W | FD6288 HIN3 | W 相上桥 PWM |
| 30 | PA9 | **TIM1_CH2** / PWM Gen2 CH2 CH2N | HIN_V | FD6288 HIN2 | V 相上桥 PWM |
| 31 | PA10 | **TIM1_CH3** / PWM Gen3 CH3 CH3N | HIN_U | FD6288 HIN1 | U 相上桥 PWM |
| 32 | PA11 | **GPIO_Input** | KEY1 | SW2 按键 | 用户输入，内部上拉可选 |
| 33 | PA12 | **TIM4_CH2** 或 **GPIO_Output** | RGB | WS2812B DIN | ⚠️ 见 [TIM4/WS2812 说明](#5-tim4--低频环--rgb重点) |
| 34 | PA13 | **SYS_JTMS-SWDIO** / Serial Wire | — | CN9 / 调试器 | SWD 下载调试，勿改 |
| 35 | PA14 | **SYS_JTCK-SWCLK** / Serial Wire | — | CN9 / 调试器 | SWD 时钟，勿改 |
| 36 | PA15 | **GPIO_Output** | MT6816_CSN | MT6816 Pin1 | SPI3 软件片选，**必须 GPIO 输出** |
| 37 | PB3 | **SPI3_SCK** / Full-Duplex Master | MT6816_SCK | MT6816 Pin7 | 编码器 SPI 时钟 |
| 38 | PB4 | **SPI3_MISO** / Full-Duplex Master | MT6816_MISO | MT6816 Pin6 | 编码器 SPI 读 |
| 39 | PB5 | **SPI3_MOSI** / Full-Duplex Master | MT6816_MOSI | MT6816 Pin5 | 编码器 SPI 写 |
| 40 | PB6 | **USART1_TX** / Asynchronous | — | CN9 Pin3 | 串口发送 |
| 41 | PB7 | **USART1_RX** / Asynchronous | — | CN9 Pin2 | 串口接收 |
| 42 | PB8 | **FDCAN1_RX** / FDCAN_Activate | — | CAN 收发器 | CAN 接收 |
| 43 | PB9 | **FDCAN1_TX** / FDCAN_Activate | — | CAN 收发器 | CAN 发送 |

---

## 3. RCC 时钟

### 3.1 引脚

| 引脚 | 模式 | 原因 |
|------|------|------|
| PF0 | HSE External Oscillator (IN) | 8MHz 无源晶振，比 HSI 精度高，FOC 定时更准确 |
| PF1 | HSE External Oscillator (OUT) | 晶振回路输出 |

### 3.2 时钟树参数

| 参数 | 配置值 | 原因 |
|------|--------|------|
| HSE | **8 MHz** | 与板载 X1 晶振一致 |
| PLL Source | HSE | |
| PLLM | **/2** | 8MHz÷2 = 4MHz 进 PLL |
| PLLN | **×85** | 4MHz×85 = 340MHz VCO |
| PLLR | **/2** | SYSCLK = **170 MHz** |
| SYSCLK | 170 MHz | G4 最高性能，PWM/ADC 带宽充足 |
| AHB / APB1 / APB2 | 不分频 | Timer 时钟 = 170 MHz |
| ADC12 Clock | SYSCLK | ADC 转换速度快 |

**为什么 170MHz：** FOC 电流环 10~20kHz，高主频保证 TIM/ADC/SPI 时序余量。

---

## 4. TIM1 — 三相 PWM + ADC 触发（重点）

### 4.1 引脚配置（Pinout 视图）

在 Pinout 中点击 **TIM1**，勾选：

| 通道 | 引脚 | CubeMX 模式名 | 原因 |
|------|------|---------------|------|
| CH1 | PA8 | PWM Generation1 **CH1 CH1N** | 互补 PWM：HIN_W + LIN_W |
| CH2 | PA9 | PWM Generation2 **CH2 CH2N** | HIN_V + LIN_V |
| CH3 | PA10 | PWM Generation3 **CH3 CH3N** | HIN_U + LIN_U |
| CH4 | （无引脚） | **PWM Generation4 No Output** | 仅内部 OC4，触发 ADC |

> **关键：** 必须选带 **CHxN** 的互补模式，才会同时映射高侧 (PA) 和低侧 (PB) 引脚。  
> FD6288 需要独立的 HIN/LIN 信号，不是单端 PWM。

### 4.2 TIM1 参数配置（Configuration 页）

| 参数 | 应设值 | 原因 |
|------|--------|------|
| Clock Source | Internal Clock | 标准 PWM 源 |
| **Counter Mode** | **Center Aligned mode 1** | 中心对齐：对称 PWM，电流采样窗口在周期中心，噪声小 |
| Prescaler | **0** | 不分频，170MHz 满精度 |
| **Period (ARR)** | **4250** | PWM 频率 ≈ 170M/(2×4250) ≈ **20 kHz** |
| Clock Division | DIV2 | 仅影响 dead-time 时钟，当前工程已设 |
| **Auto-reload preload** | Enable | 占空比缓冲更新，避免 glitch |
| Repetition Counter | 0 | 每个周期更新 |

#### Channel 1/2/3（PWM 输出）

| 参数 | 值 | 原因 |
|------|-----|------|
| Mode | PWM Generation CHx / CHxN | 互补输出 |
| Pulse | 0（初始） | 上电 0% 占空比，安全 |
| OC Mode | PWM1 | 高侧有效高 |
| OC Polarity | High | 与 FD6288 同相逻辑 |
| OCN Polarity | High | 低侧同理 |
| OC/OCN Idle State | Reset | 空闲时关断 |

#### Channel 4（ADC 触发，无引脚）

| 参数 | 值 | 原因 |
|------|-----|------|
| Mode | **PWM Generation4 No Output** | 不占用引脚 |
| **OC Mode** | **PWM2** | 配合 TRGO 在特定时刻产生触发边沿 |
| Pulse | 可设 ARR-1 或 0 | 决定 OC4REF 触发时刻 = 采样点 |

#### Break and Dead Time

| 参数 | 值 | 原因 |
|------|-----|------|
| **Dead Time** | **9**（可微调） | 防止上下桥直通；FD6288 内部还有 200ns 死区 |
| Break State | Disable | 本板无 TIM1 Break 引脚接驱动 Fault |
| Automatic Output | **Enable** | 运行模式自动使能输出 |
| OffState Run/Idle | Enable (OSSR/OSSI) | 空闲态安全关断 |

#### Master Mode（触发 ADC）

| 参数 | 值 | 原因 |
|------|-----|------|
| **Trigger Output (TRGO)** | **OC4REF** | 将 TIM1_CH4 比较事件输出为触发信号 |
| → 连接到 ADC1 注入触发 | TIM1_CC4 上升沿 | 硬件同步采样，不占用 CPU |

### 4.3 为什么 TIM1 是 FOC 核心

```
         ┌─── TIM1 更新中断 ──→ SguanFOC_High_Loop()  (电流环)
         │
TIM1 ────├─── CH1/2/3 PWM ────→ FD6288 ──→ 电机
         │
         └─── CH4 OC4REF ─────→ ADC1 注入 (Iv, Iw)
```

- **中心对齐 PWM**：采样点固定在低侧导通窗口中间，电流纹波最小
- **CH4 硬件触发 ADC**：采样时刻与 PWM 严格同步，这是 FOC 稳定的前提
- **20kHz**：兼顾开关损耗与电流环带宽（SguanFOC `TIM_T=1e-4` 即 10kHz 控制）

### 4.4 NVIC

| 中断 | 建议优先级 | 用途 |
|------|:----------:|------|
| TIM1 Update (UP) | **0**（最高） | `SguanFOC_High_Loop()` |
| TIM1 CC | 0 | 若用 CC 中断 |
| TIM1 Break | 0 | 预留 Fault |

---

## 5. TIM4 — 低频环 / RGB（重点）

### 5.1 当前配置 vs 建议

| 项目 | 当前工程 | 建议（FOC 移植） | 原因 |
|------|----------|------------------|------|
| PA12 模式 | TIM4_CH2 PWM | **Internal Clock 定时中断** 或 GPIO | WS2812B 不能用普通 PWM |
| Counter Mode | Up | Up | 低频环用递增即可 |
| Prescaler | 0 | **169** | 170MHz÷170 = 1MHz |
| Period | 65535 | **999** | 1MHz÷1000 = **1kHz** |
| 中断 | 未启用 | **Enable Update IRQ** | `SguanFOC_Low_Loop()` |

### 5.2 两种用途二选一

#### 方案 A：SguanFOC 低频环（推荐优先）

| 配置 | 值 |
|------|-----|
| PA12 | **释放** 或 GPIO（RGB 后期用 SPI 驱动 WS2812） |
| TIM4 | 仅 Internal Clock，不开 PWM 引脚 |
| 频率 | 1 kHz 更新中断 |
| 用途 | 状态机、保护、速度环外环 |

#### 方案 B：WS2812B RGB 灯

| 配置 | 值 |
|------|-----|
| PA12 | **GPIO_Output**（不用 TIM4 PWM） |
| 驱动方式 | SPI+DMA 或 bit-bang 模拟 WS2812 时序 |
| 原因 | WS2812 是 800Kbps 单线协议，**TIM4 占空比 PWM 无效** |

### 5.3 为什么需要 TIM4 低频环

SguanFOC 架构：
- **High_Loop (TIM1, ~10kHz)**：电流环、坐标变换、PWM
- **Low_Loop (TIM4, ~1kHz)**：状态机切换、故障保护、速度环慢任务

---

## 6. ADC1 / ADC2 — 电流采样（重点）

### 6.1 引脚（Analog 模式）

| 引脚 | 信号 | CubeMX 模式 | 原因 |
|------|------|-------------|------|
| PB1 | ADC1_IN12 | IN12 Single-Ended | V 相电流运放输出 → **注入 Rank1** |
| PB0 | ADC1_IN15 | IN15 Single-Ended | W 相电流运放输出 → **注入 Rank2** |
| PB11 | ADC1_IN14 | IN14 Single-Ended | VBUS 分压 → 常规慢速采样 |
| PA2 | ADC1_IN3 | IN3 Single-Ended | NTC 温度 |
| PA3 | ADC1_IN4 | IN4 Single-Ended | 外部 NTC |
| PB2 | ADC2_IN12 | IN12 Single-Ended | IBUS 母线电流 → ADC2 常规 |

> Pinout 中选 **ADC1/ADC2 → INx Single-Ended**，引脚自动变为 Analog。

### 6.2 ADC1 参数

| 参数 | 值 | 原因 |
|------|-----|------|
| Clock Prescaler | Synchronous /4 | ADC 时钟在规格内 |
| Resolution | **12 bits** | 与 SguanFOC `ADC_Precision=4096` 一致 |
| Data Align | Right | HAL 标准 |
| Scan Conversion Mode | **Enable** | 多通道常规扫描 |
| Continuous Mode | **Disable** | 注入由 TIM 触发，常规由软件触发 |
| DMA | Disable（注入） | FOC 在中断里读注入 DR |
| **Injected Conversion** | **Enable** | FOC 电流采样必须用注入组 |
| Number of Injected | **2** | Iv + Iw 双通道 |
| External Trigger Injected | **TIM1_CC4 Rising** | 与 TIM1 CH4 硬件同步 |
| Sampling Time | **47.5 Cycles** | shunt+运放输出阻抗低，47.5 足够 |

#### ADC1 注入 Rank

| Rank | Channel | 引脚 | 信号 |
|:----:|:-------:|:----:|:----:|
| 1 | ADC1_IN12 | PB1 | Iv |
| 2 | ADC1_IN15 | PB0 | Iw |

#### ADC1 常规 Rank（慢速）

| Rank | Channel | 引脚 | 信号 |
|:----:|:-------:|:----:|:----:|
| 1 | ADC1_IN14 | PB11 | VBUS |
| 2 | ADC1_IN3 | PA2 | NTC1 |
| 3 | ADC1_IN4 | PA3 | NTC2 |

### 6.3 ADC2 参数

| 参数 | 值 | 原因 |
|------|-----|------|
| Injected | **Disable** | IBUS 不需要同步 PWM |
| Regular Rank1 | ADC2_IN12 (PB2) | 软件触发读母线电流 |
| 其余同 ADC1 | 12bit, 47.5 cycles | |

### 6.4 为什么注入 + TIM1_CC4

| 若用常规 + 软件触发 | 问题 |
|---------------------|------|
| 采样时刻随机 | 电流波形畸变，Id/Iq 不准 |
| 与 PWM 不同步 | FOC 振荡甚至失控 |

**注入组 + TIM1_CC4 硬件触发** = 每次在 PWM 周期固定位置采样 → FOC 标准做法。

### 6.5 NVIC

| 中断 | 优先级 | 用途 |
|------|:------:|------|
| ADC1_2 | 1 | 注入转换完成（可选，也可在 TIM1 中断读 DR） |

---

## 7. SPI1 — 外部 SPI 扩展

### 7.1 引脚

| 引脚 | 信号 | CubeMX 模式 | 原因 |
|------|------|-------------|------|
| PA5 | SPI1_SCK | Full-Duplex **Master** | 时钟 |
| PA6 | SPI1_MISO | Full-Duplex Master | 主读从 |
| PA7 | SPI1_MOSI | Full-Duplex Master | 主写从 |
| PA4 | GPIO_Output | **软件 NSS** | CN47 片选，手动控制 |

### 7.2 SPI1 参数

| 参数 | 当前值 | 建议值 | 原因 |
|------|--------|--------|------|
| Mode | Master | Master | MCU 作主设备 |
| Direction | 2 Lines | 2 Lines | 全双工 |
| Data Size | ⚠️ 4 Bit | **8 Bit** | 常规 SPI 设备 |
| CPOL / CPHA | Mode 0 | 按外设手册 | 取决于 CN47 接的设备 |
| Prescaler | /4 | /4 ~ /32 | 42.5MHz 偏快，可按需降 |
| NSS | Soft | Soft | PA4 软件片选 |

> SPI1 与 FOC 无关，仅 CN47 扩展口；FOC 首启可不初始化。

---

## 8. SPI3 — MT6816 磁编码器（重点）

### 8.1 引脚

| 引脚 | 信号 | CubeMX 模式 | 原因 |
|------|------|-------------|------|
| PB3 | SPI3_SCK | Full-Duplex **Master** | MT6816 时钟 |
| PB4 | SPI3_MISO | Full-Duplex Master | 读角度数据 |
| PB5 | SPI3_MOSI | Full-Duplex Master | 发读命令 |
| PA15 | **GPIO_Output** | 软件 CSN | **必须 GPIO**，MT6816 片选低有效 |

> **PA15 不能配 SPI3_NSS**，MT6816 要求 CSN 独立 GPIO 控制，且读角度时序有 100ns setup 要求。

### 8.2 SPI3 参数（⚠️ 当前工程有误，必须修改）

| 参数 | 当前错误值 | **正确值** | 原因 |
|------|-----------|-----------|------|
| Mode | Master | Master | |
| Data Size | ❌ **4 Bit** | ✅ **16 Bit** | MT6816 SPI 帧为 16bit |
| **CPOL** | ❌ Low (Mode 0) | ✅ **High (Mode 3)** | MT6816 手册：SPI Mode 3 |
| **CPHA** | 1 Edge | ✅ **2 Edge (Mode 3)** | 同上 |
| Prescaler | /4 (42.5MHz) | **/8 或 /16** | MT6816 SCK 周期 ≥64ns，即 ≤15.6MHz |
| NSS | Soft | Soft | PA15 手动 |
| First Bit | MSB First | MSB First | MT6816 MSB 优先 |

### 8.3 为什么是 Mode 3 + 16bit

MT6816 数据手册明确：
- SPI **Mode 3**（CPOL=1, CPHA=1），空闲 SCK 高
- 16 位帧：bit0=R/W, bit1-7=地址, bit8-15=数据
- 读 0x03/0x04 寄存器拼成 14bit 角度

CubeMX 默认 Mode 0 + 4bit 无法与 MT6816 通信。

---

## 9. USART1 — 调试串口（重点）

### 9.1 引脚

| 引脚 | 信号 | CubeMX 模式 | 原因 |
|------|------|-------------|------|
| PB6 | USART1_TX | **Asynchronous** | 发送 → CN9 Pin3 |
| PB7 | USART1_RX | **Asynchronous** | 接收 → CN9 Pin2 |

> 在 Pinout 中点 **USART1 → Mode: Asynchronous**，PB6/PB7 自动分配。

### 9.2 USART1 参数

| 参数 | 值 | 原因 |
|------|-----|------|
| Baud Rate | **921600** | 当前 SguanFOC / JustFloat 配置 |
| Word Length | 8 Bits | 标准 |
| Parity | None | |
| Stop Bits | 1 | |
| Direction | TX + RX | 双向：printf + 上位机命令 |
| OverSampling | 16 | 标准 |
| Hardware Flow Control | None | 无 RTS/CTS |

### 9.3 为什么用 DMA

| 参数 | 配置 | 原因 |
|------|------|------|
| USART1_TX | DMA1 Channel2, Normal | JustFloat 大量浮点发送不阻塞 FOC |
| USART1_RX | DMA1 Channel4, Normal | 上位机命令后台接收 |

### 9.4 为什么选 PB6/PB7

- 硬件 CN9 调试口已固定 TX/RX
- PB6/PB7 无 TIM1/SPI 冲突
- AF7 = USART1（CubeMX 自动处理）

---

## 10. FDCAN1 — CAN 通信

### 10.1 引脚

| 引脚 | 信号 | CubeMX 模式 | 原因 |
|------|------|-------------|------|
| PB8 | FDCAN1_RX | **FDCAN_Activate** | CAN 收发器 RX |
| PB9 | FDCAN1_TX | **FDCAN_Activate** | CAN 收发器 TX |

> GIM6010 24V 电机规格书通信协议为 **CAN**，后期对接电机协议时使用。

### 10.2 FDCAN1 参数

| 参数 | 当前值 | 说明 |
|------|--------|------|
| Frame Format | Classic CAN | 标准帧 |
| Mode | Normal | 正常模式 |
| Nominal Prescaler | 16 | 当前约 3.54 Mbps |
| Data Prescaler | 1 | Classic 模式不用 |

> 波特率需与电机 CAN 协议文档一致，FOC 首启阶段可不初始化 FDCAN。

---

## 11. DMA 配置

| DMA 通道 | 请求 | 方向 | 模式 | 原因 |
|----------|------|------|------|------|
| DMA1 Ch1 | **USART1_TX** | Memory→Periph | Normal | 串口发送 |
| DMA1 Ch2 | **USART1_RX** | Periph→Memory | Normal | 串口接收 |

> ADC 注入采样不用 DMA，在 TIM1 中断中直接读 `HAL_ADCEx_InjectedGetValue()`。

---

## 12. NVIC 中断优先级

| 中断 | Preempt Priority | 用途 |
|------|:----------------:|------|
| **TIM1 UP/CC** | **0** | SguanFOC 高频电流环 |
| ADC1_2 | 1 | ADC 完成（可选） |
| **TIM4 UP** | **2** | SguanFOC 低频环（启用 TIM4 中断后） |
| USART1 | 3 | 串口 / SguanFOC_Printf_Loop |
| DMA1 Ch1/2 | 3 | 串口 DMA |
| SysTick | 15 | HAL 时基（最低） |

**原则：** 电流环 > 速度/状态机 > 通信 > HAL 时基。

---

## 13. GPIO 独立引脚

| 引脚 | 模式 | 输出/输入 | 初始电平 | 原因 |
|------|------|-----------|----------|------|
| PA4 | GPIO_Output | CSN | **High** | SPI 片选 idle 高 |
| PA15 | GPIO_Output | MT6816_CSN | **High** | 编码器片选 idle 高 |
| PA11 | GPIO_Input | KEY1 | — | 按键，建议 **Pull-up** |
| PA12 | GPIO_Output 或 TIM | RGB | Low | WS2812 方案见 TIM4 章节 |

### PA13 / PA14 — 不要改

| 引脚 | 模式 | 原因 |
|------|------|------|
| PA13 | SYS_JTMS-SWDIO | ST-Link 下载调试 |
| PA14 | SYS_JTCK-SWCLK | ST-Link 时钟 |

---

## 14. CubeMX 操作步骤速查

### 14.1 从零配置 FOC 最小集（推荐顺序）

```
1. RCC → HSE ON → PLL → 170MHz
2. Pinout → TIM1 → 选 CH1/2/3 (互补) + CH4 (No Output)
3. Pinout → ADC1 → IN12, IN15, IN14, IN3, IN4
4. Pinout → ADC2 → IN12
5. Pinout → SPI3 → Full-Duplex Master (PB3/4/5)
6. Pinout → PA15 → GPIO_Output (MT6816_CSN)
7. Pinout → USART1 → Async (PB6/7)
8. Pinout → TIM4 → Internal Clock（不开 PA12 PWM，若做低频环）

9. TIM1 Config → Center Aligned, ARR=4250, CH4 PWM2, TRGO=OC4REF, DeadTime=9
10. ADC1 Config → Injected×2, Trigger=TIM1_CC4, Rank Iv/Iw
11. SPI3 Config → Mode3, 16bit, Prescaler /8
12. USART1 → 921600, 开 DMA TX/RX
13. TIM4 Config → PSC=169, ARR=999, 开 Update Interrupt
14. NVIC → TIM1=0, TIM4=2, ADC=1, USART=3
15. Generate Code
```

### 14.2 Pinout 视图颜色含义

| 颜色 | 含义 |
|------|------|
| 绿色 | 已分配外设功能 |
| 黄色 | 有冲突 / 需选 AF |
| 灰色 | 未使用 |

---

## 15. 当前工程待修正项

| # | 外设 | 问题 | 应改为 |
|---|------|------|--------|
| 1 | **SPI3** | DataSize = 4 Bit | **16 Bit** |
| 2 | **SPI3** | SPI Mode 0 | **SPI Mode 3** |
| 3 | **SPI3** | 42.5 MHz 过快 | Prescaler **/8** 或 **/16** |
| 4 | **SPI1** | DataSize = 4 Bit | **8 Bit** |
| 5 | **TIM4** | Period=65535，仅 RGB PWM | **1kHz 中断**（PSC=169, ARR=999） |
| 6 | **TIM4** | 无 Update 中断 | **Enable** + NVIC |
| 7 | **PA12** | TIM4_CH2 PWM | WS2812 需 **GPIO+SPI**，不能靠 TIM4 占空比 |

修正后重新 **Generate Code**，USER CODE 区域内容会保留。

---

## 16. 修订记录

| 日期 | 版本 | 说明 |
|------|------|------|
| 2026-07-28 | v1.0 | 初版，全引脚 + 关键外设 TIM/SPI/UART/ADC 配置及原因 |

**相关文档：**
- `GIM6010_引脚与外设对照表.md`
- `GIM6010_BSP板级支持编写指南.md`
- `GIM6010_SguanFOC_AI辅助移植操作指南.md`

---

<p align="center"><b>CubeMX 配置指南 — 按本文档配，FOC 硬件层一次到位</b></p>
