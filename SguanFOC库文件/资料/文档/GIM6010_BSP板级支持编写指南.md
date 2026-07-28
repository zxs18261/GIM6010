# GIM6010 BSP 板级支持编写指南

> 基于板载关键芯片数据手册及硬件网表整理，说明如何编写各外设 BSP 文件，为 SguanFOC 移植和后续驱动开发打基础。  
> 文档路径：`E:\github\GIM6010\SguanFOC库文件\资料\文档`  
> 数据手册路径：`资料\硬件资料\硬件电路板相关文件\对应芯片数据手册`

---

## 目录

1. [BSP 总体架构](#1-bsp-总体架构)
2. [芯片清单与手册对照](#2-芯片清单与手册对照)
3. [bsp_board — 板级初始化](#3-bsp_board--板级初始化)
4. [bsp_pwm — 三相 PWM（FD6288 + TIM1）](#4-bsp_pwm--三相-pwmfd6288--tim1)
5. [bsp_adc — 电流/电压/温度（INA181A）](#5-bsp_adc--电流电压温度ina181a)
6. [bsp_encoder — 磁编码器（MT6816）](#6-bsp_encoder--磁编码器mt6816)
7. [bsp_uart — 调试串口（USART1）](#7-bsp_uart--调试串口usart1)
8. [bsp_can — CAN 通信（FDCAN1）](#8-bsp_can--can-通信fdcan1)
9. [bsp_rgb — 状态灯（WS2812B）](#9-bsp_rgb--状态灯ws2812b)
10. [bsp_key — 用户按键](#10-bsp_key--用户按键)
11. [与 SguanFOC UserData_Function.h 对接](#11-与-sguanfoc-userdata_functionh-对接)
12. [CubeMX 需修正项汇总](#12-cubemx-需修正项汇总)
13. [建议目录结构与文件模板](#13-建议目录结构与文件模板)
14. [修订记录](#14-修订记录)

---

## 1. BSP 总体架构

### 1.1 设计原则

```
┌─────────────────────────────────────────────────────┐
│  Application / SguanFOC                             │
│  UserData_Function.h  ← 调用 BSP 接口               │
├─────────────────────────────────────────────────────┤
│  BSP 层（本指南）                                    │
│  bsp_pwm / bsp_adc / bsp_encoder / bsp_uart / ...   │
├─────────────────────────────────────────────────────┤
│  HAL 层（CubeMX 生成，尽量不修改）                   │
│  tim.c / adc.c / spi.c / usart.c / fdcan.c          │
└─────────────────────────────────────────────────────┘
```

| 原则 | 说明 |
|------|------|
| BSP 封装硬件细节 | 上层只调用 `BSP_Encoder_GetRad()`，不关心 SPI 寄存器 |
| 不修改 CubeMX 生成文件 | 时钟、GPIO、DMA 初始化留在 `Core/Src` |
| 一芯片一模块 | 每个关键芯片对应一对 `.c/.h` |
| 可单元测试 | BSP 函数输入输出明确，便于示波器/串口验证 |

### 1.2 建议 BSP 模块列表

| 模块 | 文件 | 对应芯片/外设 | 优先级 |
|------|------|--------------|:------:|
| 板级初始化 | `bsp_board.c/h` | 全局 | P0 |
| 电机 PWM | `bsp_pwm.c/h` | FD6288 + TIM1 | P0 |
| 电流/电压采样 | `bsp_adc.c/h` | INA181A + ADC1/2 | P0 |
| 磁编码器 | `bsp_encoder.c/h` | MT6816 + SPI3 | P0 |
| 调试串口 | `bsp_uart.c/h` | USART1 | P1 |
| CAN 通信 | `bsp_can.c/h` | FDCAN1 + 收发器 | P2 |
| RGB 状态灯 | `bsp_rgb.c/h` | WS2812B | P2 |
| 用户按键 | `bsp_key.c/h` | GPIO KEY1 | P3 |

---

## 2. 芯片清单与手册对照

| 位号 | 芯片型号 | 手册文件 | 功能 | BSP 模块 |
|------|----------|----------|------|----------|
| U1 | STM32G431CBT6 | `STM32G431CBT6TR.pdf` | 主控 MCU | HAL（CubeMX） |
| U2 | **FD6288Q** | `FD6288.pdf` | 三相栅极驱动 | `bsp_pwm` |
| U3 | **MT6816** | `MT6816.pdf` | 14bit 磁编码器 | `bsp_encoder` |
| U15/U19 | **INA181Ax** | `INA181A.pdf` | V/W 相电流运放 | `bsp_adc` |
| U16 | **INA181Ax** | `INA181A.pdf` | 母线 IBUS 运放 | `bsp_adc` |
| U18 | 1.65V 基准 | （网表） | 电流采样偏置 | `bsp_adc` 常量 |
| Q1~Q13 | **SP80N03BGHNK** | `SP80N03BGHNK.pdf` | 80V N-MOSFET | 参数参考，无独立 BSP |
| LED1 | **WS2812B** | `WS2812B LED.pdf` | RGB 状态灯 | `bsp_rgb` |
| U9 | CAN 收发器 | （网表） | FDCAN 物理层 | `bsp_can` |

---

## 3. bsp_board — 板级初始化

### 3.1 职责

- 调用 CubeMX 生成的 `MX_*_Init()`
- 按正确顺序启动各 BSP 子模块
- 提供 `BSP_Board_Init()` 统一入口

### 3.2 推荐初始化顺序

```c
void BSP_Board_Init(void)
{
    /* 1. CubeMX 已完成的 HAL/外设初始化（main.c 中） */
    /* MX_GPIO_Init(); MX_DMA_Init(); MX_ADC1_Init(); ... */

    /* 2. BSP 子模块初始化 */
    BSP_ADC_Init();          // ADC 校准 + 注入通道准备
    BSP_PWM_Init();          // TIM1 PWM + 驱动器就绪
    BSP_Encoder_Init();      // MT6816 SPI + 上电等待 16ms
    BSP_UART_Init();         // DMA 接收启动（可选）
    /* BSP_CAN_Init(); */  // 后期启用
    /* BSP_RGB_Init(); */  // 后期启用
}
```

### 3.3 关键常量（bsp_board.h）

```c
#define BSP_VBUS_NOMINAL        24.0f       // 额定母线 V
#define BSP_SHUNT_R_OHM         0.005f      // 5mΩ 采样电阻 R2/R14/R16
#define BSP_IAMP_GAIN           20.0f       // INA181A1 = 20V/V（需实物确认丝印）
#define BSP_IAMP_VREF           1.65f       // U18 提供的 REF 偏置
#define BSP_ADC_VREF            3.3f        // MCU ADC 基准
#define BSP_ADC_RESOLUTION      4096        // 12bit
#define BSP_PWM_ARR             4250        // TIM1 满量程
#define BSP_GEAR_RATIO          8.0f        // 减速比（电机规格书）
#define BSP_MOTOR_POLE_PAIRS    14          // 极对数（24V 规格书）
```

---

## 4. bsp_pwm — 三相 PWM（FD6288 + TIM1）

### 4.1 FD6288 芯片要点（来自数据手册）

| 参数 | 值 | BSP 影响 |
|------|-----|----------|
| 封装 | QFN24（本板 U2） | — |
| 逻辑输入 | HIN1~3 / LIN1~3 | 接 MCU TIM1 |
| 逻辑兼容 | 3.3V/5V | STM32 3.3V 直接驱动 |
| VCC 工作范围 | 5.0 ~ 20 V | 由 12V/24V 母线供电 |
| 内置死区 | **200 ns（典型）** | MCU 侧 dead time 可设小值 |
| 直通防止 | 有 | 硬件保护，无需软件干预 |
| UVLO | VCC/VBS < 4.3V 关断 | 启动前确认母线电压 |
| 输出与输入 | **同相** | 占空比增大 → 高侧导通增加 |
| MCU 使能脚 | **无**（网表未接 EN/SD） | 上电即可用，无需 GPIO 使能 |

### 4.2 硬件连接（MCU → FD6288 → MOSFET）

| MCU TIM1 | 网络名 | FD6288 引脚 | 驱动 |
|----------|--------|-------------|------|
| CH1 (PA8) | HIN_W | HIN3 (Pin24) | W 相上桥 |
| CH2 (PA9) | HIN_V | HIN2 (Pin23) | V 相上桥 |
| CH3 (PA10) | HIN_U | HIN1 (Pin22) | U 相上桥 |
| CH1N (PB13) | LIN_W | LIN3 (Pin3) | W 相下桥 |
| CH2N (PB14) | LIN_V | LIN2 (Pin2) | V 相下桥 |
| CH3N (PB15) | LIN_U | LIN1 (Pin1) | U 相下桥 |
| CH4 (无引脚) | — | — | 触发 ADC 注入 |

### 4.3 bsp_pwm.h 接口设计

```c
void BSP_PWM_Init(void);
void BSP_PWM_Start(void);
void BSP_PWM_Stop(void);
void BSP_PWM_SetDuty(uint16_t duty_u, uint16_t duty_v, uint16_t duty_w);
void BSP_PWM_Enable(void);   // 若后续硬件加 EN 脚则在此实现
void BSP_PWM_Disable(void);
```

### 4.4 bsp_pwm.c 实现要点

```c
#include "tim.h"

void BSP_PWM_Init(void)
{
    /* CubeMX 已 MX_TIM1_Init() */
}

void BSP_PWM_Start(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    /* CH4 用于 ADC 触发，需 Start 但无引脚输出 */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}

void BSP_PWM_SetDuty(uint16_t u, uint16_t v, uint16_t w)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, u);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, v);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, w);
}
```

### 4.5 PWM 频率计算

```
f_tim = 170 MHz
Center Aligned Mode 1: f_pwm = f_tim / (2 × ARR)
                      = 170e6 / (2 × 4250)
                      ≈ 20 kHz
```

SguanFOC 控制周期 `TIM_T = 1e-4`（10 kHz）→ 可在 TIM1 更新中断或每 2 个 PWM 周期执行一次 High_Loop。

---

## 5. bsp_adc — 电流/电压/温度（INA181A）

### 5.1 INA181A 芯片要点（来自数据手册）

| 参数 | INA181A1（推测本板） | 说明 |
|------|---------------------|------|
| 增益 | **20 V/V**（A1） | 丝印需实物确认 A1/A2/A3/A4 |
| 共模范围 | -0.2 ~ +26 V | 适合下桥臂采样 |
| 带宽 | 350 kHz | 满足 20kHz PWM |
| 输出公式 | **VOUT = Gain × Vshunt + VREF** | 双向电流 |
| REF 引脚 | 外接基准 | 本板 U18 → **1.65 V** |
| 增益误差 | ±1% max | 标定可修正 |
| 供电 | 2.7 ~ 5.5 V | MCU 3.3V 域 |

> 文件名 `INA181A.pdf` 对应 A 系列；本板网表 U15/U16/U19 为 SOT-23-6，与 INA181 封装一致。  
> **当前 `UserData_Motor.h` 中 `Amplifier=10.0` 与 A1 的 20V/V 不符，BSP 中应改为 20.0 并实测验证。**

### 5.2 电流换算公式

```
Vshunt = I × Rshunt = I × 0.005
VOUT   = Gain × Vshunt + VREF
       = 20 × I × 0.005 + 1.65
       = 0.1 × I + 1.65

反算：
I = (VOUT - 1.65) / 0.1          [A]
I = (ADC_raw / 4096 × 3.3 - 1.65) / 0.1
```

### 5.3 采样量程估算（Gain=20, Rshunt=5mΩ, VREF=1.65V）

| 电流 | VOUT | 是否超 3.3V ADC |
|------|------|:---------------:|
| 0 A | 1.65 V | ✅ |
| +6.6 A（额定） | 2.31 V | ✅ |
| +16.5 A | 3.30 V | ⚠️ 上限 |
| +30.47 A（峰值） | 4.65 V | ❌ 饱和 |

> 峰值电流时 ADC 会饱和，FOC 需依赖 `Sguan.safe` 限流，或确认实际增益是否为 A2（50V/V）等。

### 5.4 硬件通道映射

| BSP 通道 | GPIO | ADC | 注入/常规 | 运放 | 信号 |
|----------|------|-----|-----------|------|------|
| `BSP_ADC_IV` | PB1 | ADC1_IN12 | **注入 Rank1** | U15 | V 相电流 |
| `BSP_ADC_IW` | PB0 | ADC1_IN15 | **注入 Rank2** | U19 | W 相电流 |
| `BSP_ADC_IBUS` | PB2 | ADC2_IN12 | 常规 | U16 | 母线电流 |
| `BSP_ADC_VBUS` | PB11 | ADC1_IN14 | 常规 | 分压 | 母线电压 |
| `BSP_ADC_NTC1` | PA2 | ADC1_IN3 | 常规 | NTC2 | 板载温度 |
| `BSP_ADC_NTC2` | PA3 | ADC1_IN4 | 常规 | P1 外接 | 外部温度 |

**FOC 双电阻采样：** Iv + Iw → 推算 Iu = -(Iv + Iw)，对应 SguanFOC `Current_Num = 1`（AC 相）。

### 5.5 注入采样触发

```
触发源：TIM1_CC4 上升沿（CubeMX 已配置）
采样时刻：PWM 中心/下桥臂导通窗口（需示波器验证）
```

### 5.6 bsp_adc.h 接口设计

```c
void BSP_ADC_Init(void);
void BSP_ADC_StartInjected(void);           // 启动注入转换

int16_t BSP_ADC_GetIvRaw(void);             // 注入 Rank1
int16_t BSP_ADC_GetIwRaw(void);             // 注入 Rank2
float   BSP_ADC_GetIvAmp(void);             // 换算为 A
float   BSP_ADC_GetIwAmp(void);             // 换算为 A

uint16_t BSP_ADC_GetVbusRaw(void);
float   BSP_ADC_GetVbusVolt(void);          // 含分压比标定

uint16_t BSP_ADC_GetNtcRaw(uint8_t ch);
float   BSP_ADC_GetNtcTemp(uint8_t ch);     // 需 NTC 曲线
```

### 5.7 bsp_adc.c 实现要点

```c
/* 注入通道读取（在 ADC 注入完成中断或 High_Loop 中） */
int16_t BSP_ADC_GetIvRaw(void)
{
    return (int16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
}

int16_t BSP_ADC_GetIwRaw(void)
{
    return (int16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
}

float BSP_ADC_RawToCurrent(int16_t raw)
{
    float vout = (float)raw / BSP_ADC_RESOLUTION * BSP_ADC_VREF;
    return (vout - BSP_IAMP_VREF) / (BSP_IAMP_GAIN * BSP_SHUNT_R_OHM);
}
```

### 5.8 VBUS 分压（网表 R19/R20）

```
R19 = 3 kΩ, R20 = 30 kΩ
分压比 ≈ 30 / (3 + 30) = 10/11
VBUS = Vadc × (33/3) = Vadc × 11  （需实物标定修正）
```

---

## 6. bsp_encoder — 磁编码器（MT6816）

### 6.1 MT6816 芯片要点（来自数据手册）

| 参数 | 值 | BSP 影响 |
|------|-----|----------|
| 分辨率 | **14 bit**（16384 步/圈） | 角度量化 |
| 接口 | **4 线 SPI**（出厂默认） | SPI3 + PA15 CSN |
| SPI 模式 | **Mode 3（CPOL=1, CPHA=1）** | ⚠️ CubeMX 当前为 Mode 0，必须改 |
| 最高转速 | 25,000 RPM | 远超本电机 |
| 上电延迟 | **16 ms** 后才输出有效数据 | `Init()` 中 `HAL_Delay(16)` |
| SCK 周期 | ≥ 64 ns（≈ 15.6 MHz max） | SPI 预分频 /4~ /8 |
| CSN Setup | CSN↓ 到首个 SCK↓ ≥ **100 ns** | 片选后短延时 |
| 角度公式 | θ = Σ(bi × 2^i) / 16384 × 360° | 见下方 |
| 寄存器 | 0x03[13:6] + 0x04[5:0] | 读角度 |
| 校验 | 0x04[0] 偶校验位 PC | 建议检查 |
| 弱磁报警 | 0x04[1] No_Mag_Warning | 磁体安装检测 |
| HVPP 引脚 | 正常工作时 **接地** | SPI 模式（原理图应已接 GND） |

### 6.2 硬件连接

| 信号 | MCU | MT6816 引脚 |
|------|-----|-------------|
| CSN | PA15 (GPIO) | Pin1 |
| MOSI | PB5 (SPI3) | Pin5 (A/U) |
| MISO | PB4 (SPI3) | Pin6 (B/V) |
| SCK | PB3 (SPI3) | Pin7 (Z/W) |

### 6.3 SPI 读角度流程（4 线 SPI）

```
1. CSN = 0
2. 延时 ≥ 100 ns
3. 发送读命令帧：bit0=1(R/W读), bit1-7=地址, bit8-15=0x00
   - 读 0x03：0x80 | 0x03 << 1 → 发送 0x86 或按手册 16bit 帧格式
4. 接收 0x03 数据（Angle<13:6>）
5. 读 0x04：获取 Angle<5:0> + No_Mag_Warning + PC
6. CSN = 1
7. 拼接 14bit 角度，校验 PC
8. 转换为弧度：rad = angle14 / 16384.0f × 2π
```

**14 位角度拼接：**

```c
uint16_t angle14 = ((uint16_t)(reg03 & 0xFF) << 6) | ((reg04 >> 2) & 0x3F);
float rad = (float)angle14 / 16384.0f * 2.0f * 3.14159265f;
```

### 6.4 CubeMX SPI3 必须修改

| 参数 | 当前错误值 | 正确值 |
|------|-----------|--------|
| DataSize | 4 Bit | **16 Bit** |
| CLKPolarity | Low (Mode 0) | **High (Mode 3)** |
| CLKPhase | 1Edge | **2Edge (Mode 3)** |
| BaudRatePrescaler | /4 | **/8 或 /16**（≤15MHz） |
| NSS | Soft | Soft（PA15 手动控制） |

### 6.5 bsp_encoder.h 接口设计

```c
void     BSP_Encoder_Init(void);
uint16_t BSP_Encoder_GetRaw14(void);       // 0~16383
float    BSP_Encoder_GetRad(void);         // 0~2π 电机轴
float    BSP_Encoder_GetOutputRad(void);   // 0~2π 输出轴（÷减速比）
int8_t   BSP_Encoder_CheckMagnet(void);    // 0=OK, -1=弱磁
```

### 6.6 bsp_encoder.c 实现骨架

```c
#include "spi.h"
#include "main.h"

static void MT6816_CSN_Low(void)  { HAL_GPIO_WritePin(MT6816_CSN_GPIO_Port, MT6816_CSN_Pin, GPIO_PIN_RESET); }
static void MT6816_CSN_High(void) { HAL_GPIO_WritePin(MT6816_CSN_GPIO_Port, MT6816_CSN_Pin, GPIO_PIN_SET); }

static uint8_t MT6816_ReadReg(uint8_t addr)
{
    uint8_t tx[2] = { (uint8_t)(0x80 | (addr << 1)), 0x00 };
    uint8_t rx[2] = { 0 };
    MT6816_CSN_Low();
    HAL_SPI_TransmitReceive(&hspi3, tx, rx, 2, 10);
    MT6816_CSN_High();
    return rx[1];  /* 第二字节为数据，具体以手册时序为准 */
}

void BSP_Encoder_Init(void)
{
    MT6816_CSN_High();
    HAL_Delay(16);   /* MT6816 上电准备时间 TPwrUp */
}

float BSP_Encoder_GetRad(void)
{
    uint8_t reg03 = MT6816_ReadReg(0x03);
    uint8_t reg04 = MT6816_ReadReg(0x04);
    uint16_t angle14 = ((uint16_t)reg03 << 6) | ((reg04 >> 2) & 0x3F);
    return (float)angle14 / 16384.0f * 6.2831853f;
}
```

> 注意：MT6816 SPI 为 16 位帧连续传输，具体 TX/RX 字节序需对照手册图-19 微调。

---

## 7. bsp_uart — 调试串口（USART1）

### 7.1 硬件连接

| 信号 | GPIO | 连接器 |
|------|------|--------|
| TX | PB6 | CN9 Pin3 |
| RX | PB7 | CN9 Pin2 |

### 7.2 当前 CubeMX 配置

| 参数 | 值 |
|------|-----|
| 波特率 | 115200 |
| DMA TX | DMA1_Channel1 |
| DMA RX | DMA1_Channel2 |

### 7.3 bsp_uart.h 接口设计

```c
void BSP_UART_Init(void);
void BSP_UART_Send(const uint8_t *data, uint16_t len);
void BSP_UART_ReceiveIT(uint8_t *buf, uint16_t len);
/* 可选：printf 重定向 */
int BSP_UART_PutChar(int ch);
```

### 7.4 与 SguanFOC 关系

- `SguanFOC_Printf_Loop()` 在 USART 中断中调用
- JustFloat 上位机协议由 `Sguan_printf.c` 处理
- BSP 层只需提供底层 Send/Receive

---

## 8. bsp_can — CAN 通信（FDCAN1）

### 8.1 硬件

| 信号 | GPIO | 连接到 |
|------|------|--------|
| RX | PB8 | U9 CAN 收发器 |
| TX | PB9 | U9 CAN 收发器 |
| 终端电阻 | R96 = 120Ω | CAN_H / CAN_L |
| 外部接口 | U116 | XT30 4P |

### 8.2 电机规格

- GIM6010 24V 版通信协议：**CAN**
- 当前 CubeMX 波特率 ≈ 3.54 Mbps（需与电机协议文档对齐）

### 8.3 bsp_can.h 接口设计（后期实现）

```c
void BSP_CAN_Init(void);
void BSP_CAN_Send(uint32_t id, uint8_t *data, uint8_t len);
int  BSP_CAN_Receive(uint32_t *id, uint8_t *data, uint8_t *len);
```

> FOC 首启阶段建议暂不实现，先完成 USART 调试。

---

## 9. bsp_rgb — 状态灯（WS2812B）

### 9.1 WS2812B 芯片要点（来自数据手册）

| 参数 | 值 | ⚠️ 重要 |
|------|-----|---------|
| 协议 | **单线级联，800 Kbps** | 不是普通 PWM |
| 数据格式 | **24 bit GRB** | 绿-红-蓝顺序 |
| 复位时间 | LOW ≥ **100 µs** | 帧间间隔 |
| 0 码 | T0H ≈ 0.28 µs 高 + T0L ≈ 1.72 µs 低 | 时序严格 |
| 1 码 | T1H ≈ 0.9 µs 高 + T1L ≈ 1.1 µs 低 | 时序严格 |
| 供电 | 5.2 V typ（4.5~5.5 V） | 板上有电平/供电电路 |
| DIN | Pin2 | 接 PA12 |

### 9.2 ⚠️ CubeMX 配置问题

当前 PA12 配置为 **TIM4_CH2 PWM**，这对 WS2812B **无效**。  
WS2812B 需要 bit-bang 或 SPI+DMA 特殊时序，不能用占空比控制颜色。

### 9.3 推荐实现方式

| 方式 | 说明 | 推荐度 |
|------|------|:------:|
| SPI + DMA | 3.2MHz SPI 模拟 WS2812 时序 | ⭐⭐⭐ |
| 定时器 + DMA | TIM + GPIO 翻转 | ⭐⭐ |
| 关中断 bit-bang | 简单但影响 FOC 实时性 | ⭐ |

### 9.4 bsp_rgb.h 接口设计

```c
void BSP_RGB_Init(void);
void BSP_RGB_SetColor(uint8_t r, uint8_t g, uint8_t b);
void BSP_RGB_SetState(uint8_t state);   /* 预定义：运行/故障/待机 */
```

### 9.5 状态定义建议

| 状态 | 颜色 | 用途 |
|------|------|------|
| 待机 | 蓝 | 未使能 |
| 运行 | 绿 | FOC 闭环中 |
| 故障 | 红 | 过流/过压/过温 |
| 校准 | 黄 | 编码器/参数辨识 |

---

## 10. bsp_key — 用户按键

### 10.1 硬件

| 信号 | GPIO | 连接 |
|------|------|------|
| KEY1 | PA11 | SW2 用户按键 |

### 10.2 bsp_key.h 接口设计

```c
void     BSP_KEY_Init(void);
uint8_t  BSP_KEY_Read(void);          /* 0=按下, 1=释放（含消抖） */
```

---

## 11. 与 SguanFOC UserData_Function.h 对接

BSP 编写完成后，在 `UserData_Function.h` 中仅做薄封装：

```c
#include "bsp_board.h"
#include "bsp_pwm.h"
#include "bsp_adc.h"
#include "bsp_encoder.h"

static inline void User_InitialInit(void)
{
    BSP_PWM_Start();
    BSP_ADC_StartInjected();
}

static inline signed int User_ReadADC_Raw(unsigned char ch)
{
    switch (ch) {
    case 0: return BSP_ADC_GetIvRaw();
    case 1: return BSP_ADC_GetIwRaw();
    default: return 0;
    }
}

static inline float User_Encoder_ReadRad(void)
{
    return BSP_Encoder_GetRad();   /* 或 GetOutputRad() 视控制对象 */
}

static inline void User_PwmDuty_Set(unsigned short u, unsigned short v, unsigned short w)
{
    BSP_PWM_SetDuty(u, v, w);
}

static inline float User_VBUS_DataGet(void)
{
    return BSP_ADC_GetVbusVolt();
}

static inline float User_Temperature_DataGet(void)
{
    return BSP_ADC_GetNtcTemp(0);
}

static inline void User_Delay(unsigned int ms)
{
    HAL_Delay(ms);
}
```

---

## 12. CubeMX 需修正项汇总

| 外设 | 参数 | 当前 | 应改为 | 影响 |
|------|------|------|--------|------|
| SPI3 | DataSize | 4 Bit | **16 Bit** | MT6816 无法通信 |
| SPI3 | CLKPolarity/Phase | Mode 0 | **Mode 3** | MT6816 读数错误 |
| SPI1 | DataSize | 4 Bit | **8/16 Bit** | 外部 SPI 扩展 |
| TIM4 | 用途 | RGB PWM | **1kHz 中断** 或改 WS2812 方案 | 低频环 / RGB |
| TIM4 | Period | 65535 | 按 1kHz 计算 | SguanFOC_Low_Loop |

---

## 13. 建议目录结构与文件模板

```
GIM6010_V0.0/
├── BSP/
│   ├── bsp_board.c / bsp_board.h       ← 板级常量 + 总初始化
│   ├── bsp_pwm.c   / bsp_pwm.h         ← FD6288 / TIM1
│   ├── bsp_adc.c   / bsp_adc.h         ← INA181A / ADC1/2
│   ├── bsp_encoder.c / bsp_encoder.h   ← MT6816 / SPI3
│   ├── bsp_uart.c  / bsp_uart.h        ← USART1
│   ├── bsp_can.c   / bsp_can.h         ← FDCAN1（后期）
│   ├── bsp_rgb.c   / bsp_rgb.h         ← WS2812B（后期）
│   └── bsp_key.c   / bsp_key.h         ← KEY1
├── Core/          ← CubeMX 生成，不修改
├── SguanFOC/      ← 电机库 + UserData_*.h
└── MDK-ARM/       ← Keil 工程，添加 BSP 源文件
```

### bsp_xxx.h 统一模板

```c
#ifndef __BSP_XXX_H
#define __BSP_XXX_H

#include <stdint.h>

void BSP_XXX_Init(void);
/* 其他对外接口 */

#endif
```

### Keil 工程添加步骤

1. 右键 Target → Add Group → 命名 `BSP`
2. 添加 `BSP/*.c` 文件
3. Options → C/C++ → Include Paths 添加 `../BSP`

---

## 14. 修订记录

| 日期 | 版本 | 说明 |
|------|------|------|
| 2026-07-28 | v1.0 | 初版，基于 6 份芯片手册 + 网表 + 引脚对照表 |

**参考手册：**
- `对应芯片数据手册/FD6288.pdf`
- `对应芯片数据手册/MT6816.pdf`
- `对应芯片数据手册/INA181A.pdf`
- `对应芯片数据手册/SP80N03BGHNK.pdf`
- `对应芯片数据手册/STM32G431CBT6TR.pdf`
- `对应芯片数据手册/WS2812B LED.pdf`

**相关文档：**
- `GIM6010_引脚与外设对照表.md`
- `GIM6010_电机参数规格书(24V).md`
- `GIM6010_SguanFOC_AI辅助移植操作指南.md`

---

<p align="center"><b>GIM6010 BSP 编写指南 — 按模块实现，对接 SguanFOC</b></p>
