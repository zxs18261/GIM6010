# GIM6010 VOFA+ 调试对接指南

> 工程：`E:\github\GIM6010\SguanFOC库文件`  
> 协议：SguanFOC 内置 **JustFloat**（非 FireWater）  
> 适用：VOFA+ 波形观测、串口下行调参（AO/BO/CO）  
> 修订：2026-08-02（18 通道 + 开环 AO 调速）

---

## 目录

1. [概述](#1-概述)
2. [硬件与串口参数](#2-硬件与串口参数)
3. [软件数据流](#3-软件数据流)
4. [接口对接清单](#4-接口对接清单)
5. [18 通道映射](#5-18-通道映射)
6. [VOFA+ 使用步骤](#6-vofa-使用步骤)
7. [下行调参命令与开环运行](#7-下行调参命令与开环运行)
8. [与参考工程差异](#8-与参考工程差异)
9. [常见问题](#9-常见问题)
10. [修订记录](#10-修订记录)

---

## 1. 概述

本工程通过 **USART1 + JustFloat** 与 VOFA+ 通信：

| 方向 | 内容 | 实现 |
|------|------|------|
| 上行（板→PC） | **18** 路 float 波形 | `User_UserTX` → `Printf_Loop` → UART TX DMA |
| 下行（PC→板） | ASCII 命令改参数 | Idle-DMA 收包 → `Printf_Adjust` → `User_AO/BO/CO_Adjust` |

帧格式（JustFloat）：

```
[ float×18 ][ 00 00 80 7F ]
总长 = 18×4 + 4 = 76 字节
```

帧尾在 `Printf_Init()` 中初始化。

---

## 2. 硬件与串口参数

| 项 | 本工程取值 |
|----|------------|
| 外设 | USART1 |
| TX | **PB6** |
| RX | **PB7** |
| 波特率 | **115200** |
| 数据格式 | 8N1 |
| TX | DMA1_Channel1（`HAL_UART_Transmit_DMA`） |
| RX | DMA1_Channel2 + Idle（`HAL_UARTEx_ReceiveToIdle_DMA`） |
| 协议插件 | **JustFloat** |
| 通道数 | **18**（`CH_COUNT`：固定 12 + 调试扩展 6） |

接线：USB 转串口 TX↔板 RX，RX↔板 TX，GND 共地。

> ⚠️ 不要与参考工程 `手搓与AI编写` 混用波特率（那边是 **921600**）。

---

## 3. 软件数据流

### 3.1 上行发送

```
main while(1)
  └─ SguanFOC_main_Loop()
        ├─ Sguan_Start_Tick()          // 初始化（status==UNINITIALIZED 时）
        └─ Printf_Normal_Loop()        // 仅当 status∈(3,19)
              ├─ User_UserTX()         // 填充 Sguan.TXdata.fdata[0..17]
              └─ Printf_Loop()
                    └─ User_CorrespondSet()  // HAL_UART_Transmit_DMA
```

发送门槛（`SguanFOC.c`）：

```c
if (Sguan.status > 3 && Sguan.status < 19) {
    Printf_Normal_Loop(&Sguan);
}
```

即 `status` 至少为 `MOTOR_STATUS_IDLE (0x04)` 才会发数。

当前 `main.c`：

```c
Sguan.mode = Velocity_OPEN_MODE;
Sguan.status = MOTOR_STATUS_UNINITIALIZED;  // 走校准后进 IDLE，再发 JustFloat
```

### 3.2 下行接收

```
USART1 Idle 中断
  └─ USART1_IRQHandler → HAL_UART_IRQHandler
        └─ HAL_UARTEx_RxEventCallback()     // Hardware/Timer.c
              ├─ SguanFOC_Printf_Loop()
              │     └─ 检测到 '?' → Printf_Adjust()
              │           └─ User_AO/BO/CO_Adjust()
              └─ 重新启动 ReceiveToIdle_DMA
```

必须具备：

- `USART1_IRQn` NVIC 使能（`usart.c` MspInit）
- `USART1_IRQHandler`（`stm32g4xx_it.c`）
- DMA RX 中断（`DMA1_Channel2_IRQHandler`）

---

## 4. 接口对接清单

按文件逐项对接，缺一则 VOFA 无波形或无法调参。

### 4.1 发送驱动 — `SguanFOC/UserData_Function.h`

```c
static inline void User_CorrespondSet(unsigned char *ch, unsigned short int size)
{
    uint32_t tickstart = HAL_GetTick();
    while (huart1.gState != HAL_UART_STATE_READY) {
        if ((HAL_GetTick() - tickstart) > 20U) return;
    }
    (void)HAL_UART_Transmit_DMA(&huart1, ch, size);
}
```

要点：

- 使用 **DMA 发送**，勿与阻塞 `Transmit` 混用未等待完成
- 缓冲为全局 `Sguan.TXdata`，DMA 期间勿另开同缓冲发送
- 需 `#include "usart.h"`

### 4.2 通道填充 — `SguanFOC/UserData_UserControl.h`

在 `User_UserTX()` 中为 `Sguan.TXdata.fdata[0..17]` 赋值（见第 5 节）。

### 4.3 帧尾初始化 — `main.c` 或库初始化

```c
Printf_Init(&Sguan.TXdata);   // 写入 00 00 80 7F
```

库走 `Sguan_Start_Tick()` 完整初始化时也会调用；若跳过校准，须在 `main` 里手动调用。

### 4.4 主循环 — `main.c`

```c
while (1) {
    SguanFOC_main_Loop();   // 负责周期发 JustFloat
}
```

### 4.5 接收回调 — `Hardware/Timer.c`

```c
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1) {
        SguanFOC_Printf_Loop(Sguan_PrintfBuff, Size);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, Sguan_PrintfBuff, sizeof(Sguan_PrintfBuff));
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    }
}
```

`main` 中上电启动一次 RX：

```c
HAL_UARTEx_ReceiveToIdle_DMA(&huart1, Sguan_PrintfBuff, sizeof(Sguan_PrintfBuff));
__HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
```

### 4.6 中断 — CubeMX / `stm32g4xx_it.c`

| 中断 | 作用 |
|------|------|
| `USART1_IRQHandler` | Idle 收包 |
| `DMA1_Channel1_IRQHandler` | TX DMA 完成 |
| `DMA1_Channel2_IRQHandler` | RX DMA |

### 4.7 下行调参钩子 — `UserData_UserControl.h`（已实现）

| 函数 | 用途 |
|------|------|
| `User_AO_Adjust(float)` | 开环写 `Uq_in`；闭环写对应 Target |
| `User_BO_Adjust(float)` | 切换控制模式 |
| `User_CO_Adjust(float)` | &lt;0.5 停转清零；≥0.5 重新初始化 |

### 4.8 调试开关 — `UserData_Config.h`

```c
#define Open_Printf_Debug 0   // 0=JustFloat 正常 18 通道；1=文本状态 Debug
```

VOFA 看波形时务必保持 **0**。

---

## 5. 18 通道映射

`CH_COUNT = 18`：前 12 为固定名称，后 6 为调试扩展。

### 5.1 固定 12 通道

| Ch | 名称 | 变量 | 含义 |
|:--:|------|------|------|
| 0 | status | `Sguan.status` | 状态机（IDLE≈4） |
| 1 | Real_Speed | `encoder.Real_Speed` | 实际速度 [rad/s] |
| 2 | Target_Speed | `foc.Target_Speed` | 目标速度 |
| 3 | Real_Id | `current.Real_Id` | Id |
| 4 | Real_Iq | `current.Real_Iq` | Iq |
| 5 | Target_Id | `foc.Target_Id` | 目标 Id |
| 6 | Target_Iq | `foc.Target_Iq` | 目标 Iq |
| 7 | Uq_in | `foc.Uq_in` | **开环 Uq（AO 生效回显）** |
| 8 | Real_Ia | `current.Real_Ia` | Ia |
| 9 | Real_Pos | `encoder.Real_Pos` | 位置 |
| 10 | Pos_offset | `encoder.Pos_offset` | 角度偏置 |
| 11 | mode | `Sguan.mode` | 0=开环 |

### 5.2 扩展 6 通道（调试）

| Ch | 名称 | 变量 | 含义 |
|:--:|------|------|------|
| 12 | VBUS | `motor.VBUS` / Real_VBUS | 母线电压 [V]（当前多为配置 24） |
| 13 | Erad | `encoder.Real_Erad` | 电角度 [rad] |
| 14 | MechRad | MT6816 / Real_Rad | 机械角 [rad] |
| 15 | Edeg | Erad×180/π | 电角度 [deg] |
| 16 | MechDeg | MechRad×180/π | 机械角 [deg] |
| 17 | last_AO | `s_vofa_last_AO` | 最近一次 AO 指令值 |

### 推荐观察（开环）

| 目的 | 通道 |
|------|------|
| 指令是否进板 | Ch7 `Uq_in`、Ch17 `last_AO`（发 `AO=2.4?` 后两者≈2.4） |
| 模式/状态 | Ch0≈4、Ch11≈0 |
| 是否在转 | Ch14/16 机械角变化、Ch1 速度 |
| 母线 | Ch12≈24 |

---

## 6. VOFA+ 使用步骤

1. 安装并打开 **VOFA+**
2. 选择对应 COM 口
3. 波特率设为 **115200**，8N1
4. 协议插件选择 **JustFloat**
5. 通道数设为 **18**
6. 打开串口，添加示波器，勾选 Ch0/Ch7/Ch11/Ch12/Ch14/Ch17
7. 上电 **24V**，等待校准结束（约数秒，电机有一次回零拉齐）
8. 确认 Ch0≈4、Ch11≈0、Ch12≈24 后发开环命令

### 建议控件布局

- 示波器 1：Ch7 Uq_in、Ch17 last_AO  
- 示波器 2：Ch14 机械角 / Ch13 电角度  
- 示波器 3：Ch0 status、Ch11 mode、Ch12 VBUS  

---

## 7. 下行调参命令与开环运行

在 VOFA **发送窗口**发 ASCII（必须以 `?` 结尾）：

### 开环转起来（推荐顺序）

```text
BO=0?
AO=2.4?
```

| 步骤 | 命令 | 预期 VOFA 回显 |
|------|------|----------------|
| 确认开环 | `BO=0?` | Ch11 → 0 |
| 给 Uq | `AO=2.4?` | Ch17→2.4，Ch7→2.4，电机缓慢转 |
| 停转 | `CO=0?` 或 `AO=0?` | Ch7→0 |
| 重新校准 | `CO=1?` | 再次走初始化 |

> 开环 `|Uq|` 软件限幅为 `0.35×VBUS`（24V 时约 ±8.4V）。首次建议 **1.5~3.0**，勿一次给太大。

### 命令表

| 前缀 | 回调 | 含义 |
|------|------|------|
| `AO=` | `User_AO_Adjust` | mode0→Uq；mode1→Iq*；mode2→Speed*；mode3→Pos* |
| `BO=` | `User_BO_Adjust` | &lt;1 开环；&lt;2 电流；&lt;3 速度；否则位置 |
| `CO=` | `User_CO_Adjust` | &lt;0.5 清零停转；≥0.5 重新 UNINITIALIZED |

---

## 8. 与参考工程差异

| | 本工程（SguanFOC） | `手搓与AI编写` |
|--|-------------------|----------------|
| 波特率 | **115200** | **921600** |
| 协议实现 | `Sguan_printf.c` | `bsp_vofa.c` |
| 通道定义 | `User_UserTX` | `app_telemetry` / `tools/vofa/README.md` |
| 发送 | JustFloat 库内置 | `BSP_VOFA_SendJustFloat` |
| Shell | 无 | `vofa on/off` |

两边都是 JustFloat，但 **波特率与通道含义不同**，VOFA 工程配置不要混用。

---

## 9. 常见问题

| 现象 | 可能原因 | 处理 |
|------|----------|------|
| VOFA 完全无数据 | `status` 仍为 0（STANDBY） | `Printf_Init` + 置 `IDLE` 或走完整初始化 |
| 波形乱跳 / 拆帧失败 | 波特率错或混了文本 | 确认 115200；`Open_Printf_Debug=0` |
| 只有 TX 无 RX 调参 | 缺 USART1 中断 | 检查 `USART1_IRQHandler` 与 NVIC |
| 发送卡死 / 偶发丢帧 | DMA 未等完成又发 | `User_CorrespondSet` 等待 `gState==READY` |
| CubeMX 重生成后 RX 失效 | USART1 中断被关掉 | 在 CubeMX 勾选 USART1 global interrupt |
| 灯灭且无串口 | 其它模块 HardFault / Error_Handler | 先查 RGB DMA1_Ch3 中断是否被删 |
| 通道全 0 | FOC/编码器未接 | 正常；先确认 Ch0/Ch11 常数是否合理 |

### 上电自检清单

- [ ] USART1 115200，PB6/PB7 接线正确  
- [ ] `User_CorrespondSet` 为 DMA 发送  
- [ ] `User_UserTX` 已填充 18 通道  
- [ ] `Printf_Init` 已调用（校准流程内会调用）  
- [ ] `status > 3`（校准后 IDLE）  
- [ ] VOFA：JustFloat + **18** 通道 + 115200  
- [ ] `Open_Printf_Debug == 0`  
- [ ] `User_PwmDuty_Set` 已写 TIM1 CCR（否则电机不转）  

---

## 10. 修订记录

| 日期 | 说明 |
|------|------|
| 2026-08-02 | 18 通道；开环 AO/BO/CO 已对接；PWM/注入 ADC 补齐 |
| 2026-08-01 | 初版：汇总 JustFloat 对接、DMA 发送、VOFA 使用与排障 |

---

<p align="center"><b>VOFA+ × SguanFOC JustFloat — 先通链路，再看波形，后调参数</b></p>
