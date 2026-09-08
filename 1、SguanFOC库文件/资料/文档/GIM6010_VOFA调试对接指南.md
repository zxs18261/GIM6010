# GIM6010 VOFA+ 调试对接指南

> 工程：`E:\github\GIM6010\SguanFOC库文件`  
> 协议：SguanFOC 内置 **JustFloat**（非 FireWater）  
> 适用：VOFA+ 波形观测、串口下行调参（AO/BO/CO）  
> 修订：2026-08-24（921600 + 21 通道 + 实际 DMA 发送帧率）

---

## 目录

1. [概述](#1-概述)
2. [硬件与串口参数](#2-硬件与串口参数)
3. [软件数据流](#3-软件数据流)
4. [接口对接清单](#4-接口对接清单)
5. [21 通道映射](#5-21-通道映射)
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
| 上行（板→PC） | **21** 路 float 波形 | `User_UserTX` → `Printf_Loop` → UART TX DMA |
| 下行（PC→板） | ASCII 命令改参数 | Idle-DMA 收包 → `Printf_Adjust` → `User_AO/BO/CO_Adjust` |

帧格式（JustFloat）：

```
[ float×21 ][ 00 00 80 7F ]
总长 = 21×4 + 4 = 88 字节
```

帧尾在 `Printf_Init()` 中初始化。

---

## 2. 硬件与串口参数

| 项 | 本工程取值 |
|----|------------|
| 外设 | USART1 |
| TX | **PB6** |
| RX | **PB7** |
| 波特率 | **921600** |
| 数据格式 | 8N1 |
| TX | DMA1_Channel2（`HAL_UART_Transmit_DMA`） |
| RX | DMA1_Channel4 + Idle（`HAL_UARTEx_ReceiveToIdle_DMA`） |
| 协议插件 | **JustFloat** |
| 通道数 | **21**（`CH_COUNT`：固定 12 + 调试/限幅扩展 9） |

接线：USB 转串口 TX↔板 RX，RX↔板 TX，GND 共地。

> 当前源码 `Core/Src/usart.c` 与 CubeMX `.ioc` 均配置为 **921600**；VOFA+ 必须使用相同波特率。

---

## 3. 软件数据流

### 3.1 上行发送

```
main while(1)
  └─ SguanFOC_main_Loop()
        ├─ Sguan_Start_Tick()          // 初始化（status==UNINITIALIZED 时）
        └─ Printf_Normal_Loop()        // STANDBY、校准/运行态持续遥测；故障态降频遥测
              ├─ User_UserTX()         // 填充 Sguan.TXdata.fdata[0..20]
              └─ Printf_Loop()
                    └─ HAL_UART_Transmit_DMA() // 双缓冲启动发送，并写入 Ch6 实际帧率
```

发送门槛（`SguanFOC.c`）：

```c
if (((Sguan.status == MOTOR_STATUS_STANDBY) ||
     (Sguan.status >= MOTOR_STATUS_CALIBRATING)) &&
    (Sguan.status < 19U)) {
    Printf_Normal_Loop(&Sguan);
}
```

因此等待首次母线校准时可看到 `Ch0=3`，正常运行时为 `Ch0>=4`，执行 `CO=0?` 后仍会看到 `Ch0=0`；故障状态使用较低频率继续发送。

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
- DMA RX 中断（`DMA1_Channel4_IRQHandler`）

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
- `Printf_Loop()` 先复制到专用双缓冲，DMA 不直接读取正在更新的 `Sguan.TXdata`
- 需 `#include "usart.h"`

### 4.2 通道填充 — `SguanFOC/UserData_UserControl.h`

在 `User_UserTX()` 中为 `Sguan.TXdata.fdata[0..20]` 赋值（见第 5 节）；Ch6 在 `Printf_Loop()` 启动 DMA 前按实际发送间隔覆盖。

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
| `DMA1_Channel2_IRQHandler` | TX DMA 完成 |
| `DMA1_Channel4_IRQHandler` | RX DMA |

### 4.7 下行调参钩子 — `UserData_UserControl.h`（已实现）

| 函数 | 用途 |
|------|------|
| `User_AO_Adjust(float)` | 开环写 `Uq_in`；闭环写对应 Target |
| `User_BO_Adjust(float)` | 切换控制模式 |
| `User_CO_Adjust(float)` | &lt;0.5 停转清零；≥0.5 重新初始化 |

### 4.8 调试开关 — `UserData_Config.h`

```c
#define Open_Printf_Debug 0   // 0=JustFloat 正常 36 通道；1=文本状态 Debug
```

VOFA 看波形时务必保持 **0**。

---

## 5. 36 通道映射

`CH_COUNT = 36`：Ch0～Ch20保持原定义，Ch21～Ch35只从末尾追加校准和安全诊断量。

### 5.1 固定 12 通道

| Ch | 名称 | 变量 | 含义 |
|:--:|------|------|------|
| 0 | status | `Sguan.status` | 状态机（IDLE≈4） |
| 1 | Real_Speed | `encoder.Real_Speed` | 实际速度 [rad/s] |
| 2 | Target_Speed | `foc.Target_Speed` | 目标速度 |
| 3 | Real_Id | `current.Real_Id` | Id |
| 4 | Real_Iq | `current.Real_Iq` | Iq |
| 5 | Duty_u | `foc.Duty_u` | U 相 PWM 比较值 [count] |
| 6 | VOFA_TX_Hz | `20000 / Δg_high_loop_cnt` | 实际 VOFA 数据帧发送频率 [Hz]，首帧为 0 |
| 7 | Uq_in | `foc.Uq_in` | **开环 Uq（AO 生效回显）** |
| 8 | Real_Ia | `current.Real_Ia` | Ia |
| 9 | Real_Pos | `encoder.Real_Pos` | 位置 |
| 10 | Pos_offset | `encoder.Pos_offset` | 角度偏置 |
| 11 | mode | `Sguan.mode` | 0=开环 |

### 5.2 扩展 9 通道（调试/限幅）

| Ch | 名称 | 变量 | 含义 |
|:--:|------|------|------|
| 12 | VBUS | `motor.VBUS` / Real_VBUS | 母线电压 [V]（当前多为配置 24） |
| 13 | Erad | `encoder.Real_Erad` | 电角度 [rad] |
| 14 | MechRad | MT6816 / Real_Rad | 机械角 [rad] |
| 15 | Edeg | Erad×180/π | 电角度 [deg] |
| 16 | MechDeg | MechRad×180/π | 机械角 [deg] |
| 17 | last_AO | `s_vofa_last_AO` | 最近一次 AO 指令值 |
| 18 | Applied_Speed | `s_ol_applied_shaft_speed` | 开环实际用于积分的输出轴速度 [rad/s] |
| 19 | Peak_Torque | `GIM6010_OUTPUT_PEAK_TORQUE_NM` | 输出轴峰值转矩 [N·m] |
| 20 | Max_Speed | `GIM6010_SHAFT_MAX_RAD_S` | 输出轴软件速度上限 [rad/s] |

### 5.3 追加15个校准与安全诊断通道

- Ch21：校准结果码。0未判定、1等待母线、2正在校准、3本次校准保存成功、4从Flash加载、5编码器失败、6相序失败、7 Flash保存失败。
- Ch22：本次实际对齐Ud[V]；新校准约0.96 V，Flash加载为0 V。
- Ch23：最终相序检查结果；1通过、0未运行/失败、-1方向相反。
- Ch24：Flash校准有效标志；加载或保存回读成功后为1。
- Ch25：母线ADC有效标志。
- Ch26：复位后未滤波母线最小值[V]，用于捕获启动瞬时低样本。
- Ch27～Ch33：依次为`Encoder_Dir`、`Motor_Dir`、`PWM_Dir`、`Current_Dir0`、`Current_Dir1`、`Current_Num`、极对数。
- Ch34：TIM1硬件MOE；预定位结束、待机或故障后必须为0。
- Ch35：当前Ud[V]；预定位释放后必须为0。

Ch6 不是 VOFA 界面的绘图刷新率，而是控制器实际启动相邻两帧 JustFloat 数据发送的频率。计算使用 20 kHz ADC/High_Loop 计数器作为时基：

```text
Ch6 = 20000 / (当前帧 High_Loop 计数 - 上一帧 High_Loop 计数)
```

36通道数据帧共148 byte。921600-8N1每字节按10 bit计，理论物理上限约为`921600/(148×10)=622.7 Hz`；当前`USER_VOFA_TX_HZ=500`约占80.3%链路带宽，因此Ch6正常应在500 Hz附近。若主循环延迟或DMA/UART忙，会直接丢弃过期遥测并显示实际降低后的帧率，不阻塞控制环。

### 推荐观察（开环）

| 目的 | 通道 |
|------|------|
| 指令是否进板 | Ch7 `Uq_in`、Ch17 `last_AO`（发 `AO=2.4?` 后两者≈2.4） |
| 实际串口帧率 | Ch6 `VOFA_TX_Hz` |
| 模式/状态 | Ch0≈4、Ch11≈0 |
| 是否在转 | Ch14/16 机械角变化、Ch1 速度 |
| 母线 | Ch12≈24 |

---

## 6. VOFA+ 使用步骤

1. 安装并打开 **VOFA+**
2. 选择对应 COM 口
3. 波特率设为 **921600**，8N1
4. 协议插件选择 **JustFloat**
5. 通道数设为 **36**
6. 打开串口，添加示波器，优先勾选 Ch0/Ch7/Ch10/Ch12/Ch14/Ch21～Ch26/Ch34/Ch35
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
| `CO=` | `User_CO_Adjust` | &lt;0.5 清零并进入 STANDBY 高阻停机（Ch0=0，遥测继续）；≥0.5 重新 UNINITIALIZED |

---

## 8. 与参考工程差异

| | 本工程（SguanFOC） | `手搓与AI编写` |
|--|-------------------|----------------|
| 波特率 | **921600** | **921600** |
| 协议实现 | `Sguan_printf.c` | `bsp_vofa.c` |
| 通道定义 | `User_UserTX` | `app_telemetry` / `tools/vofa/README.md` |
| 发送 | JustFloat 库内置 | `BSP_VOFA_SendJustFloat` |
| Shell | 无 | `vofa on/off` |

两边都是 JustFloat，当前波特率相同，但 **通道数与通道含义不同**，VOFA 工程配置仍不能直接混用。

---

## 9. 常见问题

| 现象 | 可能原因 | 处理 |
|------|----------|------|
| VOFA 完全无数据 | 尚未执行首次初始化，或 TX DMA/接线异常 | 确认 `main.c` 从 UNINITIALIZED 启动，并检查 TX DMA 与串口接线 |
| 波形乱跳 / 拆帧失败 | 波特率错或混了文本 | 确认 921600；`Open_Printf_Debug=0` |
| 只有 TX 无 RX 调参 | 缺 USART1 中断 | 检查 `USART1_IRQHandler` 与 NVIC |
| 发送卡死 / 偶发丢帧 | DMA 未等完成又发 | `User_CorrespondSet` 等待 `gState==READY` |
| CubeMX 重生成后 RX 失效 | USART1 中断被关掉 | 在 CubeMX 勾选 USART1 global interrupt |
| 灯灭且无串口 | 其它模块 HardFault / Error_Handler | 先查 RGB DMA1_Ch3 中断是否被删 |
| 通道全 0 | FOC/编码器未接 | 正常；先确认 Ch0/Ch11 常数是否合理 |

### 上电自检清单

- [ ] USART1 921600，PB6/PB7 接线正确
- [ ] `User_CorrespondSet` 为 DMA 发送  
- [ ] `User_UserTX` 已填充 21 通道，`Printf_Loop` 覆盖 Ch6
- [ ] `Printf_Init` 已调用（校准流程内会调用）  
- [ ] 首次校准时 `Ch0=3`，完成后 `Ch0=4`；`CO=0?` 后 `Ch0=0`
- [ ] VOFA：JustFloat + **21** 通道 + 921600
- [ ] `Open_Printf_Debug == 0`  
- [ ] `User_PwmDuty_Set` 已写 TIM1 CCR（否则电机不转）  

---

## 10. 修订记录

| 日期 | 说明 |
|------|------|
| 2026-08-24 | 按源码复核为 921600；更新为 21 通道/88 字节，并修正 Ch6 为 DMA 成功启动间隔帧率 |
| 2026-08-02 | 18 通道；开环 AO/BO/CO 已对接；PWM/注入 ADC 补齐 |
| 2026-08-01 | 初版：汇总 JustFloat 对接、DMA 发送、VOFA 使用与排障 |

---

<p align="center"><b>VOFA+ × SguanFOC JustFloat — 先通链路，再看波形，后调参数</b></p>
