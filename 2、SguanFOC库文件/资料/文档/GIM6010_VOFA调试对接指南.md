# GIM6010 VOFA+ 调试对接指南

> **版本提示（2026-09-05）**：历史调试记录仍保留，但所有可执行操作步骤已按当前四模式接口更新。命令、单位、安全边界仍以工程根目录 `四模式上位机控制说明.md` 为最终依据。

> 工程：当前工作副本 `D:\Desktop\临时\2、SguanFOC库文件`  
> 协议：SguanFOC 内置 **JustFloat**（非 FireWater）  
> 适用：VOFA+ 波形观测、串口下行调参（AO/BO/CO/VO）  
> 修订：2026-09-05（921600 + 当前 24 通道 + 四模式安全启动）

---

## 目录

1. [概述](#1-概述)
2. [硬件与串口参数](#2-硬件与串口参数)
3. [软件数据流](#3-软件数据流)
4. [接口对接清单](#4-接口对接清单)
5. [24 通道映射](#5-当前-24-通道映射)
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
| 上行（板→PC） | **24** 路 float 波形 | `User_UserTX` → `Printf_Loop` → UART TX DMA |
| 下行（PC→板） | ASCII 命令改参数 | Idle-DMA 入队 → `APP_UartCommandPoll` → `Printf_Receive` → `User_AO/BO/CO/VO_Adjust` |

帧格式（JustFloat）：

```
[ float×24 ][ 00 00 80 7F ]
总长 = 24×4 + 4 = 100 字节
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
| 通道数 | **24**（`CH_COUNT`，Ch15～Ch23 为电流环/故障快照诊断） |

接线：USB 转串口 TX↔板 RX，RX↔板 TX，GND 共地。

> 当前源码 `Core/Src/usart.c` 与 CubeMX `.ioc` 均配置为 **921600**；VOFA+ 必须使用相同波特率。

---

## 3. 软件数据流

### 3.1 上行发送

```
main while(1)
  ├─ SguanFOC_main_Loop()              // 待命/故障态显式 CO=1? 后执行初始化/对齐
  │    └─ IDLE/运行/故障态填充 24 通道并请求发送
  └─ APP_JointPoll()
       └─ STANDBY/初始化态补充遥测
            └─ User_UserTX() → Printf_Loop()
                 └─ User_CorrespondSet()       // 500Hz 限速 + DMA 双缓冲；忙时丢本轮新帧
```

上电约 500ms 后先停在 `Ch0=0`；在 STANDBY（或故障条件已消失的锁存故障态）发送合法 `BO=<0..3>?` 和 `CO=1?`，才会看到初始化/校准状态 1～3，完成后为 4。若已处于 IDLE/运行态，`CO=1?` 会关桥、清目标和控制器并回到 IDLE，不会再次对齐。初始化/对齐进行中（Ch0=1～3）重复 `CO=1?` 会被忽略，不会关桥；任意时刻停机必须发送 `CO=0?`。正常状态下 `CO=0?` 回到 0，故障状态则继续保留故障码遥测。

### 3.2 下行接收

```
USART1 Idle 中断
  └─ USART1_IRQHandler → HAL_UART_IRQHandler
        └─ HAL_UARTEx_RxEventCallback()
              ├─ 复制完整 DMA 数据块到环形队列（溢出立即关 MOE 并锁停机）
              └─ 重新启动 ReceiveToIdle_DMA

main / APP_UartCommandPoll()
  └─ 检查停机序号并清断裂半帧
       └─ SguanFOC_Printf_Loop() → Printf_Receive()
            └─ 完整 '?' 帧严格解析 → User_AO/BO/CO/VO_Adjust()
```

必须具备：

- `USART1_IRQn` NVIC 使能（`usart.c` MspInit）
- `USART1_IRQHandler`（`stm32g4xx_it.c`）
- DMA RX 中断（`DMA1_Channel4_IRQHandler`）

---

## 4. 接口对接清单

按文件逐项对接，缺一则 VOFA 无波形或无法调参。

### 4.1 发送驱动 — `SguanFOC/UserData_Function.h`

当前实现的核心行为如下（缓冲区和索引定义见源码）：

```c
if ((ch == NULL) || (size == 0U) ||
    (size > sizeof(s_tx_buffer[0])) ||
    (huart1.gState != HAL_UART_STATE_READY)) {
    return;                         /* DMA 忙时丢弃本轮新遥测，不覆盖正在发送帧 */
}
if ((HAL_GetTick() - s_last_tx_tick) < USER_VOFA_TX_PERIOD_MS) {
    return;                         /* 500 Hz 限速 */
}
memcpy(s_tx_buffer[s_tx_index], ch, size);
if (HAL_UART_Transmit_DMA(&huart1, s_tx_buffer[s_tx_index], size) == HAL_OK) {
    s_last_tx_tick = HAL_GetTick();
    s_tx_index ^= 1U;
}
```

要点：

- 使用 **DMA 发送**，不得等待 UART READY 或与阻塞 `Transmit` 混用
- `User_CorrespondSet()` 先复制到专用双缓冲，DMA 不直接读取正在更新的 `Sguan.TXdata`；DMA 忙时丢弃本轮遥测
- 需 `#include "usart.h"`

### 4.2 通道填充 — `SguanFOC/UserData_UserControl.h`

在 `User_UserTX()` 中为 `Sguan.TXdata.fdata[0..23]` 赋值（见第 5 节）；Ch6 直接写入 20 kHz `High_Loop` 累计计数的低 16 位，不是发送间隔或帧率。

### 4.3 帧尾初始化 — `main.c` 或库初始化

```c
Printf_Init(&Sguan.TXdata);   // 写入 00 00 80 7F
```

`APP_JointInit()` 会在 STANDBY 遥测开始前调用一次；库走 `Sguan_Start_Tick()` 完整初始化时也会再次调用。

### 4.4 主循环 — `main.c`

```c
while (1) {
    SguanFOC_main_Loop();   // 初始化/对齐状态机及已初始化态 JustFloat
    APP_JointPoll();        // UART 主循环解析、待命态遥测、编码器/ADC/CAN
}
```

### 4.5 接收回调 — `APP/app_callbacks.c`

ISR 的正确职责是复制 DMA 数据块并重新挂接接收，不能在 ISR 内调用命令解析器。下面是控制流伪代码，完整的队列发布、内存屏障和重试逻辑以当前源码为准：

```c
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1) {
        if (rx_queue_has_room) {
            copy_dma_block_and_publish_to_queue(Size);
        } else {
            User_PwmOutputDisable();       /* 字节流断裂，失效安全停机 */
            latch_uart_stop_event();
        }
        if (APP_UartRxStart() == 0U) {
            User_PwmOutputDisable();       /* 停机通道丢失，保持关桥 */
            request_main_loop_rx_retry();
        }
    }
}
```

`main` 中上电启动一次 RX：

```c
if (APP_UartRxStart() == 0U) {
    Error_Handler();
}
```

主循环经 `APP_JointPoll()` 调用 `APP_UartCommandPoll()`：它先处理停机序号、清空断裂半帧，再把队列数据交给 `SguanFOC_Printf_Loop()` 严格解析。队列溢出、UART 错误或 DMA 重启失败都会先关 MOE；接收重启由主循环限速重试。

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
| `User_CO_Adjust(float)` | 仅精确 0 可任意时刻停机清零；精确 1 在 Ch0=1～3 时忽略，其他状态先关桥清目标，再按 STANDBY/故障或 IDLE/运行状态处理 |

### 4.8 调试开关 — `UserData_Config.h`

```c
#define Open_Printf_Debug 0   // 当前四模式交付固定为 0：JustFloat 24 通道
```

VOFA 看波形时务必保持 **0**。

---

## 5. 当前 24 通道映射

`CH_COUNT = 24`，定义以 `UserData_UserControl.h::User_UserTX()` 为准。

| Ch | 含义 | 单位 |
|:--:|------|------|
| 0 | 电机状态机编号 | — |
| 1 / 2 | 输出轴实测 / 目标转速 | rpm |
| 3 / 4 | Id / Iq | A |
| 5 | U 相逻辑 PWM 比较值 | count |
| 6 | 20 kHz High_Loop 计数低 16 位 | count |
| 7 | Uq 给定 | V |
| 8 | A 相实际电流 | A |
| 9 | PLL 连续机械位置（电机轴） | rad |
| 10 | 上电对齐后的机械零位偏置 | rad |
| 11 | 模式：0开环、1电流、2速度、3位置 | — |
| 12 | 母线电压 | V |
| 13 / 14 | 实际电角度 / MT6816 单圈机械角 | rad |
| 15 / 16 | IV / IW 原始 ADC；故障时为首次冻结快照 | count |
| 17 / 18 | IV / IW 电流偏置；故障时为首次冻结快照 | count |
| 19 / 20 | 两路采样换算电流；故障时为首次冻结快照 | A |
| 21 / 22 | d / q 电流控制器输出；故障时为首次冻结快照 | V |
| 23 | Iq 目标；故障时为首次冻结快照 | A |

Ch6 是控制高环累计计数的低 16 位，不是帧率。500 Hz 遥测正常时，相邻帧的 Ch6 增量应约为 40；计数跨 65535 时会回绕。

24 通道数据帧共 100 byte。921600-8N1 每字节按 10 bit 计，500 Hz 约占 54% 物理带宽；DMA 忙时会丢弃本轮新遥测，不覆盖正在发送帧，也不阻塞控制环。

### 推荐观察（开环）

| 目的 | 通道 |
|------|------|
| 指令是否进板 | Ch7 `Uq_in`（开环发 `AO=0.2?` 后约为 0.2） |
| 高频控制是否持续 | 相邻遥测帧的 Ch6 增量通常约为 40 |
| 模式/状态 | 发送运动目标前 Ch0=4；开环 `AO` 生效后 Ch0≈7，Ch11=0 |
| 是否在转 | Ch14 单圈机械角变化、Ch1 输出轴 rpm |
| 母线 | Ch12≈24 |

---

## 6. VOFA+ 使用步骤

1. 安装并打开 **VOFA+**
2. 选择对应 COM 口
3. 波特率设为 **921600**，8N1
4. 协议插件选择 **JustFloat**
5. 通道数设为 **24**
6. 打开串口，添加示波器，优先勾选 Ch0/Ch1/Ch3/Ch4/Ch7/Ch9/Ch11～Ch23
7. 用限流电源上电；等待约 500 ms 后确认 Ch0=0（STANDBY），此时功率桥应保持关闭
8. 发送 `BO=0?`、`CO=1?`，等待对齐完成且 Ch0=4，再发送开环目标

### 建议控件布局

- 示波器 1：Ch3/Ch4 Id/Iq、Ch7 Uq  
- 示波器 2：Ch9 连续机械位置、Ch13 电角度、Ch14 单圈机械角  
- 示波器 3：Ch0 status、Ch11 mode、Ch12 VBUS；故障诊断另看 Ch15～Ch23  

---

## 7. 下行调参命令与开环运行

在 VOFA **发送窗口**发 ASCII（必须以 `?` 结尾）：

### 开环转起来（推荐顺序）

```text
BO=0?
CO=1?
（等待 Ch0=4）
VO=1?
AO=0.2?
（观察完成）
CO=0?
```

| 步骤 | 命令 | 预期 VOFA 回显 |
|------|------|----------------|
| 确认开环 | `BO=0?` | Ch11 → 0 |
| 显式初始化 | `CO=1?` | Ch0 依次进入初始化/对齐，最终为 4；此时 MOE 仍关闭 |
| 给开环速度 | `VO=1?` | 设置输出轴目标速度 1 rad/s，不单独打开功率桥 |
| 给小 Uq | `AO=0.2?` | Ch7→约0.2；开环的非零 AO 才允许开桥并缓慢转动 |
| 停机 | `CO=0?` | 立即关 MOE、清目标并回 STANDBY；`AO=0?` 只清当前运动目标 |
| 使能/安全重置 | `CO=1?` | Ch0=1～3 时忽略；其他状态先关桥清旧目标，STANDBY/允许清除的故障态重新初始化，IDLE/运行态只回 IDLE；随后须重新发送目标 |

> 开环 `|Uq|` 软件限幅为 `0.35×VBUS`。该上限不是安全起步值；首次限流台架从 0.2 V 开始，仅在波形和方向稳定后小步增加，并同时确认电流与故障关断。

### 命令表

| 前缀 | 回调 | 含义 |
|------|------|------|
| `AO=` | `User_AO_Adjust` | mode0→Uq[V]；mode1→Iq[A]；mode2→输出轴速度[rad/s]；mode3→电机轴连续位置[rad] |
| `VO=` | `User_VO_Adjust` | 仅 mode0：输出轴开环速度[rad/s] |
| `BO=` | `User_BO_Adjust` | 只接受精确 `BO=0?`、`BO=1?`、`BO=2?`、`BO=3?` |
| `CO=` | `User_CO_Adjust` | 只接受精确 `CO=0?` 停机或 `CO=1?` 状态相关使能/重置；Ch0=1～3 重复使能被忽略，仅 `CO=0?` 可任意时刻停机，其他形式拒绝 |

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
| VOFA 完全无数据 | TX DMA/串口配置或接线异常 | STANDBY 也应有遥测；检查 TX DMA、921600-8N1 与串口接线 |
| 波形乱跳 / 拆帧失败 | 波特率错或混了文本 | 确认 921600；`Open_Printf_Debug=0` |
| 只有 TX 无 RX 调参 | 缺 USART1 中断 | 检查 `USART1_IRQHandler` 与 NVIC |
| 偶发少帧 | TX DMA 忙时主动丢弃本轮新遥测 | 这是非阻塞设计，不会覆盖正在发送帧；若持续丢帧则检查串口带宽和主循环阻塞 |
| CubeMX 重生成后 RX 失效 | USART1 中断被关掉 | 在 CubeMX 勾选 USART1 global interrupt |
| 灯灭且无串口 | 其它模块 HardFault / Error_Handler | 先查 RGB 使用的 `DMA1_Channel1_IRQHandler` 是否被删 |
| 通道全 0 | FOC/编码器未接 | 正常；先确认 Ch0/Ch11 常数是否合理 |

### 上电自检清单

- [ ] USART1 921600，PB6/PB7 接线正确
- [ ] `User_CorrespondSet` 为 DMA 发送  
- [ ] `User_UserTX` 已填充 24 通道；Ch6 为 High_Loop 计数低 16 位
- [ ] `Printf_Init` 已在 `APP_JointInit` 调用，STANDBY 也持续遥测  
- [ ] 上电约 500 ms 后 `Ch0=0`；发送 `BO=<模式>?`、`CO=1?` 后才校准并进入 `Ch0=4`
- [ ] `CO=0?` 后 `Ch0=0` 且 MOE 关闭；故障码不会被 STOP 覆盖
- [ ] VOFA：JustFloat + **24** 通道 + 921600
- [ ] `Open_Printf_Debug == 0`  
- [ ] `User_PwmDuty_Set` 已写 TIM1 CCR（否则电机不转）  

---

## 10. 修订记录

| 日期 | 说明 |
|------|------|
| 2026-09-05 | 同步当前 24 通道、严格 BO/CO、独立 VO、UART 失效安全队列及条件化初始化/停机流程 |
| 2026-08-24 | 按源码复核为 921600；更新为 21 通道/88 字节，并修正 Ch6 为 DMA 成功启动间隔帧率 |
| 2026-08-02 | 18 通道；开环 AO/BO/CO 已对接；PWM/注入 ADC 补齐 |
| 2026-08-01 | 初版：汇总 JustFloat 对接、DMA 发送、VOFA 使用与排障 |

---

<p align="center"><b>VOFA+ × SguanFOC JustFloat — 先通链路，再看波形，后调参数</b></p>
