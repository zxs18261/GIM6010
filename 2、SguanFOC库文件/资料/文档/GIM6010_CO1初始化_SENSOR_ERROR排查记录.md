# GIM6010 使能初始化即报 SENSOR_ERROR(20) 排查记录

> 工程：`E:\github\GIM6010\2、SguanFOC库文件`
> 现象场景：发送 `CO=1?` 显式使能初始化 → **无黄闪、立即红灯**，VOFA Ch0(状态)=20
> 文档版本：v0.1（2026-XX，进行中，待 Ch15 阶段码确认后定稿）
> 性质：现场故障排查 + 静态定位记录；尚未给出最终修复，待硬件确认

---

## 1. 目录

1. [问题现象](#1-问题现象)
2. [VOFA 帧解读](#2-vofa-帧解读)
3. [故障码 20 是什么](#3-故障码-20-是什么)
4. [静态定位](#4-静态定位)
5. [最可疑根因](#5-最可疑根因)
6. [诊断改动（临时）](#6-诊断改动临时)
7. [待确认与下一步](#7-待确认与下一步)
8. [建议修复方向](#8-建议修复方向)
9. [修订记录](#9-修订记录)

---

## 1. 问题现象

在 STANDBY 待命（Ch0=0、绿慢闪、MOE 关闭）下发 `BO=0?` 后发 `CO=1?`：

- 期望：进入初始化/对齐（Ch0 1→2→3，黄灯快闪），约 2s 后 Ch0=4（IDLE）。
- 实际：**根本没有黄闪，立即红灯快闪**，状态锁定。

| 项 | 值 |
|----|----|
| LED | 红快闪（APP 故障态） |
| Ch0（状态） | **20** |
| Ch6（20kHz 高环计数） | **0（恒定）** |
| Ch12（母线电压） | 23.5 V（正常） |
| Ch14（MT6816 机械角） | ≈1.69 rad（编码器在读） |
| Ch15~23（电流环诊断） | 全 0 |

---

## 2. VOFA 帧解读

JustFloat 24 通道，little-endian float。对原始 hex 解码后关键通道（典型帧）：

```
[07:32:27.035] 00 00 A0 41 ...   → 0x41A00000 = 20.0  (Ch0 状态)
...
                CD 05 BC 41 ...   → ≈23.50 V          (Ch12 VBUS)
                BB 21 D8 3F ...   → ≈1.6885 rad       (Ch14 机械角)
```

- Ch0 = **20**，非运行态 5~13，非过压/欠压/过温/过流/编码器错误（分别 14/15/16/18/19）。
- **Ch6=0**：20 kHz 注入计数从未增长 → ADC 注入链没有布防成功。
- Ch15~23 全 0：故障不是由 20 kHz `High_Loop` 冻结路径置的（那条会填快照）。

---

## 3. 故障码 20 是什么

按 `SguanFOC v3.0.1/Sguan_MotorStatus.h`：

```
0x14 = MOTOR_STATUS_SENSOR_ERROR   (传感器故障, 锁定, 手动解除)
```

触发 SENSOR_ERROR(20) 的可能代码路径：
1. `SguanFOC.c Sguan_Start_Tick()` L1657：`User_InitialInit()` 返回 0 → 置 SENSOR_ERROR
2. `SguanFOC.c Sguan_PreflightFaultStatus()`：VBUS/温度 NaN/Inf → SENSOR_ERROR
3. `app_callbacks.c`：ADC 错误/注入队列溢出/5ms 高环心跳超时 → SENSOR_ERROR
4. `Sguan_PreflightFaultStatus` 中 `MOTOR_STATUS_SENSOR_ERROR_Signal()` 置位

**Ch6=0 且无黄闪 → 高度指向路径 1**（初始化在对齐带电前就失败，注入链还没布防）。

---

## 4. 静态定位

### 4.1 排除项

| 现象 | 排除/指向 |
|----|----|
| Ch0=20 | 传感器/ADC 链故障（非过流18/编码器19/电压14~15） |
| Ch6=0 | ADC 注入链从未布防，20 kHz 未跑 |
| Ch15~23 全0 | 非 High_Loop 冻结路径置位 |
| 无黄闪 | 未进入 CALIBRATING 带电对齐，初始化立即失败 |

**四者叠加唯一自洽**：`Sguan_Start_Tick()` 调用 `User_InitialInit()` 返回 0 → 直接锁 `SENSOR_ERROR(20)`。

### 4.2 `User_InitialInit()`（UserData_Function.h）每条 return 0

| 行 | 判定 | 失败含义 |
|----|------|----------|
| L31 | `HAL_ADCEx_InjectedStop_IT != HAL_OK` | 停止注入失败 |
| L34 | `HAL_ADC_Stop != HAL_OK` | HAL 停 ADC 失败 |
| L38-39 | 规则/注入转换仍在跑 | 转换未停 |
| L43 | **`LL_ADC_IsEnabled()!=0`** | **★ADC 仍使能** |
| L47 | `HAL_ADCEx_Calibration_Start != HAL_OK` | 校准失败 |
| L73 | `HAL_ADCEx_InjectedStart_IT != HAL_OK` | 注入布防失败 |

---

## 5. 最可疑根因

**头号嫌疑：`User_InitialInit` L43 —— ADC 在校准前必须 disabled，但待命态已把 ADC enable。**

发现**代码注释与实现矛盾**：

- `BSP_ADC.c` `BSP_ADC_ReadChannelLL()`（L97-109）：待命态慢速采样若发现 ADC 未使能，会用 `LL_ADC_Enable` 使能 ADC，做单通道读后**从不 disable**（注释：为不杀 20 kHz 注入链而不碰 ADDIS）。
- `User_InitialInit` 却**假设进入时 ADC 已 disabled**（L43 `LL_ADC_IsEnabled()!=0` 即 return 0）。
- `BSP_ADC.c` 头注释（L100）声称 *"User_InitialInit 已在校准前先 LL_ADC_Disable"*，但**全工程 grep 无任何应用侧 `LL_ADC_Disable` 调用**——注释描述的动作不存在。

**时序脆弱性**：上电待命停留 >10ms（正常几乎必然）后 `BSP_ADC_Poll` 已把 ADC1 enable；随后 `CO=1?` 进入 `User_InitialInit` 时发现 ADC 仍 enable → return 0 → SENSOR_ERROR。之前偶尔能过属时序巧合。

---

## 6. 诊断改动（临时）

为坐实根因，已加入临时诊断量 `g_foc_init_stage`，通过 **VOFA Ch15** 回传失败阶段码。

### 改动文件
| 文件 | 改动 |
|----|----|
| `APP/app_callbacks.c` | 定义 `volatile uint8_t g_foc_init_stage = 0U` |
| `APP/app_callbacks.h` | `extern volatile uint8_t g_foc_init_stage;` |
| `SguanFOC v3.0.1/UserData_Function.h` | `User_InitialInit` 各 return 0 前写入阶段码 |
| `SguanFOC v3.0.1/UserData_UserControl.h` | Ch15 阶段码非零时改显阶段码 |

### 阶段码对照（VOFA Ch15）
| Ch15 | 含义 |
|----|----|
| 1 | `HAL_ADCEx_InjectedStop_IT` 失败 |
| 2 | `HAL_ADC_Stop` 失败 |
| 3 | 规则/注入转换仍在跑 |
| **4** | **★ADC 仍使能（待命态 BSP_ADC_Poll 遗留）** |
| 5 | 校准启动失败 |
| 6 | 注入布防失败 |
| 0 | `User_InitialInit` 通过（若仍红转查 preflight） |

> ⚠️ 这些均为临时诊断代码，根因确认后**必须还原**，勿留污染到正式固件。

---

## 7. 待确认与下一步

### 7.1 验证方法
1. Keil 重新编译烧录（改的都是既有文件，无需动 uvprojx）。
2. 上电等绿慢闪（STANDBY，Ch0=0），停留 >10ms 让 `BSP_ADC_Poll` 使能 ADC。
3. 发 `CO=1?`。
4. 看 **Ch15**：
   - **=4** → 实锤待命态 ADC 遗留使能，按第 8 节修复。
   - =其他 → 按对照表定位对应步。
   - =0 仍红 → 转查 `Sguan_PreflightFaultStatus`（VBUS/温度 NaN 等）。

> ⚠️ **现场反馈待更新**：需采集 Ch15 阶段码后回填本节结论。

---

## 8. 建议修复方向

在 `User_InitialInit` 的 disabled 检查**之前**，显式关闭待命态 LL 路径使能的 ADC：

```c
static inline uint8_t User_InitialInit(void){
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
    CLEAR_BIT(htim1.Instance->CR1, TIM_CR1_CEN);

    if (HAL_ADCEx_InjectedStop_IT(&hadc1) != HAL_OK) return 0U;

    /* ★ 待命态 BSP_ADC_Poll 的 LL 单通道读把 ADC enable 后从不 disable，
     *   这里校准前必须显式关掉，否则下方 LL_ADC_IsEnabled()!=0 直接 return 0。 */
    if (LL_ADC_IsEnabled(hadc1.Instance) != 0UL) {
        LL_ADC_Disable(hadc1.Instance);
        // 等待 ADDIS 完成 / ADEN 清零
    }

    if (HAL_ADC_Stop(&hadc1) != HAL_OK) return 0U;
    ...  // 原 L38-43 断言保留
}
```

要点：先 `InjectedStop_IT` 停注入，再 `LL_ADC_Disable`，顺序不可颠倒。改动前先以 Ch15=4 确认。

---

## 9. 修订记录

| 日期 | 内容 |
|------|------|
| 2026-XX | 建立记录；现象、帧解码、静态定位、诊断改动；待 Ch15 确认 |
