# GIM6010 SguanFOC AI 辅助移植操作指南

> **历史参考提示（2026-08-25）**：本文保留早期移植思路和分阶段方法，部分控制频率、状态边界、VOFA通道及启动流程已不再代表当前源码。新AI或新工程师必须先阅读[`docs/foc/ai-development-handoff.md`](../../docs/foc/ai-development-handoff.md)，并以当前源码、`.ioc`、项目档案和VOFA契约为准。

> 本文档说明如何借助 AI（如 Cursor Agent）快速完成 GIM6010 工程中 SguanFOC 电机控制库的移植，并在实物上实现稳定运行。  
> 工程路径：`E:\github\GIM6010\SguanFOC库文件`  
> 文档路径：`E:\github\GIM6010\SguanFOC库文件\资料\文档`

---

## 目录

1. [工程现状](#1-工程现状)
2. [参考资源路径](#2-参考资源路径)
3. [AI 能做什么 / 不能做什么](#3-ai-能做什么--不能做什么)
4. [整体移植流程概览](#4-整体移植流程概览)
5. [阶段 0：准备上下文](#5-阶段-0准备上下文)
6. [阶段 1：硬件接口与主循环挂接](#6-阶段-1硬件接口与主循环挂接)
7. [阶段 2：电流采样与 PWM 对齐验证](#7-阶段-2电流采样与-pwm-对齐验证)
8. [阶段 3：有感闭环运行](#8-阶段-3有感闭环运行)
9. [阶段 4：保护与通信](#9-阶段-4保护与通信)
10. [阶段 5：调参与实物优化](#10-阶段-5调参与实物优化)
11. [需修改的文件清单](#11-需修改的文件清单)
12. [上电前安全检查](#12-上电前安全检查)
13. [AI 协作技巧与 Prompt 模板](#13-ai-协作技巧与-prompt-模板)
14. [常见问题排查](#14-常见问题排查)
15. [文档维护说明](#15-文档维护说明)

---

## 1. 工程现状

| 项目 | 说明 |
|------|------|
| MCU | STM32G431 |
| 工程名 | GIM6010_V0.0 |
| 生成方式 | STM32CubeMX + Keil MDK-ARM |
| 电机库版本 | SguanFOC v3.0.1（有感 FOC，Q31 定点，STA 速度环） |
| 当前状态 | CubeMX 外设已配置，SguanFOC 库已加入，**尚未完成代码对接** |

### 已配置外设

| 外设 | 用途 |
|------|------|
| TIM1 | 三相 PWM 输出，CC4 触发 ADC 注入采样 |
| ADC1 / ADC2 | 双电阻电流采样（注入通道，TIM1 CC4 触发） |
| SPI1 / SPI3 | 编码器或传感器通信 |
| FDCAN1 | 总线通信 |
| USART1 + DMA | 串口调试 / JustFloat 上位机 |
| TIM4 | 低频控制任务（状态机、保护） |

### 尚未完成的工作

- `main.c` 未调用 `SguanFOC_main_Loop()`
- `stm32g4xx_it.c` 未挂接 SguanFOC 高中低频中断任务
- `UserData_Function.h` 硬件接口仍为注释模板
- 电机参数与控制模式待根据 GIM6010 实物确认

---

## 2. 参考资源路径

| 资源 | 路径 |
|------|------|
| 本工程根目录 | `E:\github\GIM6010\SguanFOC库文件` |
| 工程文档目录 | `E:\github\GIM6010\SguanFOC库文件\资料\文档` |
| 硬件资料目录 | `E:\github\GIM6010\SguanFOC库文件\资料\硬件资料` |
| SguanFOC 官方库 | `E:\github\SguanFOC\SguanFOC_Library-main` |
| STM32G4 官方例程 | `E:\github\SguanFOC\SguanFOC_Library-main\配套最新例程开源③[STM32G4，下桥臂双电阻]` |
| 使用说明 PDF | `E:\github\SguanFOC\SguanFOC_Library-main\配套QT上位机及FOC使用说明①[PDF]` |
| 移植流程截图 | `E:\github\GIM6010\SguanFOC库文件\资料\移植流程Snipaste_2026-07-28_22-15-29.png` |

---

## 3. AI 能做什么 / 不能做什么

### AI 擅长

- 对照官方例程编写 `UserData_Function.h` 硬件接口
- 修改 `main.c`、中断服务函数、Keil 工程引用
- 解释编译报错，对比 `.ioc` 与例程差异
- 给出 PID / STA 调参建议和 Debug 通道配置

### 仍需人工完成

- 确认引脚、ADC 通道、编码器型号与接线
- 编译、烧录、示波器/串口/上位机观测
- 第一次上电时的现场安全监护
- 电流环、速度环的最终实物调参

> **原则：AI 写代码，人做验证。分阶段推进，每步可测。**

---

## 4. 整体移植流程概览

```
阶段 0  准备上下文（路径、硬件信息、目标）
   ↓
阶段 1  硬件接口 + main/中断挂接 → 编译通过
   ↓
阶段 2  电流采样与 PWM 对齐 → 开环慢转
   ↓
阶段 3  有感闭环（速度环）→ 空载恒速
   ↓
阶段 4  保护与通信（FDCAN 等）
   ↓
阶段 5  调参优化 → 实物稳定运行
```

**最短路径建议：**

1. 对照 STM32G4 官方例程，完成 `UserData_Function.h` + `main.c` + 中断挂接
2. 核对 TIM1 / ADC1 注入采样与 `TIM_T = 1e-4` 是否匹配
3. 实现 GIM6010 编码器读角度，接入 `User_Encoder_ReadRad`
4. 配置 `VelCur_DOUBLE_MODE` + GIM6010 电机参数，空载低速测试
5. 接 JustFloat 上位机，观察 Id / Iq / 速度 / 角度

---

## 5. 阶段 0：准备上下文

在与 AI 开始移植前，一次性提供以下信息：

### 5.1 固定信息（可直接复制）

```
工程路径：E:\github\GIM6010\SguanFOC库文件
参考例程：E:\github\SguanFOC\SguanFOC_Library-main\配套最新例程开源③[STM32G4，下桥臂双电阻]
文档路径：E:\github\GIM6010\SguanFOC库文件\资料\文档
SguanFOC 版本：v3.0.1（有感 FOC，Q31 定点）
目标：GIM6010 实物有感 FOC 速度环稳定运行
约束：不修改 CubeMX 生成的 tim.c / adc.c / spi.c 等（仅改 USER CODE 区）
```

### 5.2 需补充的硬件信息

| 信息项 | 示例 | 备注 |
|--------|------|------|
| 编码器类型 | SPI 磁编 / ABI 增量 / 其他 | 决定 `User_Encoder_ReadRad` 实现 |
| 编码器接口 | SPI1 / SPI3 / TIM 编码器模式 | 对应 CubeMX 配置 |
| 驱动使能引脚 | SD / EN 引脚名 | 决定 `User_InitialInit` 内容 |
| 电流采样方式 | 下桥臂双电阻 | 与 `Current_Num` 对应 |
| 母线电压 | 12V / 24V | 填入 `Sguan.motor.VBUS` |
| 电机极对数 | 7 | 填入 `Sguan.motor.Poles` |

### 5.3 启动 Prompt 模板

```
请按官方例程「配套最新例程开源③[STM32G4，下桥臂双电阻]」，
为 GIM6010 工程完成 SguanFOC 移植。

要求：
1. 只改 UserData_*.h、main.c、stm32g4xx_it.c 的 USER CODE 区
2. 第一目标：开环能转
3. 编码器是 [填写型号/接口]
4. 参考文档在 资料\文档 文件夹
5. 不要修改 CubeMX 自动生成的外设初始化文件
```

---

## 6. 阶段 1：硬件接口与主循环挂接

### 6.1 目标

- 编译通过
- 烧录成功
- 串口有输出
- 开环模式下电机能慢速转动

### 6.2 需修改文件

#### `SguanFOC/UserData_Function.h`

实现以下 inline 函数：

| 函数 | 功能 |
|------|------|
| `User_InitialInit()` | 使能驱动、启动 PWM、启动编码器 |
| `User_Delay(ms)` | 毫秒延时，通常 `HAL_Delay(ms)` |
| `User_ReadADC_Raw(ch)` | 读取注入 ADC 原始值（两相电流） |
| `User_Encoder_ReadRad()` | 读取编码器弧度（0 ~ 2π） |
| `User_PwmDuty_Set(u,v,w)` | 设置 TIM1 三通道比较值 |
| `User_VBUS_DataGet()` | 读取母线电压（暂不用可返回 -9999.0f） |
| `User_Temperature_DataGet()` | 读取温度（暂不用可返回 -9999.0f） |

#### `Core/Src/main.c`

```c
#include "SguanFOC.h"

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();
    // ... 其他 MX_*_Init ...

    while (1) {
        SguanFOC_main_Loop();   // 初始化 + 串口数据收发
    }
}
```

#### `Core/Src/stm32g4xx_it.c`

```c
#include "SguanFOC.h"

// TIM1 更新/比较中断 → 高频任务（电流环、PWM）
void TIM1_UP_TIM16_IRQHandler(void) {
    // HAL 中断处理 ...
    SguanFOC_High_Loop();
}

// TIM4 中断 → 低频任务（状态机、保护）
void TIM4_IRQHandler(void) {
    // HAL 中断处理 ...
    SguanFOC_Low_Loop();
}

// USART1 中断 → 串口调试
void USART1_IRQHandler(void) {
    // HAL 中断处理 ...
    SguanFOC_Printf_Loop(Sguan_PrintfBuff, Size);
}
```

> 具体中断函数名以 CubeMX 生成为准，对照官方例程确认。

### 6.3 验证清单

- [ ] Keil 编译 0 Error
- [ ] 烧录后 MCU 正常运行（无 HardFault）
- [ ] 串口有 JustFloat / printf 数据输出
- [ ] 驱动使能脚电平正确
- [ ] 开环模式下电机能慢速转动（低占空比、限流）

---

## 7. 阶段 2：电流采样与 PWM 对齐验证

### 7.1 目标

- ADC 采样时刻与 PWM 周期对齐
- Id / Iq 数值合理
- 手动转轴时电流方向正确

### 7.2 关键核对项

| 核对项 | 说明 | 对应配置 |
|--------|------|----------|
| ADC 触发源 | TIM1 CC4 上升沿触发注入转换 | `.ioc` → ADC1 Injected Trigger |
| 控制周期 | `TIM_T = 1e-4`（10 kHz） | `UserData_Config.h` |
| PWM 频率 | 与 TIM1 配置一致 | `tim.c` / `.ioc` |
| 满占空比 | `Sguan.motor.Duty` 与 TIM1 ARR 一致 | `UserData_Motor.h`（当前默认 4249） |
| 电流通道 | `Current_Num`：0=AB，1=AC，2=BC | `UserData_Motor.h` |
| 电流方向 | `Current_Dir0/1`：±1 | 根据硬件接线调整 |
| 采样电阻 | `Sampling_Rs` | 默认 0.005 Ω |
| 运放增益 | `Amplifier` | 默认 10.0 |

### 7.3 验证清单

- [ ] 静止时 Id / Iq 接近 0
- [ ] 手动转轴时电流有合理响应
- [ ] 电流方向与电机转向一致
- [ ] 无周期性电流尖峰（采样时刻不对时会明显）

---

## 8. 阶段 3：有感闭环运行

### 8.1 目标

- 速度-电流双闭环稳定运行
- 空载恒速，加负载不失步

### 8.2 配置步骤

#### 8.2.1 选择控制模式（`UserData_Motor.h`）

```c
Sguan.mode = VelCur_DOUBLE_MODE;   // 速度-电流串级闭环
```

可选模式（v3.0.1）：

| 模式宏 | 说明 |
|--------|------|
| `Velocity_OPEN_MODE` | 速度开环（测试用） |
| `Current_SINGLE_MODE` | 电流单闭环（力矩控制） |
| `VelCur_DOUBLE_MODE` | 速度-电流双闭环 |
| `PosVelCur_THREE_MODE` | 位置-速度-电流三闭环 |

#### 8.2.2 填写电机参数（`UserData_Motor.h`）

```c
Sguan.identify.Ld   = ?;      // D 轴电感 (H)
Sguan.identify.Lq   = ?;      // Q 轴电感 (H)
Sguan.identify.Rs   = ?;      // 相电阻 (Ω)
Sguan.identify.Flux = ?;      // 磁链 (Wb)
Sguan.motor.Poles   = ?;      // 极对数
Sguan.motor.VBUS    = ?;      // 母线电压 (V)
```

#### 8.2.3 编码器配置

```c
Sguan.motor.Encoder_Dir = 1;   // 或 -1，根据实物方向调整
```

确保 `User_Encoder_ReadRad()` 返回 0 ~ 2π 的弧度值。

#### 8.2.4 控制器参数（`UserData_Parameter.h`）

默认参数可作为起点，实物测试后再调：

- 电流环 Kp / Ki
- 速度环 STA 参数（`k1`, `k2`, `boundary`）或 PI 参数
- 滤波器截止频率

### 8.3 验证清单

- [ ] 空载 500 rpm 恒速运行
- [ ] 正反转方向正确
- [ ] 加载后速度恢复
- [ ] 无异常振动和异响
- [ ] 电流不超过 `Sguan.safe` 限制

---

## 9. 阶段 4：保护与通信

### 9.1 保护功能（`UserData_Motor.h` → `Sguan.safe`）

| 参数 | 说明 | 建议初值 |
|------|------|----------|
| `VBUS_MAX / VBUS_MIN` | 母线电压上下限 | 按实际电源设置 |
| `DQcur_MAX` | D/Q 轴电流上限 | 先设 0.5A 测试 |
| `Temp_MAX` | 最高温度 | 60°C |
| `Current_limit` | 状态机电流阈值 | 0.5A |
| `Speed_limit` | 状态机速度阈值 | 5.0 rad/s |

### 9.2 FDCAN 通信（可选，单独阶段）

- 第一版建议只用 USART 调试
- FDCAN 协议对接稳定后再集成
- 避免与 FOC 首启调试混在一起

### 9.3 验证清单

- [ ] 过流时驱动自动关闭
- [ ] 过压 / 欠压保护生效
- [ ] 故障后可重新使能

---

## 10. 阶段 5：调参与实物优化

### 10.1 上位机调试

1. 安装 SguanFOC 配套 QT 上位机（见官方 PDF 说明）
2. 串口连接 USART1，波特率与 CubeMX 配置一致
3. 选择 JustFloat 通道，推荐观察：

| 通道 | 内容 |
|------|------|
| Id / Iq | 电流环响应 |
| 速度 | 速度环跟踪 |
| 角度 | 编码器 / 电角度 |
| Vd / Vq | 电压输出 |
| 母线电压 | 电源状态 |

### 10.2 调参顺序

```
1. 电流环 Kp/Ki → Id/Iq 跟随良好、无振荡
2. 速度环 k1/k2（STA）或 Kp/Ki（PI）→ 速度平稳
3. 滤波器截止频率 → 噪声与响应平衡
4. 加负载 → 微调速度环
5. 提高转速 → 检查高速稳定性
```

### 10.3 让 AI 协助调参

提供具体现象，例如：

```
现象：速度环 1000rpm 时有 50Hz 振动
当前参数：Current_Q.Kp=0.3425, Speed.k1=0.36, Speed.k2=52
请分析可能原因并建议调整方向。
```

---

## 11. 需修改的文件清单

| 文件 | 阶段 | 修改内容 |
|------|------|----------|
| `SguanFOC/UserData_Function.h` | 1 | 硬件接口实现 |
| `SguanFOC/UserData_Motor.h` | 1~3 | 电机参数、控制模式、安全阈值 |
| `SguanFOC/UserData_Parameter.h` | 3~5 | PID/STA/滤波器参数 |
| `SguanFOC/UserData_Config.h` | 1 | 控制周期、Q31 开关等 |
| `SguanFOC/UserData_Status.h` | 3 | 状态机回调（可选） |
| `SguanFOC/UserData_UserControl.h` | 3 | 指令处理（可选） |
| `Core/Src/main.c` | 1 | 引入 SguanFOC，主循环 |
| `Core/Src/stm32g4xx_it.c` | 1 | 中断挂接 |
| `Core/Inc/main.h` | 1 | 头文件引用（如需要） |

> **不要修改**：CubeMX 生成的 `tim.c`、`adc.c`、`spi.c`、`gpio.c` 等（除非重新生成）。

---

## 12. 上电前安全检查

每次上电前逐项确认：

- [ ] 电源限流或使用限流电源
- [ ] `Sguan.safe.DQcur_MAX` 已设为小值（如 0.5A）
- [ ] 驱动使能引脚逻辑正确
- [ ] 电机三相接线无误
- [ ] 编码器接线与 SPI/接口配置一致
- [ ] 先开环低占空比测试，再切闭环
- [ ] 台架固定，避免失控旋转
- [ ] 示波器或上位机就绪，可随时观测电流

---

## 13. AI 协作技巧与 Prompt 模板

### 13.1 六条技巧

| # | 技巧 | 说明 |
|---|------|------|
| 1 | 一次只改一个模块 | 避免 AI 一次改太多文件导致难排查 |
| 2 | 明确约束 | 如「不改 CubeMX 生成文件」「只用 USART」 |
| 3 | 报错给完整信息 | 编译错误全文、电机现象、串口数据 |
| 4 | 让 AI 做 diff 对照 | 对比本工程与官方例程差异 |
| 5 | 用 @ 引用文件 | Cursor 中 `@SguanFOC/UserData_Function.h` 直接定位 |
| 6 | 分阶段验收 | 每阶段编译+烧录+观测后再继续 |

### 13.2 常用 Prompt 模板

**硬件接口移植：**
```
对照官方例程 Hardware/Timer.c，为 GIM6010 实现 UserData_Function.h 中的
User_InitialInit、User_ReadADC_Raw、User_PwmDuty_Set、User_Encoder_ReadRad。
编码器使用 [SPI1/型号]，电流采样为下桥臂双电阻。
```

**中断挂接：**
```
参照官方例程 main.c 和 stm32g4xx_it.c，为 GIM6010 工程挂接
SguanFOC_main_Loop、SguanFOC_High_Loop、SguanFOC_Low_Loop、SguanFOC_Printf_Loop。
只修改 USER CODE 区域。
```

**编译报错：**
```
GIM6010 工程编译报错如下：
[粘贴完整错误信息]
请分析原因并修复，不要改动 CubeMX 生成的外设文件。
```

**运行异常：**
```
GIM6010 开环测试现象：[描述现象，如反转、抖动、不转]
当前配置：Current_Num=1, PWM_Dir=-1, Motor_Dir=1, Duty=4249
串口数据：Id=xx, Iq=xx
请分析可能原因。
```

**调参协助：**
```
GIM6010 速度环空载 800rpm 时有轻微振荡。
当前 STA 参数：k1=0.36, k2=52, boundary=12.0
电流环：Kp=0.3425, Ki=1257.27
请建议调整方向。
```

---

## 14. 常见问题排查

| 现象 | 可能原因 | 排查方向 |
|------|----------|----------|
| 编译报错 undefined reference | Keil 工程未添加 SguanFOC 源文件 | 检查 MDK 工程文件列表 |
| 烧录后 HardFault | 中断未正确初始化、栈溢出 | 检查中断挂接顺序 |
| 电机不转 | 驱动未使能、PWM 未启动 | 检查 `User_InitialInit` |
| 电机反转 | 方向参数错误 | 调整 `Motor_Dir` / `PWM_Dir` / `Encoder_Dir` |
| 电流很大 | 采样方向反、电角度错 | 检查 `Current_Dir`、编码器读数 |
| 闭环振荡 | Kp 过大、采样时刻不对 | 降 Kp、查 ADC 触发 |
| 串口无数据 | 波特率不对、DMA 未配 | 核对 USART1 配置 |
| 速度不准 | 极对数错、编码器比例错 | 核对 `Poles` 和编码器分辨率 |

---

## 15. 文档维护说明

### 15.1 目录约定

本工程产生的所有文档统一存放在：

```
E:\github\GIM6010\SguanFOC库文件\资料\
├── 文档\          ← 操作指南、移植记录、调参日志等 MD/PDF
├── 硬件资料\      ← 原理图、引脚定义、电机手册等
└── *.png          ← 流程截图等
```

### 15.2 建议后续文档

| 文档名 | 内容 |
|--------|------|
| `GIM6010_引脚与外设对照表.md` | CubeMX 引脚 vs 硬件原理图 |
| `GIM6010_电机参数记录.md` | 实测 Ld/Lq/Rs/Flux、极对数 |
| `GIM6010_调参日志.md` | 各阶段参数与测试结果 |
| `GIM6010_问题排查记录.md` | 遇到的问题与解决方案 |

### 15.3 修订记录

| 日期 | 版本 | 说明 |
|------|------|------|
| 2026-07-28 | v1.0 | 初版，AI 辅助移植整体操作步骤 |

---

<p align="center"><b>GIM6010 × SguanFOC — 分阶段移植，稳步上云霄</b></p>
