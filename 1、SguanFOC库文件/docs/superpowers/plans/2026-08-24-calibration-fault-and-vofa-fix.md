# GIM6010预定位故障与VOFA诊断修复 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 修复复位后虚假欠压、故障状态仍运行PWM和待机未显式关断的问题，使当前新固件首次母线上电能够安全执行装配校准，并通过VOFA读出预定位最终结果。

**Architecture:** 1 kHz Low Loop负责带去抖和滞环的保护状态转换，任何硬件故障或STANDBY都统一调用安全零输出；20 kHz High Loop只允许4～13运行态执行控制。校准结果保存在SguanFOC运行时结构，VOFA保持JustFloat与原0～20通道顺序不变，从Ch21追加诊断量。

**Tech Stack:** STM32G431 HAL、Keil MDK/Arm Compiler 6、SguanFOC、USART1 921600-8N1、JustFloat、MT6816 SPI3 DMA。

---

### Task 1: 修复保护状态机和零输出路径

**Files:**
- Modify: `SguanFOC/UserData_Config.h`
- Modify: `SguanFOC/SguanFOC.c`
- Modify: `SguanFOC/UserData_Function.h`

- [x] **Step 1: 固定故障复现判据**

确认当前源码仍满足：Ch0=15对应欠压、Ch0=0对应STANDBY；`Status_VBUS_UNDERVOLTAGE()`在电压仍低于下限时调用STANDBY；High Loop允许14～18；`Status_Switch_STANDBY()`没有关闭MOE。

- [x] **Step 2: 增加保护去抖与恢复滞环常量**

在`UserData_Config.h`新增带单位中文备注的欠压20 ms进入确认、过压1 ms确认和1 V母线恢复滞环。1 kHz Low Loop按连续样本计数，不用20 kHz ISR延时。

- [x] **Step 3: 修正进入和恢复逻辑**

欠压必须连续20个1 ms样本越界才写入15，过压最多确认1个1 ms样本后写入14；恢复必须连续满足安全区，欠压为`VBUS >= VBUS_MIN + 1 V`、过压为`VBUS <= VBUS_MAX - 1 V`。删除“故障持续十倍周期后强制进入IDLE”的不安全路径；恢复后只进入STANDBY，等待显式`CO=1?`。

- [x] **Step 4: 统一安全清零**

`Status_Switch_STANDBY()`和所有14以上故障立即清零Ud/Uq、Id/Iq/速度/位置目标，写零矢量并关闭TIM1 MOE。High Loop控制范围改为4～13；14以上只维持MOE关闭，不清除Flash偏置和电流零偏。

- [x] **Step 5: 静态验证**

运行`rg`确认不存在`status > 3 && status < 19`旧门控、不存在故障超时强制`Status_Switch_IDLE`，并确认STANDBY路径调用统一安全清零。

### Task 2: 增加校准结果与母线启动诊断

**Files:**
- Modify: `BSP/BSP_ADC.h`
- Modify: `BSP/BSP_ADC.c`
- Modify: `SguanFOC/SguanFOC.h`
- Modify: `SguanFOC/SguanFOC.c`

- [x] **Step 1: 保存复位后母线最小采样**

在ADC BSP中保存首个有效样本以来的最小换算母线电压，新增只读getter；该值用于判断是否存在低于12 V的启动瞬态，不参与控制。

- [x] **Step 2: 定义校准结果枚举与运行时结构**

结果值固定为：0未判定、1等待母线、2正在校准、3本次校准并保存成功、4从Flash加载、5编码器失败、6相序检查失败、7 Flash保存失败。结构同时保存实际施加的Ud[V]、最终相序结果和Flash有效标志。

- [x] **Step 3: 在每条校准出口写结果**

Flash加载写4且实际Ud为0；等待母线写1；打开MOE前写2和实际Ud；新校准保存成功写3；三类失败分别写5/6/7并调用安全清零。新构建的固件签名会使旧记录失效，从而让当前版本首次上母线执行校准。

### Task 3: 从Ch21追加VOFA通道

**Files:**
- Modify: `SguanFOC/Sguan_printf.h`
- Modify: `SguanFOC/UserData_UserControl.h`
- Modify: `SguanFOC/UserData_Function.h`

- [x] **Step 1: 通道数从21改为36**

保持Ch0～Ch20完全不变，新增Ch21～Ch35；36个float加4字节帧尾共148 byte，500 Hz为74000 byte/s，占921600-8N1理论线速约80.3%。

- [x] **Step 2: 写入追加通道**

Ch21校准结果；Ch22本次实际对齐Ud[V]；Ch23相序结果；Ch24 Flash校准有效；Ch25母线ADC有效；Ch26复位后母线最小值[V]；Ch27～Ch33依次为Encoder_Dir、Motor_Dir、PWM_Dir、Current_Dir0、Current_Dir1、Current_Num、极对数；Ch34为TIM1 MOE实际状态；Ch35为当前Ud[V]。

- [x] **Step 3: 静态检查帧结构**

确认DMA发送长度继续使用`sizeof(PRINTF_STRUCT)`，帧尾保持`00 00 80 7F`，原通道索引没有移动或复用。

### Task 4: 同步文档与修改记录

**Files:**
- Modify: `.cursor/rules/gim6010-vofa-contract.mdc`
- Modify: `docs/foc/vofa-contract.md`
- Modify: `docs/foc/project-profile.md`
- Modify: `docs/foc/stage-status.md`
- Modify: `docs/foc/debug-records/2026-08-24-calibration-no-visible-motion.md`
- Create: `资料/文档/GIM6010_预定位故障与VOFA诊断修改说明.md`
- Modify: `资料/文档/GIM6010_VOFA调试对接指南.md`

- [x] **Step 1: 更新36通道通信契约和带宽**

同步串口921600、148 byte帧、500 Hz和Ch21～Ch35定义，明确Ch22是本次校准实际施加值、Ch35是当前输出值。

- [x] **Step 2: 写明预定位后的期望值**

新校准成功后预期：Ch0=4、Ch21=3、Ch22约0.96、Ch23=1、Ch24=1、Ch25=1、Ch27～Ch33为保存后的装配参数、Ch34=0、Ch35=0；Ch7=0、Ch18=0，Ch5约为TIM1中点2125。后续掉电重启加载Flash时Ch21=4、Ch22=0且不转动。

- [x] **Step 3: 更新证据等级**

源码和编译结果标source-confirmed；只有用户复测得到上述VOFA序列后才能把预定位、欠压去抖和零输出标为bench-verified。

### Task 5: 编译和嵌入式审查

**Files:**
- Verify: `MDK-ARM/GIM6010_V0.0.uvprojx`
- Verify: `MDK-ARM/GIM6010_V0.0/GIM6010_V0_0.build_log.htm`
- Verify: `MDK-ARM/GIM6010_V0.0/GIM6010_V0_0.hex`

- [x] **Step 1: 运行Keil全量构建**

使用当前工程已验证的Keil命令构建，要求0 error、0 warning；记录Code/RO/RW/ZI大小和HEX SHA-256。

- [x] **Step 2: 执行embedded-review**

重点检查1 kHz/20 kHz共享状态、非原子字段、故障时MOE、ADC ISR、Flash写入窗口、VOFA帧尺寸和DMA忙处理。当前没有ACP双模型能力时使用单模型审查并如实记录。

- [x] **Step 3: 最终差异审计**

只审计本计划涉及的源码和文档，不覆盖用户现有脏工作区；确认每处代码增删具有目的、单位或安全原因的中文备注。
