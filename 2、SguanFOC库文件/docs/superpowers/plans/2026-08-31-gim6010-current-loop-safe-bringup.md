# GIM6010 Current-Loop Safe Bring-Up Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Produce a bounded GIM6010 current-loop diagnostic firmware that cannot recreate the unbounded feedforward voltage seen in `vofa+4.csv`.

**Architecture:** Keep board-specific ADC routing in `UserData_Function.h`, keep bench limits in `UserData_Config.h`, and enforce a final bounded-time DQ voltage safety gate in the float control path immediately before SVPWM. Keep VOFA live values and frozen fault snapshots on the same IV/IW mapping used by control.

**Tech Stack:** STM32G431, SguanFOC 3.0.1, Arm Compiler 5, Keil MDK 5.42, PowerShell static verification.

---

### Task 1: Establish a red-capable static safety check

**Files:**
- Create: `tools/verify_foc_bringup_safety.ps1`

- [ ] Run the verifier against the pre-change project:

```powershell
powershell -ExecutionPolicy Bypass -File tools\verify_foc_bringup_safety.ps1 -ProjectRoot 'E:\github\GIM6010\2、SguanFOC库文件'
```

Expected: `FAIL` because the old project swaps IV/IW and has no final 0.30 V DQ limiter.

### Task 2: Restore one authoritative current-channel mapping

**Files:**
- Modify: `SguanFOC v3.0.1/UserData_Function.h`
- Modify: `SguanFOC v3.0.1/UserData_UserControl.h`
- Modify: `SguanFOC v3.0.1/SguanFOC.c`

- [ ] Make `User_ReadADC_Raw(0)` read `ADC_InjectedValues[0]` (IV/Rank1/PB1) and channel 1 read index 1 (IW/Rank2/PB0).
- [ ] Apply the same index-to-offset mapping to VOFA `current0_live/current1_live` and the frozen fault snapshot.
- [ ] Keep `Current_Dir0/1=-1`; channel identity and feedback polarity remain independent variables.

### Task 3: Bound the current-loop actuator command

**Files:**
- Modify: `SguanFOC v3.0.1/UserData_Config.h`
- Modify: `SguanFOC v3.0.1/SguanFOC.c`

- [ ] Define `USER_CURRENT_FEEDFORWARD_ENABLE 0` and `USER_CLOSED_LOOP_DQ_VOLTAGE_LIMIT_V 0.30f`.
- [ ] In `Control_Current_SINGLE`, compile `Ud_ff/Uq_ff` as zero while the motor inductance unit remains unverified.
- [ ] Before closed-loop SVPWM, reject NaN/Inf and scale by the conservative bound `|Ud|+|Uq| <= 0.30 V`, which guarantees `sqrt(Ud^2+Uq^2) <= 0.30 V` without an unbounded square-root loop in the 20 kHz ISR.

### Task 4: Verify and build

**Files:**
- Build: `MDK-ARM/GIM6010_V0.0.uvprojx`
- Verify: `MDK-ARM/GIM6010_V0.0/GIM6010_V0_0.build_log.htm`
- Verify: `MDK-ARM/GIM6010_V0.0/GIM6010_V0_0.axf`
- Verify: `MDK-ARM/GIM6010_V0.0/GIM6010_V0_0.hex`

- [ ] Run `tools/verify_foc_bringup_safety.ps1`; expect every assertion to pass.
- [ ] Run Keil rebuild (`UV4.exe -r GIM6010_V0.0.uvprojx -j0`) from `MDK-ARM`.
- [ ] Require `0 Error(s), 0 Warning(s)` and a nonzero build duration with `compiling SguanFOC.c` in the log.
- [ ] Require AXF/HEX timestamps to be newer than all four modified source headers/files.

### Task 5: Bench acceptance gate

- [ ] Flash with the motor bus off, reset into `Ch0=0`, select `BO=1?`, then use a low-voltage current-limited supply.
- [ ] Send `CO=1?`, wait for `Ch0=4`, verify raw ADC and offsets near midscale, and confirm `Ch7` is near zero.
- [ ] Send `AO=0.01?` only briefly, followed by `AO=0?` and `CO=0?`.
- [ ] Stop immediately for `|Id|` or `|Iq| > 0.5 A`, ADC rail saturation, supply current limiting, bus collapse, or nonzero output beyond the 0.30 V bound.

Bench behavior is the only valid regression test for the original OCP symptom; compilation and static checks cannot prove hardware phase order or feedback sign.
