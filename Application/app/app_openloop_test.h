/**
  ******************************************************************************
  * @file    app_openloop_test.h
  * @brief   低电流开环拖转测试（电压矢量旋转，不依赖编码器校准）
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 Z.Xusheng
  * SPDX-License-Identifier: MIT
  *
  * This file is part of the GIM6010 joint motor controller firmware,
  * distributed under the MIT License. See the LICENSE file in the repository
  * root for full terms.
  ******************************************************************************
  */

#ifndef __APP_OPENLOOP_TEST_H
#define __APP_OPENLOOP_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* ===================== 可调参数（上板前在此改） =========================== */
/** 1 = 上电自动开环拖转（调试专用）；0 = 仅 shell 命令 `olrun` 启动 */
#define OLTEST_AUTO_START           (1)

/** 自动启动时的运行时长 [s]（仅 OLTEST_AUTO_START=1 时有效） */
#define OLTEST_AUTO_DURATION_S      (15.0f)

/** 目标 q 轴电压 [V]（d 轴恒为 0；约 Rs*Iq 量级，默认 ~0.35V ≈ 0.4A） */
#define OLTEST_VQ_TARGET_V          (0.40f)

/** 电角速度 [rad/s]：6.28≈1e圈/s 过慢肉眼难见；31.4≈5e圈/s 输出轴约 3.8°/s 可察觉 */
#define OLTEST_WE_ERADS             (31.4f)

/** 相电流峰值超限自动停机 [A]（低于 PROT_OC_LIMIT_A，留裕量） */
#define OLTEST_I_TRIP_A             (1.5f)

/** Vq 软启动爬升时间 [ms] */
#define OLTEST_RAMP_MS              (800U)

/** 1 = 开环启动时自动输出编码器 CSV（上位机 enclog）；0 = 手动 enclog on */
#define OLTEST_ENC_LOG_AUTO         (1U)
#define OLTEST_ENC_LOG_HZ           (20U)

/* Exported functions prototypes ---------------------------------------------*/
void APP_OLTEST_Init(void);
void APP_OLTEST_Poll(void);
HAL_StatusTypeDef APP_OLTEST_Start(float durationSec);
void              APP_OLTEST_Stop(void);
uint8_t           APP_OLTEST_IsRunning(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_OPENLOOP_TEST_H */
