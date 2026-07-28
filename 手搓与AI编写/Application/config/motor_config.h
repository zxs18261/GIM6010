/**
  ******************************************************************************
  * @file    motor_config.h
  * @brief   电机参数与控制参数统一配置（GIM6010-8 出轴款）
  * @note    - 通过 MOTOR_TYPE 选择目标电机；
  *          - GIM6010-8：14 极对数、减速比 8:1（伺泰威官方资料）；
  *          - 相阻/相感/Kt 等为工程初值，建议预定位校准与实测修正。
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

#ifndef __MOTOR_CONFIG_H
#define __MOTOR_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "board_config.h"

/* ============================ 电机选型 ==================================== */
#define MOTOR_TYPE_GIM6010_8        (1)

#ifndef MOTOR_TYPE
#define MOTOR_TYPE                  MOTOR_TYPE_GIM6010_8
#endif

#if (MOTOR_TYPE == MOTOR_TYPE_GIM6010_8)

#define MOTOR_NAME                  "GIM6010-8"
#define MOTOR_POLE_PAIRS            (14U)
#define MOTOR_GEAR_RATIO            (8.0f)
/** 以下为工程初值 [待实测确认] */
#define MOTOR_RS_OHM                (0.90f)
#define MOTOR_LS_H                  (0.000350f)
#define MOTOR_KT_ROTOR              (0.085f)
#define MOTOR_CUR_RATED_A           (2.5f)
#define MOTOR_CUR_PEAK_A            (14.0f)
#define MOTOR_SPEED_MAX_RADS        (60.0f)

#else
#error "MOTOR_TYPE not supported"
#endif

#define MOTOR_KT_OUT                (MOTOR_KT_ROTOR * MOTOR_GEAR_RATIO)

/* ============================ 控制参数 ==================================== */
#define CTRL_CURRENT_TS             (1.0f / (float)BOARD_PWM_FREQ_HZ)
#define CTRL_CURRENT_BW_RADS        (6283.2f)
#define CTRL_DECOUPLE_ENABLE        (1U)

#define CTRL_SLOW_LOOP_DIV          (20U)
#define CTRL_SLOW_TS                (CTRL_CURRENT_TS * (float)CTRL_SLOW_LOOP_DIV)
#define CTRL_SPEED_KP               (1.5f)
#define CTRL_SPEED_KI               (15.0f)
#define CTRL_POS_KP                 (20.0f)
#define CTRL_POS_SPEED_LIMIT        (15.0f)
#define CTRL_PLL_BW_RADS            (1256.6f)

/* ============================ 校准参数 ==================================== */
#define CAL_CURRENT_A               (3.0f)
#define CAL_RAMP_TIME_S             (0.5f)
#define CAL_SETTLE_TIME_S           (0.8f)
#define CAL_ROTATE_EREV             (8U)
#define CAL_ROTATE_SPEED_ERADS      (12.57f)
#define CAL_OFFSET_POINTS           (16U)

/* ============================ 保护参数 ==================================== */
#define PROT_OC_LIMIT_A             (15.0f)
#define PROT_OV_LIMIT_V             (30.0f)
#define PROT_UV_LIMIT_V             (9.0f)
#define PROT_VBUS_FAULT_MS          (10U)
#define PROT_MOS_OT_LIMIT_C         (90.0f)
#define PROT_MOS_WARN_C             (75.0f)
#define PROT_MOT_OT_LIMIT_C         (110.0f)
#define PROT_MOT_WARN_C             (90.0f)
#define PROT_ENC_ERR_LIMIT          (16U)
#define PROT_CAN_TIMEOUT_MS         (500U)

/* ========================= MIT 协议量程 ==================================== */
#define MIT_P_MAX                   (12.5f)
#define MIT_V_MAX                   (65.0f)
#define MIT_KP_MAX                  (500.0f)
#define MIT_KD_MAX                  (5.0f)
#define MIT_T_MAX                   (18.0f)

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_CONFIG_H */
