/**
  ******************************************************************************
  * @file    app_telemetry.c
  * @brief   VOFA+ JustFloat 12 路遥测编排
  * @note    通道映射（VOFA 设 JustFloat / 921600 / 12 通道）：
  *          Ch0  Iu_A          U 相电流 [A]（重构）
  *          Ch1  Iv_A          V 相电流 [A]
  *          Ch2  Iw_A          W 相电流 [A]
  *          Ch3  Theta_e_deg   电角度 [deg]
  *          Ch4  Theta_m_deg    出轴机械角 [deg]（会话零位）
  *          Ch5  Pos_act_rad    出轴实际位置 [rad]
  *          Ch6  Pos_tgt_rad    出轴位置给定 [rad]
  *          Ch7  Spd_act        出轴实际速度 [rad/s]
  *          Ch8  Spd_tgt        出轴速度给定 [rad/s]
  *          Ch9  Vbus_V         母线电压 [V]
  *          Ch10 Id_A           d 轴电流 [A]
  *          Ch11 Iq_A           q 轴电流 [A]
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 Z.Xusheng
  * SPDX-License-Identifier: MIT
  ******************************************************************************
  */

#include "app_telemetry.h"
#include "bsp_vofa.h"
#include "bsp_uart.h"
#include "motor_ctrl.h"
#include "board_config.h"
#include "main.h"

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define TELEM_RAD_TO_DEG    (180.0 / M_PI)

static uint8_t  s_enabled;
static uint32_t s_periodMs;
static uint32_t s_lastTick;

/*
 * 作用：初始化 VOFA 遥测状态。
 * 逻辑：按 board_config 默认开关与频率设置周期计数。
 */
void APP_TELEM_Init(void)
{
    if (BOARD_TELEM_VOFA_DEFAULT_ON != 0U)
    {
        s_enabled = 1U;
    }
    else
    {
        s_enabled = 0U;
    }

    if ((BOARD_TELEM_VOFA_RATE_HZ == 0U) || (BOARD_TELEM_VOFA_RATE_HZ > 2000U))
    {
        s_periodMs = 1U;
    }
    else
    {
        s_periodMs = 1000U / BOARD_TELEM_VOFA_RATE_HZ;
        if (s_periodMs == 0U)
        {
            s_periodMs = 1U;
        }
    }

    s_lastTick = HAL_GetTick();

    if (s_enabled != 0U)
    {
        BSP_UART_Printf("# VOFA JustFloat %uch @ %u Hz (disable: vofa off)\r\n",
                        (unsigned int)BOARD_TELEM_VOFA_CH_COUNT,
                        (unsigned int)BOARD_TELEM_VOFA_RATE_HZ);
    }
}

uint8_t APP_TELEM_IsEnabled(void)
{
    return s_enabled;
}

/*
 * 作用：开关 VOFA 发送并设置频率。
 * 逻辑：rateHz 限 1~2000；开启时提示关闭 log/enclog 避免串口混叠。
 */
void APP_TELEM_SetEnabled(uint8_t enable, uint32_t rateHz)
{
    if ((rateHz == 0U) || (rateHz > 2000U))
    {
        rateHz = BOARD_TELEM_VOFA_RATE_HZ;
    }

    s_periodMs = 1000U / rateHz;
    if (s_periodMs == 0U)
    {
        s_periodMs = 1U;
    }

    s_enabled  = enable;
    s_lastTick = HAL_GetTick();

    if (enable != 0U)
    {
        BSP_UART_Printf("# VOFA on %u Hz, 12ch JustFloat; use 'log off' & 'enclog off'\r\n",
                        (unsigned int)rateHz);
    }
    else
    {
        BSP_UART_Printf("vofa off\r\n");
    }
}

/*
 * 作用：主循环按 1 kHz 发送 12 路 float 到 VOFA+。
 * 逻辑：限频 → 读 g_Mc 快照 → 组帧 → BSP_VOFA_SendJustFloat。
 */
void APP_TELEM_Poll(void)
{
    uint32_t now;
    float    channels[BOARD_TELEM_VOFA_CH_COUNT];
    float    posRad;
    float    posTgt;
    float    spdRadS;
    float    spdTgt;
    float    thetaE;
    float    iu;
    float    iv;
    float    iw;
    float    vbus;
    float    id;
    float    iq;

    if (s_enabled == 0U)
    {
        return;
    }

    now = HAL_GetTick();
    if ((now - s_lastTick) < s_periodMs)
    {
        return;
    }
    s_lastTick = now;

    posRad  = g_Mc.Pos;
    posTgt  = g_Mc.TgtPos;
    spdRadS = g_Mc.Speed;
    spdTgt  = g_Mc.TgtSpeed;
    thetaE  = g_Mc.ThetaE;
    iu      = g_Mc.Iu;
    iv      = g_Mc.Iv;
    iw      = g_Mc.Iw;
    vbus    = g_Mc.Vbus;
    id      = g_Mc.Foc.Idq.A;
    iq      = g_Mc.Foc.Idq.B;

    channels[0]  = iu;
    channels[1]  = iv;
    channels[2]  = iw;
    channels[3]  = thetaE * (float)TELEM_RAD_TO_DEG;
    channels[4]  = posRad * (float)TELEM_RAD_TO_DEG;
    channels[5]  = posRad;
    channels[6]  = posTgt;
    channels[7]  = spdRadS;
    channels[8]  = spdTgt;
    channels[9]  = vbus;
    channels[10] = id;
    channels[11] = iq;

    (void)BSP_VOFA_SendJustFloat(channels, BOARD_TELEM_VOFA_CH_COUNT);
}
