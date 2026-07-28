/**
  ******************************************************************************
  * @file    app_openloop_test.c
  * @brief   低电流开环拖转测试模块
  * @note    - 使用 MC_MODE_OPENLOOP：内部积分电角度，不经编码器换相；
  *          - Vq 软启动 + 相电流峰值监测，超限或超时自动封波；
  *          - 无需 cal 校准即可运行（仅用于验证功率级与相序）。
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

#include "app_openloop_test.h"
#include "app_can_protocol.h"
#include "app_monitor.h"
#include "motor_ctrl.h"
#include "bsp_uart.h"
#include "motor_config.h"
#include <math.h>

typedef enum
{
    OLTEST_IDLE = 0U,
    OLTEST_RAMP,
    OLTEST_RUN,
} OLTEST_StateTypeDef;

static OLTEST_StateTypeDef olState = OLTEST_IDLE;
static uint32_t olStartTick = 0U;
static uint32_t olRampStartTick = 0U;
static float    olDurationMs = 0.0f;

static float OLTEST_PhasePeakA(void)
{
    float iu = fabsf(g_Mc.Iu);
    float iv = fabsf(g_Mc.Iv);
    float iw = fabsf(g_Mc.Iw);
    float peak = iu;

    if (iv > peak)
    {
        peak = iv;
    }
    if (iw > peak)
    {
        peak = iw;
    }
    return peak;
}

static void OLTEST_ApplyVq(float vq)
{
    MC_SetOpenLoop(0.0f, vq, OLTEST_WE_ERADS);
}

static void OLTEST_Finish(const char *reason)
{
    MC_Disable();
    (void)MC_SetMode(MC_MODE_DISABLED);
    olState = OLTEST_IDLE;
    BSP_UART_Printf("[oltest] stopped: %s (Iu=%.2f Iv=%.2f Iw=%.2f A)\r\n",
                    reason, g_Mc.Iu, g_Mc.Iv, g_Mc.Iw);
}

void APP_OLTEST_Init(void)
{
#if (OLTEST_AUTO_START != 0)
    (void)APP_OLTEST_Start(OLTEST_AUTO_DURATION_S);
#endif
}

HAL_StatusTypeDef APP_OLTEST_Start(float durationSec)
{
    if (g_Mc.Mode == MC_MODE_FAULT)
    {
        BSP_UART_Printf("[oltest] fault latched, run 'clear' first\r\n");
        return HAL_ERROR;
    }
    if (olState != OLTEST_IDLE)
    {
        APP_OLTEST_Stop();
    }
    if (durationSec <= 0.0f)
    {
        durationSec = 10.0f;
    }

    if (MC_SetMode(MC_MODE_OPENLOOP) != HAL_OK)
    {
        BSP_UART_Printf("[oltest] set mode failed (disable first)\r\n");
        return HAL_ERROR;
    }

    OLTEST_ApplyVq(0.0f);
    if (MC_Enable() != HAL_OK)
    {
        BSP_UART_Printf("[oltest] enable failed\r\n");
        (void)MC_SetMode(MC_MODE_DISABLED);
        return HAL_ERROR;
    }

    APP_CAN_SetControlled(0U);
    olRampStartTick = HAL_GetTick();
    olStartTick     = olRampStartTick;
    olDurationMs    = durationSec * 1000.0f;
    olState         = OLTEST_RAMP;

    BSP_UART_Printf("[oltest] start: Vq_target=%.2fV we=%.2f rad/s trip=%.1fA duration=%.1fs\r\n",
                    OLTEST_VQ_TARGET_V, OLTEST_WE_ERADS, OLTEST_I_TRIP_A, durationSec);

#if (OLTEST_ENC_LOG_AUTO != 0)
    APP_MON_SetEncLog(1U, OLTEST_ENC_LOG_HZ);
    BSP_UART_Printf("[oltest] enclog on %u Hz (enclog off to stop)\r\n",
                    (unsigned int)OLTEST_ENC_LOG_HZ);
#endif
    return HAL_OK;
}

void APP_OLTEST_Stop(void)
{
    if (olState == OLTEST_IDLE)
    {
        return;
    }
    OLTEST_Finish("user stop");
}

uint8_t APP_OLTEST_IsRunning(void)
{
    return (olState != OLTEST_IDLE) ? 1U : 0U;
}

void APP_OLTEST_Poll(void)
{
    uint32_t tick;
    float peakI;
    float vqNow;

    if (olState == OLTEST_IDLE)
    {
        return;
    }

    if (g_Mc.Enabled == 0U)
    {
        olState = OLTEST_IDLE;
        return;
    }

    peakI = OLTEST_PhasePeakA();
    if (peakI >= OLTEST_I_TRIP_A)
    {
        OLTEST_Finish("overcurrent");
        return;
    }

    tick = HAL_GetTick();

    if (olState == OLTEST_RAMP)
    {
        float rampRatio;

        if (OLTEST_RAMP_MS == 0U)
        {
            rampRatio = 1.0f;
        }
        else
        {
            rampRatio = (float)(tick - olRampStartTick) / (float)OLTEST_RAMP_MS;
            if (rampRatio > 1.0f)
            {
                rampRatio = 1.0f;
            }
        }

        vqNow = OLTEST_VQ_TARGET_V * rampRatio;
        OLTEST_ApplyVq(vqNow);

        if (rampRatio >= 1.0f)
        {
            olState     = OLTEST_RUN;
            olStartTick = tick;
            BSP_UART_Printf("[oltest] ramp done, Vq=%.2fV\r\n", vqNow);
        }
        return;
    }

    if ((float)(tick - olStartTick) >= olDurationMs)
    {
        OLTEST_Finish("timeout");
    }
}
