/**
  ******************************************************************************
  * @file    bsp_clock.c
  * @brief   时钟切换 BSP（HSI -> HSE 8MHz，SYSCLK 维持 170MHz）
  * @note    CubeMX 已用 HSE 8MHz / PLLM2 / PLLN85 / PLLR2 = 170MHz 初始化；
  *          本模块在启动早期确认 HSE 就绪并标记状态，失败则回退 HSI 配置。
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

#include "bsp_clock.h"

static uint8_t clockOnHse = 0U;

static HAL_StatusTypeDef BSP_CLOCK_ConfigPll(uint32_t pllSource, uint32_t pllm);

HAL_StatusTypeDef BSP_CLOCK_SwitchToHse(void)
{
    RCC_OscInitTypeDef oscInit = {0};

    if (__HAL_RCC_GET_SYSCLK_SOURCE() == RCC_CFGR_SWS_PLL)
    {
        if (__HAL_RCC_GET_PLL_OSCSOURCE() == RCC_PLLSOURCE_HSE)
        {
            clockOnHse = 1U;
            return HAL_OK;
        }
    }

    oscInit.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    oscInit.HSEState       = RCC_HSE_ON;
    oscInit.PLL.PLLState   = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&oscInit) != HAL_OK)
    {
        clockOnHse = 0U;
        return HAL_ERROR;
    }

    /* HSE 8MHz / PLLM 2 * PLLN 85 / PLLR 2 = 170MHz */
    if (BSP_CLOCK_ConfigPll(RCC_PLLSOURCE_HSE, RCC_PLLM_DIV2) != HAL_OK)
    {
        (void)BSP_CLOCK_ConfigPll(RCC_PLLSOURCE_HSI, RCC_PLLM_DIV4);
        clockOnHse = 0U;
        return HAL_ERROR;
    }

    clockOnHse = 1U;
    return HAL_OK;
}

uint8_t BSP_CLOCK_IsHse(void)
{
    return clockOnHse;
}

static HAL_StatusTypeDef BSP_CLOCK_ConfigPll(uint32_t pllSource, uint32_t pllm)
{
    RCC_OscInitTypeDef oscInit = {0};
    RCC_ClkInitTypeDef clkInit = {0};

    clkInit.ClockType      = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK
                           | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clkInit.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
    clkInit.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clkInit.APB1CLKDivider = RCC_HCLK_DIV1;
    clkInit.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clkInit, FLASH_LATENCY_4) != HAL_OK)
    {
        return HAL_ERROR;
    }

    oscInit.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    oscInit.HSIState            = RCC_HSI_ON;
    oscInit.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    oscInit.PLL.PLLState        = RCC_PLL_ON;
    oscInit.PLL.PLLSource       = pllSource;
    oscInit.PLL.PLLM            = pllm;
    oscInit.PLL.PLLN            = 85U;
    oscInit.PLL.PLLP            = RCC_PLLP_DIV2;
    oscInit.PLL.PLLQ            = RCC_PLLQ_DIV2;
    oscInit.PLL.PLLR            = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&oscInit) != HAL_OK)
    {
        return HAL_ERROR;
    }

    clkInit.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    return HAL_RCC_ClockConfig(&clkInit, FLASH_LATENCY_4);
}
