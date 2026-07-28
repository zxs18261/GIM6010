/**
  ******************************************************************************
  * @file    bsp_ws2812.c
  * @brief   WS2812 类 RGB 状态灯 BSP（TIM4_CH2 PWM + DMA，单灯）
  * @note    GIM6010 硬件：PA12 = TIM4_CH2；其余逻辑同 GIM4310 参考实现。
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

/* Includes ------------------------------------------------------------------*/
#include "bsp_ws2812.h"
#include "board_config.h"
#include "tim.h"

/* Private defines -----------------------------------------------------------*/
#define WS2812_BIT_COUNT        (24U)
#define WS2812_RESET_SLOTS      (80U)
#define WS2812_BUF_LEN          (WS2812_BIT_COUNT + WS2812_RESET_SLOTS)

/* Private variables ---------------------------------------------------------*/
static DMA_HandleTypeDef hdma_tim4_ch2;

static uint32_t pwmBuf[WS2812_BUF_LEN];
static volatile uint8_t  busy = 0U;
static volatile uint8_t  pending = 0U;
static volatile uint32_t targetGrb = 0U;
static uint8_t brightness = BOARD_LED_BRIGHTNESS_DEFAULT;
static uint8_t curR = 0U;
static uint8_t curG = 0U;
static uint8_t curB = 0U;

/* Private function prototypes -----------------------------------------------*/
static void BSP_WS2812_Encode(uint32_t grb);
static void BSP_WS2812_RefreshTarget(void);

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  初始化 WS2812 驱动（重配 TIM4 + DMA）
  */
void BSP_WS2812_Init(void)
{
    htim4.Init.Prescaler         = 0U;
    htim4.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim4.Init.Period            = BOARD_WS2812_ARR;
    htim4.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
    {
        Error_Handler();
    }

    __HAL_RCC_DMAMUX1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    hdma_tim4_ch2.Instance                 = DMA1_Channel3;
    hdma_tim4_ch2.Init.Request             = DMA_REQUEST_TIM4_CH2;
    hdma_tim4_ch2.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_tim4_ch2.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_tim4_ch2.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_tim4_ch2.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_tim4_ch2.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    hdma_tim4_ch2.Init.Mode                = DMA_NORMAL;
    hdma_tim4_ch2.Init.Priority            = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_tim4_ch2) != HAL_OK)
    {
        Error_Handler();
    }
    __HAL_LINKDMA(&htim4, hdma[TIM_DMA_ID_CC2], hdma_tim4_ch2);

    HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, BOARD_IRQPRIO_LED_DMA, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

    BSP_WS2812_SetColor(0U, 0U, 0U);
    BSP_WS2812_Poll();
}

void BSP_WS2812_SetBrightness(uint8_t value)
{
    brightness = value;
    BSP_WS2812_RefreshTarget();
}

uint8_t BSP_WS2812_GetBrightness(void)
{
    return brightness;
}

void BSP_WS2812_SetColor(uint8_t r, uint8_t g, uint8_t b)
{
    curR = r;
    curG = g;
    curB = b;
    BSP_WS2812_RefreshTarget();
}

void BSP_WS2812_Poll(void)
{
    if ((pending != 0U) && (busy == 0U))
    {
        busy    = 1U;
        pending = 0U;
        BSP_WS2812_Encode(targetGrb);
        if (HAL_TIM_PWM_Start_DMA(&htim4, TIM_CHANNEL_2, pwmBuf, WS2812_BUF_LEN) != HAL_OK)
        {
            busy = 0U;
            pending = 1U;
        }
    }
}

void BSP_WS2812_DmaIRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_tim4_ch2);
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if ((htim == &htim4) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2))
    {
        HAL_TIM_PWM_Stop_DMA(&htim4, TIM_CHANNEL_2);
        busy = 0U;
    }
}

/* Private functions ---------------------------------------------------------*/

static void BSP_WS2812_Encode(uint32_t grb)
{
    uint32_t i;

    for (i = 0U; i < WS2812_BIT_COUNT; i++)
    {
        pwmBuf[i] = ((grb & (1UL << (23U - i))) != 0U) ? BOARD_WS2812_CCR_1 : BOARD_WS2812_CCR_0;
    }
    for (; i < WS2812_BUF_LEN; i++)
    {
        pwmBuf[i] = 0U;
    }
}

static void BSP_WS2812_RefreshTarget(void)
{
    uint32_t r = ((uint32_t)curR * (uint32_t)brightness + 127U) / 255U;
    uint32_t g = ((uint32_t)curG * (uint32_t)brightness + 127U) / 255U;
    uint32_t b = ((uint32_t)curB * (uint32_t)brightness + 127U) / 255U;

#if (BOARD_WS2812_ORDER_RGB != 0)
    targetGrb = (r << 16) | (g << 8) | b;
#else
    targetGrb = (g << 16) | (r << 8) | b;
#endif
    pending = 1U;
}
