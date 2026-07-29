/**
  ******************************************************************************
  * @file    BSP_RGB.c
  * @brief   GIM6010 WS2812 RGB 驱动（TIM4_CH2 PWM + DMA1_CH3，单灯）
  * @note    PA12 = TIM4_CH2；Init 会重配 TIM4 为 WS2812 时序，专用于 RGB。
  ******************************************************************************
  */

#include "BSP_RGB.h"
#include "main.h"
#include "tim.h"

/* Private defines -----------------------------------------------------------*/
#define WS2812_BIT_COUNT    (24U)
#define WS2812_RESET_SLOTS  (80U)
#define WS2812_BUF_LEN      (WS2812_BIT_COUNT + WS2812_RESET_SLOTS)

#define BSP_RGB_DMA_IRQ_PRIO (8U)

/* Private variables ---------------------------------------------------------*/
static DMA_HandleTypeDef hdma_tim4_ch2;

static uint32_t pwmBuf[WS2812_BUF_LEN];
static volatile uint8_t  busy = 0U;
static volatile uint8_t  pending = 0U;
static volatile uint32_t targetGrb = 0U;
static uint8_t brightness = BSP_RGB_DEFAULT_BRIGHTNESS;
static uint8_t curR = 0U;
static uint8_t curG = 0U;
static uint8_t curB = 0U;

/* Private function prototypes -----------------------------------------------*/
static void BSP_RGB_Encode(uint32_t grb);
static void BSP_RGB_RefreshTarget(void);
static void BSP_RGB_ApplyScaledColor(uint8_t r, uint8_t g, uint8_t b, uint8_t level);

/* Exported functions --------------------------------------------------------*/

void BSP_RGB_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim4.Init.Prescaler         = 0U;
    htim4.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim4.Init.Period            = BSP_RGB_WS2812_ARR;
    htim4.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
    {
        Error_Handler();
    }

    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 0U;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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

    HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, BSP_RGB_DMA_IRQ_PRIO, 0U);
    HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

    brightness = BSP_RGB_DEFAULT_BRIGHTNESS;
    BSP_RGB_Off();
    BSP_RGB_Poll();
}

void BSP_RGB_Poll(void)
{
    if ((pending != 0U) && (busy == 0U))
    {
        busy    = 1U;
        pending = 0U;
        BSP_RGB_Encode(targetGrb);
        if (HAL_TIM_PWM_Start_DMA(&htim4, TIM_CHANNEL_2, pwmBuf, WS2812_BUF_LEN) != HAL_OK)
        {
            busy    = 0U;
            pending = 1U;
        }
    }
}

void BSP_RGB_SetBrightness(uint8_t value)
{
    brightness = value;
    BSP_RGB_RefreshTarget();
}

uint8_t BSP_RGB_GetBrightness(void)
{
    return brightness;
}

void BSP_RGB_SetColor(uint8_t r, uint8_t g, uint8_t b)
{
    curR = r;
    curG = g;
    curB = b;
    BSP_RGB_RefreshTarget();
}

void BSP_RGB_SetColorRGB(uint32_t rgb)
{
    BSP_RGB_SetColor((uint8_t)((rgb >> 16) & 0xFFU),
                     (uint8_t)((rgb >> 8) & 0xFFU),
                     (uint8_t)(rgb & 0xFFU));
}

void BSP_RGB_Off(void)
{
    BSP_RGB_SetColor(0U, 0U, 0U);
}

void BSP_RGB_DmaIRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_tim4_ch2);
}

uint8_t BSP_RGB_BreathLevel(uint32_t tick, uint32_t periodMs)
{
    uint32_t phase;
    uint32_t half;
    uint32_t level;

    if (periodMs == 0U)
    {
        return 255U;
    }

    phase = tick % periodMs;
    half  = periodMs / 2U;
    if (phase >= half)
    {
        phase = periodMs - phase;
    }
    level = 16U + (phase * (255U - 16U)) / half;
    return (uint8_t)level;
}

uint8_t BSP_RGB_BlinkOn(uint32_t tick, uint32_t halfPeriodMs)
{
    if (halfPeriodMs == 0U)
    {
        return 1U;
    }
    return (((tick / halfPeriodMs) & 0x01U) == 0U) ? 1U : 0U;
}

uint8_t BSP_RGB_FaultBlinkOn(uint32_t tick, uint8_t blinkCount,
                             uint32_t halfPeriodMs, uint32_t pauseMs)
{
    uint32_t activeSlots;
    uint32_t period;
    uint32_t phase;
    uint32_t slot;

    if ((blinkCount == 0U) || (halfPeriodMs == 0U))
    {
        return 0U;
    }

    activeSlots = (uint32_t)blinkCount * 2U;
    period      = activeSlots * halfPeriodMs + pauseMs;
    phase       = tick % period;
    slot        = phase / halfPeriodMs;

    if (slot >= activeSlots)
    {
        return 0U;
    }
    return ((slot & 0x01U) == 0U) ? 1U : 0U;
}

void BSP_RGB_SetColorLevel(uint8_t r, uint8_t g, uint8_t b, uint8_t level)
{
    BSP_RGB_ApplyScaledColor(r, g, b, level);
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if ((htim == &htim4) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2))
    {
        (void)HAL_TIM_PWM_Stop_DMA(&htim4, TIM_CHANNEL_2);
        busy = 0U;
    }
}

/* Private functions ---------------------------------------------------------*/

static void BSP_RGB_Encode(uint32_t grb)
{
    uint32_t i;

    for (i = 0U; i < WS2812_BIT_COUNT; i++)
    {
        pwmBuf[i] = ((grb & (1UL << (23U - i))) != 0U) ? BSP_RGB_WS2812_CCR_1 : BSP_RGB_WS2812_CCR_0;
    }
    for (; i < WS2812_BUF_LEN; i++)
    {
        pwmBuf[i] = 0U;
    }
}

static void BSP_RGB_RefreshTarget(void)
{
    uint32_t r;
    uint32_t g;
    uint32_t b;

    r = ((uint32_t)curR * (uint32_t)brightness + 127U) / 255U;
    g = ((uint32_t)curG * (uint32_t)brightness + 127U) / 255U;
    b = ((uint32_t)curB * (uint32_t)brightness + 127U) / 255U;

#if (BSP_RGB_ORDER_RGB != 0U)
    targetGrb = (r << 16) | (g << 8) | b;
#else
    targetGrb = (g << 16) | (r << 8) | b;
#endif
    pending = 1U;
}

static void BSP_RGB_ApplyScaledColor(uint8_t r, uint8_t g, uint8_t b, uint8_t level)
{
    uint32_t sr;
    uint32_t sg;
    uint32_t sb;

    sr = ((uint32_t)r * (uint32_t)level + 127U) / 255U;
    sg = ((uint32_t)g * (uint32_t)level + 127U) / 255U;
    sb = ((uint32_t)b * (uint32_t)level + 127U) / 255U;

    sr = (sr * (uint32_t)brightness + 127U) / 255U;
    sg = (sg * (uint32_t)brightness + 127U) / 255U;
    sb = (sb * (uint32_t)brightness + 127U) / 255U;

#if (BSP_RGB_ORDER_RGB != 0U)
    targetGrb = (sr << 16) | (sg << 8) | sb;
#else
    targetGrb = (sg << 16) | (sr << 8) | sb;
#endif
    pending = 1U;
}
