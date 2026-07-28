/**
  ******************************************************************************
  * @file    bsp_adc.c
  * @brief   ADC 采样 BSP（GIM6010：INA181 电流 + 母线电压注入组 + 温度规则组）
  * @note    采样架构：
  *          - 注入组由 TIM1 TRGO2 硬件触发，20kHz：
  *            ADC1 = Iv + Iw + Vbus，ADC2 = Ibus；
  *          - V/W 相 INA181×20 低侧采样，U 相由 motor_ctrl 推算；
  *          - meas.Iu 槽映射 W 相，meas.Iv 槽映射 V 相（与 FOC 接口一致）；
  *          - NTC 两路均在 ADC1 规则组 rank1/rank2（PA2/PA3）。
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
#include "bsp_adc.h"
#include "board_config.h"
#include "adc.h"
#include <math.h>

/* Private variables ---------------------------------------------------------*/
static float offsetU   = 2048.0f;   /*!< W 相（映射 Iu 槽）零流偏置 [LSB] */
static float offsetV   = 2048.0f;   /*!< V 相零流偏置 [LSB] */
static float offsetBus = 2048.0f;   /*!< 母线电流零流偏置 [LSB] */
static volatile float vbusFilt = 0.0f;
static ADC_FocCallbackTypeDef focCallback = NULL;

/* Private function prototypes -----------------------------------------------*/
static float BSP_ADC_NtcToTemp(uint16_t raw, float r25, float beta);
static float BSP_ADC_ReadRegularRank(ADC_HandleTypeDef *hadc, uint32_t rank);
static uint8_t BSP_ADC_OffsetInRange(float offs, float minLsb, float maxLsb);

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  初始化 ADC（重配注入组/触发/校准）
  */
void BSP_ADC_Init(void)
{
    ADC_InjectionConfTypeDef jConfig = {0};
    ADC_ChannelConfTypeDef chConfig  = {0};

    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK)
    {
        Error_Handler();
    }

    /* ADC1 规则组：双 rank NTC（须在注入组启动前完成） */
    hadc1.Init.ScanConvMode    = ENABLE;
    hadc1.Init.EOCSelection    = ADC_EOC_SEQ_CONV;
    hadc1.Init.NbrOfConversion = 2U;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }

    chConfig.SamplingTime = BOARD_ADC_NTC_SMPTIME;
    chConfig.SingleDiff   = ADC_SINGLE_ENDED;
    chConfig.OffsetNumber = ADC_OFFSET_NONE;
    chConfig.Offset       = 0U;

    chConfig.Rank    = BOARD_ADC_NTC_REGULAR_RANK_MOS;
    chConfig.Channel = BOARD_ADC1_RCH_MOSNTC;
    if (HAL_ADC_ConfigChannel(&hadc1, &chConfig) != HAL_OK)
    {
        Error_Handler();
    }
    chConfig.Rank    = BOARD_ADC_NTC_REGULAR_RANK_MOT;
    chConfig.Channel = BOARD_ADC1_RCH_MOTNTC;
    if (HAL_ADC_ConfigChannel(&hadc1, &chConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /* ADC1 注入：Iv + Iw + Vbus */
    jConfig.InjectedSamplingTime          = BOARD_ADC_CUR_SMPTIME;
    jConfig.InjectedSingleDiff            = ADC_SINGLE_ENDED;
    jConfig.InjectedOffsetNumber          = ADC_OFFSET_NONE;
    jConfig.InjectedOffset                = 0U;
    jConfig.InjectedNbrOfConversion       = 3U;
    jConfig.InjectedDiscontinuousConvMode = DISABLE;
    jConfig.AutoInjectedConv              = DISABLE;
    jConfig.QueueInjectedContext          = DISABLE;
    jConfig.ExternalTrigInjecConv         = ADC_EXTERNALTRIGINJEC_T1_TRGO2;
    jConfig.ExternalTrigInjecConvEdge     = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
    jConfig.InjecOversamplingMode         = DISABLE;

    jConfig.InjectedRank    = ADC_INJECTED_RANK_1;
    jConfig.InjectedChannel = BOARD_ADC1_JCH_IV;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &jConfig) != HAL_OK)
    {
        Error_Handler();
    }
    jConfig.InjectedRank    = ADC_INJECTED_RANK_2;
    jConfig.InjectedChannel = BOARD_ADC1_JCH_IW;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &jConfig) != HAL_OK)
    {
        Error_Handler();
    }
    jConfig.InjectedRank         = ADC_INJECTED_RANK_3;
    jConfig.InjectedChannel      = BOARD_ADC1_JCH_VBUS;
    jConfig.InjectedSamplingTime = BOARD_ADC_VBUS_SMPTIME;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &jConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /* ADC2 注入：Ibus */
    jConfig.InjectedSamplingTime    = BOARD_ADC_CUR_SMPTIME;
    jConfig.InjectedNbrOfConversion = 1U;
    jConfig.InjectedRank            = ADC_INJECTED_RANK_1;
    jConfig.InjectedChannel         = BOARD_ADC2_JCH_IBUS;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &jConfig) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_ADCEx_InjectedStart(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_ADCEx_InjectedStart(&hadc2) != HAL_OK)
    {
        Error_Handler();
    }
}

static uint8_t BSP_ADC_OffsetInRange(float offs, float minLsb, float maxLsb)
{
    return ((offs >= minLsb) && (offs <= maxLsb)) ? 1U : 0U;
}

/**
  * @brief  测量电流通道零流偏置（阻塞，封波状态下调用）
  */
HAL_StatusTypeDef BSP_ADC_MeasureOffsets(uint32_t samples)
{
    uint32_t i;
    uint32_t timeout;
    float sumU   = 0.0f;
    float sumV   = 0.0f;
    float sumBus = 0.0f;
    uint8_t phaseOk;
    uint8_t busOk;

    for (i = 0U; i < samples; i++)
    {
        __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOS);
        timeout = 1000000U;
        while (__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_JEOS) == 0U)
        {
            if (--timeout == 0U)
            {
                return HAL_TIMEOUT;
            }
        }

        sumV   += (float)hadc1.Instance->JDR1;
        sumU   += (float)hadc1.Instance->JDR2;
        sumBus += (float)hadc2.Instance->JDR1;
    }

    offsetV   = sumV / (float)samples;
    offsetU   = sumU / (float)samples;
    offsetBus = sumBus / (float)samples;

    phaseOk = BSP_ADC_OffsetInRange(offsetU, BOARD_ADC_OFFSET_MIN, BOARD_ADC_OFFSET_MAX) &&
              BSP_ADC_OffsetInRange(offsetV, BOARD_ADC_OFFSET_MIN, BOARD_ADC_OFFSET_MAX);
    busOk   = BSP_ADC_OffsetInRange(offsetBus, BOARD_ADC_OFFSET_BUS_MIN, BOARD_ADC_OFFSET_BUS_MAX);

    if (phaseOk == 0U)
    {
        return HAL_ERROR;
    }

    if (busOk == 0U)
    {
        /* GIM6010 IBUS 零位不一定在半量程，仍保留实测值供诊断 */
        offsetBus = BOARD_ADC_OFFSET_NOMINAL;
    }

    return HAL_OK;
}

void BSP_ADC_StartFocIrq(ADC_FocCallbackTypeDef callback)
{
    focCallback = callback;

    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOS);
    __HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_JEOS);

    HAL_NVIC_SetPriority(ADC1_2_IRQn, BOARD_IRQPRIO_FOC, 0);
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
}

void BSP_ADC_GetOffsets(float *offsU, float *offsV, float *offsBus)
{
    *offsU   = offsetU;
    *offsV   = offsetV;
    *offsBus = offsetBus;
}

void BSP_ADC_SetOffsets(float offsU, float offsV, float offsBus)
{
    offsetU   = offsU;
    offsetV   = offsV;
    offsetBus = offsBus;
}

float BSP_ADC_GetVbus(void)
{
    return vbusFilt;
}

float BSP_ADC_ReadMosTemp(void)
{
    float raw = BSP_ADC_ReadRegularRank(&hadc1, BOARD_ADC_NTC_REGULAR_RANK_MOS);
    return BSP_ADC_NtcToTemp((uint16_t)raw, BOARD_NTC_MOS_R25, BOARD_NTC_MOS_BETA);
}

float BSP_ADC_ReadMotorTemp(void)
{
    float raw = BSP_ADC_ReadRegularRank(&hadc1, BOARD_ADC_NTC_REGULAR_RANK_MOT);
    return BSP_ADC_NtcToTemp((uint16_t)raw, BOARD_NTC_MOT_R25, BOARD_NTC_MOT_BETA);
}

/**
  * @brief  ADC1/ADC2 注入完成中断服务
  */
void BSP_ADC_IRQHandler(void)
{
    ADC_TypeDef *adc1 = hadc1.Instance;
    ADC_TypeDef *adc2 = hadc2.Instance;

    if ((adc1->ISR & ADC_ISR_JEOS) != 0U)
    {
        ADC_FocMeasTypeDef meas;
        float vbusRaw;
        uint32_t rawV;
        uint32_t rawW;

        adc1->ISR = ADC_ISR_JEOS;

        rawV = adc1->JDR1;
        rawW = adc1->JDR2;

        meas.CurSat = ((rawW <= BOARD_ADC_RAIL_LOW) || (rawW >= BOARD_ADC_RAIL_HIGH) ||
                       (rawV <= BOARD_ADC_RAIL_LOW) || (rawV >= BOARD_ADC_RAIL_HIGH)) ? 1U : 0U;

        meas.Iu   = ((float)rawW - offsetU)         * BOARD_CUR_LSB * BOARD_CUR_SIGN_U;
        meas.Iv   = ((float)rawV - offsetV)         * BOARD_CUR_LSB * BOARD_CUR_SIGN_V;
        meas.Ibus = ((float)adc2->JDR1 - offsetBus) * BOARD_CUR_LSB * BOARD_CUR_SIGN_BUS;

        vbusRaw  = (float)adc1->JDR3 * BOARD_VBUS_LSB;
        vbusFilt += BOARD_VBUS_LPF_ALPHA * (vbusRaw - vbusFilt);
        meas.Vbus = vbusFilt;

        if (focCallback != NULL)
        {
            focCallback(&meas);
        }
    }
}

/* Private functions ---------------------------------------------------------*/

static float BSP_ADC_ReadRegularRank(ADC_HandleTypeDef *hadc, uint32_t rank)
{
    float result = 0.0f;
    uint32_t i;

    if (HAL_ADC_Start(hadc) == HAL_OK)
    {
        for (i = 0U; i < rank; i++)
        {
            if (HAL_ADC_PollForConversion(hadc, 2U) != HAL_OK)
            {
                return 0.0f;
            }
            result = (float)HAL_ADC_GetValue(hadc);
        }
    }
    return result;
}

static float BSP_ADC_NtcToTemp(uint16_t raw, float r25, float beta)
{
    float ratio;
    float rntc;
    float tempK;

    if ((raw < 8U) || (raw > 4088U))
    {
        return BOARD_NTC_FAULT_TEMP;
    }

    ratio = (float)raw / 4096.0f;
    rntc  = BOARD_NTC_PULLUP * ratio / (1.0f - ratio);
    tempK = 1.0f / ((1.0f / 298.15f) + logf(rntc / r25) / beta);

    return tempK - 273.15f;
}
