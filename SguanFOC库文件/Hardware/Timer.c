/*
 * @Author: 星必尘Sguan
 * @Date: 2025-05-08 19:26:48
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-08-02
 * @FilePath: \stm_SguanFOCtest\Hardware\Timer.c
 * @Description: TIM2=1kHz：Low_Loop + High_Loop（勿在 TIM1 20kHz 里跑 High_Loop+SPI，会饿死主循环）
 *
 * Copyright (c) 2025 by $JUST, All Rights Reserved.
 */
#include "Timer.h"
#include "adc.h"
#include "SguanFOC.h"
#include "main.h"

extern UART_HandleTypeDef huart1;
extern ADC_HandleTypeDef hadc1;
volatile uint32_t ADC_InjectedValues[4] = {0};

volatile uint32_t g_high_loop_cnt = 0;
volatile float g_vofa_last_AO = 0.0f;
volatile float g_vofa_last_BO = 0.0f;
volatile float g_vofa_last_CO = 0.0f;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        /* 取最近注入采样，1kHz 跑控制环（开环足够；主循环/RGB/串口可运行） */
        ADC_InjectedValues[0] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
        ADC_InjectedValues[1] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
        g_high_loop_cnt++;
        SguanFOC_High_Loop();
        SguanFOC_Low_Loop();
    }
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc){
    if (hadc->Instance == ADC1) {
        ADC_InjectedValues[0] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
        ADC_InjectedValues[1] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART1) {
        SguanFOC_Printf_Loop(Sguan_PrintfBuff, Size);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, Sguan_PrintfBuff, sizeof(Sguan_PrintfBuff));
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    }
}
