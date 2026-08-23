/*
 * @Author: 星必尘Sguan
 * @Date: 2025-05-08 19:26:48
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-08-02
 * @FilePath: \stm_SguanFOCtest\Hardware\Timer.c
 * @Description: ADC1 注入完成=20kHz High_Loop；TIM2=1kHz Low_Loop；MT6816 只请求 DMA 缓存
 *
 * Copyright (c) 2025 by $JUST, All Rights Reserved.
 */
#include "Timer.h"
#include "adc.h"
#include "SguanFOC.h"
#include "Sguan_printf.h"
#include "BSP_MT6816.h"
#include "main.h"

extern UART_HandleTypeDef huart1;
extern ADC_HandleTypeDef hadc1;
volatile uint32_t ADC_InjectedValues[4] = {0};

volatile uint32_t g_high_loop_cnt = 0;
volatile uint32_t g_high_loop_last_cycles = 0U;
volatile uint32_t g_high_loop_max_cycles = 0U;
volatile uint32_t g_high_loop_deadline_miss_cnt = 0U;
volatile float g_vofa_last_AO = 0.0f;
volatile float g_vofa_last_BO = 0.0f;
volatile float g_vofa_last_CO = 0.0f;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        /* 低频状态机/保护留在 1 kHz，绝不在此重复读取 ADC 或运行 High_Loop。 */
        SguanFOC_Low_Loop();
    }
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc){
    if (hadc->Instance == ADC1) {
        uint32_t start_cycles = DWT->CYCCNT;
        uint32_t used_cycles;
        static uint32_t deadline_cycles = 0U;

        if (deadline_cycles == 0U) {
            deadline_cycles = SystemCoreClock / USER_FOC_CONTROL_HZ;
        }

        /* ADC 回调发生在 PWM 采样窗口；先锁存两个注入 Rank，再进入电流环。 */
        ADC_InjectedValues[0] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
        ADC_InjectedValues[1] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);

        g_high_loop_cnt++;
        if ((g_high_loop_cnt % USER_ENCODER_SAMPLE_DIV) == 0U) {
            /* 只递增请求序号；DMA 由主循环启动，20 kHz ISR 不访问 SPI3。 */
            BSP_MT6816_RequestSampleFromISR();
        }

        SguanFOC_High_Loop();

        used_cycles = DWT->CYCCNT - start_cycles;
        g_high_loop_last_cycles = used_cycles;
        if (used_cycles > g_high_loop_max_cycles) {
            g_high_loop_max_cycles = used_cycles;
        }
        if (used_cycles >= deadline_cycles) {
            g_high_loop_deadline_miss_cnt++;
        }
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        Printf_TxCpltISR();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    /* RX 错误不应抢占仍在发送的 TX DMA 缓冲。 */
    if ((huart->Instance == USART1) && (huart->gState == HAL_UART_STATE_READY)) {
        Printf_TxErrorISR();
    }
}

void HAL_UART_AbortTransmitCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        Printf_TxErrorISR();
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART1) {
        SguanFOC_Printf_Loop(Sguan_PrintfBuff, Size);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, Sguan_PrintfBuff, sizeof(Sguan_PrintfBuff));
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    }
}
