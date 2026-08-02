#ifndef __USERDATA_FUNCTION_H
#define __USERDATA_FUNCTION_H
/* 电机控制User用户设置·功能接口 */
/* 用户自己的CODE BEGIN Includes */
#include "main.h"
#include "tim.h"
#include "adc.h"
#include "usart.h"
#include "Timer.h"
#include "BSP_MT6816.h"
/* 用户自己的CODE END Includes */

static inline void User_InitialInit(void){
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    (void)HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

    /* Cube 默认 TRGO2=RESET；改为 OC4REF，便于注入采样（控制环已由 TIM1 更新兜底） */
    sMasterConfig.MasterOutputTrigger  = TIM_TRGO_RESET;
    sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_OC4REF;
    sMasterConfig.MasterSlaveMode      = TIM_MASTERSLAVEMODE_DISABLE;
    (void)HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    /* 三相先等占空比（零电压矢量）；CH4 在峰值前提前触发 ADC */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 2125U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 2125U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 2125U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 4250U - 40U);

    /* 强制打开主输出（功率级使能） */
    __HAL_TIM_MOE_ENABLE(&htim1);

    (void)HAL_ADCEx_InjectedStart_IT(&hadc1);

    BSP_MT6816_Init();
}

static inline void User_Delay(unsigned int ms){
    HAL_Delay(ms);
}

static inline signed int User_ReadADC_Raw(unsigned char Current_CH){
    signed int ADC_num = 0;
    switch (Current_CH){
    case 0:
        ADC_num = (signed int)ADC_InjectedValues[0];
        break;
    case 1:
        ADC_num = (signed int)ADC_InjectedValues[1];
        break;
    default:
        break;
    }
    return ADC_num;
}

static inline float User_Encoder_ReadRad(void){
    return BSP_MT6816_GetMechRad();
}

static inline void User_PwmDuty_Set(unsigned short int Duty_u,
                                unsigned short int Duty_v,
                                unsigned short int Duty_w){
    /* 本板丝印：TIM1_CH1=W, CH2=V, CH3=U（见 main.h HIN_W/V/U） */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Duty_w);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, Duty_v);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, Duty_u);
}

static inline float User_VBUS_DataGet(void){
    return -9999.0f;
}

static inline float User_Temperature_DataGet(void){
    return -9999.0f;
}

static inline void User_CorrespondSet(unsigned char *ch, unsigned short int size){
    uint32_t tickstart = HAL_GetTick();
    while (huart1.gState != HAL_UART_STATE_READY)
    {
        if ((HAL_GetTick() - tickstart) > 20U)
        {
            return;
        }
    }
    (void)HAL_UART_Transmit_DMA(&huart1, ch, size);
}


#endif // USERDATA_FUNCTION_H
