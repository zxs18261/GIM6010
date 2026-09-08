#ifndef __USERDATA_FUNCTION_H
#define __USERDATA_FUNCTION_H
/* 电机控制User用户设置·功能接口 */
/* 用户自己的CODE BEGIN Includes */
#include "main.h"
#include "usart.h"
#include "tim.h"
#include "spi.h"
#include "adc.h"
#include "BSP_MT6816.h"
#include "BSP_ADC.h"
/* 用户自己的CODE END Includes */

static inline void User_InitialInit(void){
    /* 编码器初始化(片选置高) */
    MT6816_Init();

    /* 启动 TIM1 互补 PWM CH1/2/3(高侧) + CH1N/2N/3N(低侧) */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

    /* CH4 用于 OC4REF 触发 ADC 注入, 采样点放周期中部(ARR-40) */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 4249u - 40u);

    /* 上电先输出 50% 零占空安全(中心对齐 50% = ARR/2) */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 4249u/2u);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 4249u/2u);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 4249u/2u);

    /* 使能 TIM1 更新中断(High_Loop 挂载点) */
    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);

    /* 启动 ADC 注入组(IV/IW/VBUS, TIM1_CC4 触发) */
    BSP_ADC_Init();
}

static inline void User_Delay(unsigned int ms){
    HAL_Delay(ms);
}

static inline signed int User_ReadADC_Raw(unsigned char Current_CH){
    signed int ADC_num = 0;
    switch (Current_CH){
    case 0:  /* B相 = IV (JDR1) */
        ADC_num = (signed int)BSP_ADC_GetIV();
        break;
    case 1:  /* C相 = IW (JDR2) */
        ADC_num = (signed int)BSP_ADC_GetIW();
        break;
    default:
        break;
    }
    return ADC_num;
}

static inline float User_Encoder_ReadRad(void){
    return MT6816_GetRad();   /* 0~2π 单圈机械角 */
}

static inline void User_PwmDuty_Set(unsigned short int Duty_u,
                                    unsigned short int Duty_v,
                                    unsigned short int Duty_w){
    /* 硬件相位映射: TIM1_CH1=PA8(W), CH2=PA9(V), CH3=PA10(U) */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Duty_w);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, Duty_v);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, Duty_u);
}

static inline float User_VBUS_DataGet(void){
    return BSP_ADC_GetVBUS();
}

static inline float User_Temperature_DataGet(void){
    return BSP_ADC_GetTempDriver();   /* 驱动器温度(板载NTC); -9999=禁用 */
}

/* ================= 驱动代码(驱动层) ================= */
static inline void User_CorrespondSet(unsigned char *ch, unsigned short int size){
    HAL_UART_Transmit(&huart1, (uint8_t *)ch, size, 0xFFFF);
}

#endif // USERDATA_FUNCTION_H
