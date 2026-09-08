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
#include "BSP_ADC.h"
/* 用户自己的CODE END Includes */

static inline void User_InitialInit(void){
    (void)HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

    /* TIM1 触发源、DMA 映射与 NVIC 优先级均由 CubeMX 统一管理。 */

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    /* 三相先等占空比（零电压矢量）；CH4 在峰值前提前触发 ADC */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (USER_TIM1_PWM_ARR + 1U) / 2U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (USER_TIM1_PWM_ARR + 1U) / 2U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (USER_TIM1_PWM_ARR + 1U) / 2U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, USER_TIM1_PWM_ARR - 40U);

    /*
     * 双重关闭AOE：即使生成代码被旧配置覆盖，更新事件也不能自动恢复MOE。
     * 启动时保持功率级高阻；预定位或有效DQ指令到来后再显式打开MOE。
     */
    CLEAR_BIT(htim1.Instance->BDTR, TIM_BDTR_AOE);
    __HAL_TIM_MOE_DISABLE(&htim1);

    BSP_MT6816_Init();

    /* BSP 初始化完成后才允许 ADC 回调请求 MT6816 DMA 采样。 */
    (void)HAL_ADCEx_InjectedStart_IT(&hadc1);
}

static inline void User_Delay(unsigned int ms){
    uint32_t tickstart = HAL_GetTick();

    /* 校准阶段仍持续服务 MT6816 DMA；不会把 SPI 轮询带入 20 kHz 电流环。 */
    while ((uint32_t)(HAL_GetTick() - tickstart) < ms) {
        BSP_MT6816_Service();
        /* 阻塞式预定位期间继续发送TEXT事件和只读查询，不启动JustFloat。 */
        Printf_BackgroundService();
    }
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
    return BSP_MT6816_GetMechRadForControl();
}

static inline void User_PwmDuty_Set(unsigned short int Duty_u,
                                unsigned short int Duty_v,
                                unsigned short int Duty_w){
    /* 本板丝印：TIM1_CH1=W, CH2=V, CH3=U（见 main.h HIN_W/V/U） */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Duty_w);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, Duty_v);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, Duty_u);
}

static inline void User_PwmOutputEnable(void){
    /* TIM1 MOE 是三相桥统一的功率输出门控。 */
    __HAL_TIM_MOE_ENABLE(&htim1);
}

static inline void User_PwmOutputDisable(void){
    /* 关闭 MOE 让三相桥进入高阻，避免 Ud/Uq=0 时零矢量造成手拧制动。 */
    __HAL_TIM_MOE_DISABLE(&htim1);
}

static inline uint8_t User_PwmOutputIsEnabled(void){
    /* 直接读取TIM1 BDTR.MOE，VOFA显示的是硬件实际门控状态而不是软件期望。 */
    return ((htim1.Instance->BDTR & TIM_BDTR_MOE) != 0U) ? 1U : 0U;
}

static inline float User_VBUS_DataGet(void){
    /* 没有收到首个 ADC 样本时保留库原有的“无效值”语义。 */
    return (BSP_ADC_IsVbusValid() != 0U) ? BSP_ADC_GetVbus() : -9999.0f;
}

static inline float User_Temperature_DataGet(void){
    return -9999.0f;
}

static inline HAL_StatusTypeDef User_CorrespondSet(unsigned char *ch, unsigned short int size){
    if (huart1.gState != HAL_UART_STATE_READY)
    {
        return HAL_BUSY;
    }
    return HAL_UART_Transmit_DMA(&huart1, ch, size);
}


#endif // USERDATA_FUNCTION_H
