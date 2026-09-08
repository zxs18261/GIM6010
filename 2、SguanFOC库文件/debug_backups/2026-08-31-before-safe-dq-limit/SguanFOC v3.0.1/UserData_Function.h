#ifndef __USERDATA_FUNCTION_H
#define __USERDATA_FUNCTION_H
/* 电机控制User用户设置·功能接口
 * GIM6010 HAL 绑定层：PWM/ADC/编码器/通信 与 SguanFOC 库的对接（DOC/GIM6010.md §11.1）
 */
/* 用户自己的CODE BEGIN Includes */
#include "SguanFOC.h"     /* 对齐限流需要 Sguan.status / MOTOR_STATUS_*（本文件也被 Sguan_printf.c 包含） */
#include "main.h"
#include "tim.h"
#include "adc.h"
#include "usart.h"
#include "stm32g4xx_ll_adc.h"
#include "app_callbacks.h"
#include "BSP_MT6816.h"
#include "BSP_ADC.h"
/* 用户自己的CODE END Includes */

static inline void User_InitialInit(void){
    /* 校准要求 ADC 处于关闭态；待命态 BSP_ADC 的 LL 读取可能已将其使能 */
    if (LL_ADC_IsEnabled(hadc1.Instance) != 0UL) {
        LL_ADC_Disable(hadc1.Instance);
    }
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

    /* 强制打开主输出（功率级使能） */
    __HAL_TIM_MOE_ENABLE(&htim1);

    BSP_MT6816_Init();
    BSP_ADC_Init();

    /* BSP 初始化完成后才允许 ADC 回调请求 MT6816 DMA 采样。 */
    (void)HAL_ADCEx_InjectedStart_IT(&hadc1);
}

static inline void User_Delay(unsigned int ms){
    uint32_t tickstart = HAL_GetTick();

    /* 校准/对齐延时期间以 1kHz 阻塞直读编码器，保证零位采样新鲜（DMA 链已停用） */
    while ((uint32_t)(HAL_GetTick() - tickstart) < ms) {
        BSP_MT6816_PollBlocking();
    }
}

static inline signed int User_ReadADC_Raw(unsigned char Current_CH){
    signed int ADC_num = 0;
    /* 2026-08-31 映射试验#1：电流通道 V/W 互换（原直连时 AO=0.3 电流环正反馈爆）
     * 库 BC 模式要求 ch0=B(V)相、ch1=C(W)相；此试验反向喂入。
     * 若试验后 AO=0.3 稳定 → 丝印/走线与假设相反；若仍爆 → 还原此改动，改试 PWM 侧 */
    switch (Current_CH){
    case 0:
        ADC_num = (signed int)ADC_InjectedValues[1];   /* Rank2 = ADC_IW (PB0) */
        break;
    case 1:
        ADC_num = (signed int)ADC_InjectedValues[0];   /* Rank1 = ADC_IV (PB1) */
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
    /* 校准态对齐电压限流：库内固定 0.3×VBUS，此处按 USER_ALIGN_UD_RATIO 缩放占空比
     * （围绕零电压中心 ARR/2 线性缩放，不改算法文件即可限制对齐电流，见 UserData_Config.h） */
    if (Sguan.status == MOTOR_STATUS_CALIBRATING) {
        const float scale = USER_ALIGN_UD_RATIO / USER_ALIGN_UD_LIB_RATIO;
        const float center = (float)USER_TIM1_PWM_ARR * 0.5f;
        Duty_u = (unsigned short int)(center + ((float)Duty_u - center) * scale);
        Duty_v = (unsigned short int)(center + ((float)Duty_v - center) * scale);
        Duty_w = (unsigned short int)(center + ((float)Duty_w - center) * scale);
    }
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Duty_w);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, Duty_v);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, Duty_u);
}

static inline void User_PwmOutputDisable(void){
    /* 故障或显式停机时关断 TIM1 主输出，让三相桥高阻而不只是写零矢量。 */
    __HAL_TIM_MOE_DISABLE(&htim1);
}

static inline void User_PwmOutputEnable(void){
    /* 仅在明确的非零运动命令或校准阶段打开功率桥。 */
    __HAL_TIM_MOE_ENABLE(&htim1);
}

static inline float User_VBUS_DataGet(void){
    /* 20kHz 注入Rank3 快速值 + 100Hz 规则组后备（DOC §5.2） */
    return BSP_ADC_GetVBUS();
}

static inline float User_Temperature_DataGet(void){
    /* NTC 3950/10k，两路取最大，100℃ 过温阈值见 UserData_Motor.h */
    return BSP_ADC_GetTemp();
}

/* ================= 驱动代码(驱动层) ================= */
static inline void User_CorrespondSet(unsigned char *ch, unsigned short int size){
    /* 原版库 Printf_Normal_Loop 每个主循环都调用发送；此处做 500Hz 限速 +
     * DMA 忙丢弃，保证 JustFloat 帧间隔稳定且不阻塞主循环。 */
    static uint32_t s_last_tx_tick = 0U;

    if (huart1.gState != HAL_UART_STATE_READY) {
        return;   /* 上一帧 DMA 未完成，丢弃本轮 */
    }
    uint32_t now = HAL_GetTick();
    if ((now - s_last_tx_tick) < USER_VOFA_TX_PERIOD_MS) {
        return;
    }
    s_last_tx_tick = now;
    (void)HAL_UART_Transmit_DMA(&huart1, ch, size);
}


#endif // __USERDATA_FUNCTION_H
