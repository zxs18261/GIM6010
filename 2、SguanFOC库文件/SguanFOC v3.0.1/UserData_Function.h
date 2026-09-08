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
#include "canfd.h"
#include <string.h>
/* 用户自己的CODE END Includes */

static inline uint8_t User_InitialInit(void){
    /* HAL_TIM_PWM_Start/HAL_TIMEx_PWMN_Start 会在每次启动时自动置 MOE，不能用于
     * 安全初始化。先无条件高阻，再在 MOE=0 下预装 CCR、启用通道和计数器。 */
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
    /* __HAL_TIM_DISABLE 在 CCER 仍有通道使能时不会清 CEN，故这里直接停表，
     * 避免 ADC 停止/校准期间 TIM1_CH4 继续产生注入触发。 */
    CLEAR_BIT(htim1.Instance->CR1, TIM_CR1_CEN);

    /* 重复 CO=1 时注入链可能仍在等待 TIM1 触发；先用 HAL 合法停止注入组，
     * 再停止可能由 BSP_ADC LL 路径遗留的规则组。任一步失败都保持 MOE=0/CEN=0。 */
    if (HAL_ADCEx_InjectedStop_IT(&hadc1) != HAL_OK) {
        return 0U;
    }
    if (HAL_ADC_Stop(&hadc1) != HAL_OK) {
        return 0U;
    }
    if ((LL_ADC_REG_IsConversionOngoing(hadc1.Instance) != 0UL) ||
        (LL_ADC_INJ_IsConversionOngoing(hadc1.Instance) != 0UL) ||
        (LL_ADC_IsEnabled(hadc1.Instance) != 0UL)) {
        return 0U;
    }
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) {
        return 0U;
    }

    /* TIM1 触发源、DMA 映射与 NVIC 优先级均由 CubeMX 统一管理。 */

    /* 三相先等占空比（零电压矢量）；CH4 在峰值前提前触发 ADC */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (USER_TIM1_PWM_ARR + 1U) / 2U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (USER_TIM1_PWM_ARR + 1U) / 2U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (USER_TIM1_PWM_ARR + 1U) / 2U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, USER_TIM1_PWM_ARR - 40U);

    /* 直接建立 CCx/CCxN 与计数器运行态，不触碰 BDTR.MOE。UG 先把预装载值
     * 搬入活动寄存器；完成电源/温度/编码器预检后才显式打开功率级。 */
    htim1.Instance->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC1NE |
                             TIM_CCER_CC2E | TIM_CCER_CC2NE |
                             TIM_CCER_CC3E | TIM_CCER_CC3NE |
                             TIM_CCER_CC4E);
    htim1.Instance->EGR = TIM_EGR_UG;

    BSP_MT6816_Init();
    BSP_ADC_Init();

    /* 先确认注入组已成功布防，再启动 TIM1 触发；启动失败时计数器和功率桥
     * 都保持关闭，由调用方锁存 SENSOR_ERROR。 */
    if (HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK) {
        return 0U;
    }
    __HAL_TIM_ENABLE(&htim1);
    return 1U;
}

static inline void User_Delay(unsigned int ms){
    uint32_t tickstart = HAL_GetTick();

    /* 校准/对齐延时期间以 1kHz 阻塞直读编码器，保证零位采样新鲜（DMA 链已停用） */
    while ((uint32_t)(HAL_GetTick() - tickstart) < ms) {
        APP_UartCommandPoll();  /* 允许 CO=0 在阻塞对齐阶段及时取消。 */
        CANFD_BlockingSafetyPoll(); /* CAN DIS 同样必须在对齐通电期间立即生效。 */
        if ((Sguan.status == MOTOR_STATUS_STANDBY) ||
            (Sguan.status >= MOTOR_STATUS_OVERVOLTAGE)) return;
        BSP_MT6816_PollBlocking();
    }
}

static inline signed int User_ReadADC_Raw(unsigned char Current_CH){
    signed int ADC_num = 0;
    /* ADC 注入组硬件直连：Rank1=IV(B/V相)，Rank2=IW(C/W相)。
     * 电流极性只由 Current_Dir0/1 管理，不在这里混用通道交换。 */
    switch (Current_CH){
    case 0:
        ADC_num = (signed int)ADC_InjectedValues[0];   /* Rank1 = ADC_IV (PB1) */
        break;
    case 1:
        ADC_num = (signed int)ADC_InjectedValues[1];   /* Rank2 = ADC_IW (PB0) */
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
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
}

static inline void User_PwmOutputEnable(void){
    uint32_t primask;

    /* 仅在明确的有效运动命令（位置零目标也有效）或校准阶段打开功率桥。检查急停锁存与
     * MOE 置位必须是一个原子边沿；临界区结束后到达的急停 ISR 会立即关 MOE。 */
    primask = __get_PRIMASK();
    __disable_irq();
    if ((CANFD_IsStopPending() == 0U) && (APP_UartStopPending() == 0U) &&
        ((Sguan.status == MOTOR_STATUS_CALIBRATING) ||
         ((Sguan.status > MOTOR_STATUS_IDLE) &&
          (Sguan.status < MOTOR_STATUS_OVERVOLTAGE)))) {
        __HAL_TIM_MOE_ENABLE(&htim1);
    }
    if (primask == 0U) __enable_irq();
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
    static uint8_t s_tx_buffer[2][sizeof(PRINTF_STRUCT)];
    static uint8_t s_tx_index = 0U;
    HAL_StatusTypeDef tx_status;

    if ((ch == NULL) || (size == 0U) || (size > sizeof(s_tx_buffer[0])) ||
        (huart1.gState != HAL_UART_STATE_READY)) {
        return;   /* 上一帧 DMA 未完成，丢弃本轮 */
    }
    uint32_t now = HAL_GetTick();
    if ((now - s_last_tx_tick) < USER_VOFA_TX_PERIOD_MS) {
        return;
    }
    /* DMA 只能读取专用稳定缓冲，不能直接读取下一主循环会改写的 Sguan.TXdata。 */
    (void)memcpy(s_tx_buffer[s_tx_index], ch, size);
    tx_status = HAL_UART_Transmit_DMA(&huart1, s_tx_buffer[s_tx_index], size);
    if (tx_status == HAL_OK) {
        s_last_tx_tick = now;
        s_tx_index ^= 1U;
    }
}


#endif // __USERDATA_FUNCTION_H
