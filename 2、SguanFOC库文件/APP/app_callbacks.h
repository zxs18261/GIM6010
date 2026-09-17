/**
  ******************************************************************************
  * @file    app_callbacks.h
  * @brief   GIM6010 HAL 回调聚合层（旧工程 Hardware/Timer.c 角色，DOC §11.3）
  * @note    统一实现 HAL 中断回调，覆盖 CubeMX 弱符号：
 *          - TIM2 (1kHz)          → ADC 注入心跳监视 + SguanFOC_Low_Loop
 *          - ADC1 注入完成 (20kHz) → SguanFOC_High_Loop + VBUS 快速采样 + 连续超期关断
 *          - ADC 错误/注入队列溢出 → 立即关 TIM1 MOE 并锁 SENSOR_ERROR
  *          - USART1 idle          → 入队；主循环/校准等待调用 APP_UartCommandPoll 解析
  *          - FDCAN1 RX FIFO0      → CANFD_OnRxIsr（校验后入队，主循环解析）
  ******************************************************************************
  */

#ifndef __APP_CALLBACKS_H
#define __APP_CALLBACKS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ADC1 注入组锁存值（UserData_Function.h 的 User_ReadADC_Raw 读取）
 * [0]=Rank1 IV(PB1)  [1]=Rank2 IW(PB0)  [2]=Rank3 VBUS(PB11，喂 BSP_ADC 快速通道) */
extern volatile uint32_t ADC_InjectedValues[4];

/* 20kHz 高环诊断计数（VOFA Ch6 / 调试器观测） */
extern volatile uint32_t g_high_loop_cnt;
extern volatile uint32_t g_high_loop_last_cycles;
extern volatile uint32_t g_high_loop_max_cycles;
extern volatile uint32_t g_high_loop_deadline_miss_cnt;

/* 首次硬件故障发生时冻结的电流环现场；valid 最后置位，供 VOFA 在清零后复盘。 */
typedef struct
{
    uint32_t raw_iv;
    uint32_t raw_iw;
    float offset0;
    float offset1;
    float current0;
    float current1;
    float id;
    float iq;
    float ud;
    float uq;
    float pid_d_out;
    float pid_q_out;
    float target_iq;
    uint8_t status;
    uint8_t valid;
} APP_FocFaultSnapshot_t;

extern volatile APP_FocFaultSnapshot_t g_foc_fault_snapshot;
extern volatile uint32_t g_uart_rx_drop_cnt;
extern volatile uint32_t g_uart_rx_error_cnt;
extern volatile uint32_t g_uart_rx_restart_fail_cnt;

/* 【临时诊断，定位后移除】User_InitialInit 失败阶段码（见 UserData_Function.h） */
extern volatile uint8_t g_foc_init_stage;

uint8_t APP_UartRxStart(void);
uint8_t APP_UartStopPending(void);
void APP_UartCommandPoll(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_CALLBACKS_H */
