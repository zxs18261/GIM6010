/**
  ******************************************************************************
  * @file    board_config.h
  * @brief   板级硬件参数统一配置（GIM6010 电机控制器 V0.0 板）
  * @note    本文件集中定义与电路板绑定的全部硬件参数与引脚/通道映射，
  *          依据为网表 Netlist_Schematic1_1_2026-07-11.tel 与原理图 V1.0：
  *          - TIM1 CH1/CH2/CH3 = U/V/W 相（FD6288Q 同相驱动）；
  *          - 电流采样：V/W 两相低侧腿独立 5mR + INA181 差分放大 20 倍，
  *            母线回流 5mR + INA181×20；U 相电流由 Iu = -(Iv + Iw) 推算；
  *          - MT6816 磁编码器 SPI3（PA15=CSN / PB3=SCK / PB4=MISO / PB5=MOSI）；
  *          - CAN 120Ω 终端电阻 R96 固定焊接，无 EN_120 切换 GPIO；
  *          - 调试串口 USART1 PB6/PB7；WS2812 RGB 接 PA12（TIM4_CH2）。
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

#ifndef __BOARD_CONFIG_H
#define __BOARD_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================== 时钟 ====================================== */
#define BOARD_HSE_FREQ_HZ           (8000000UL)     /*!< 板载晶振 8MHz（CubeMX HSE_VALUE） */
#define BOARD_SYSCLK_HZ             (170000000UL)   /*!< 系统时钟 8/2*85/2 = 170MHz */
#define BOARD_TIM1_CLK_HZ           (170000000UL)   /*!< TIM1 内核时钟（APB2 x1） */

/* ============================== PWM ======================================= */
#define BOARD_PWM_FREQ_HZ           (20000UL)       /*!< PWM/电流环频率 */
#define BOARD_PWM_ARR               ((uint32_t)(BOARD_TIM1_CLK_HZ / (2UL * BOARD_PWM_FREQ_HZ)))
#define BOARD_PWM_DEADTIME_DTG      (34U)
#define BOARD_PWM_MAX_DUTY          (0.94f)
#define BOARD_PWM_ADC_TRIG_ADVANCE  (40U)
#define BOARD_PWM_CCR_U             CCR1
#define BOARD_PWM_CCR_V             CCR2
#define BOARD_PWM_CCR_W             CCR3

/* ============================ 电流采样 ==================================== */
#define BOARD_SHUNT_OHM             (0.005f)        /*!< 低侧采样电阻 5mΩ */
#define BOARD_CSA_GAIN              (20.0f)         /*!< INA181A2 增益 ×20 */
#define BOARD_ADC_VREF              (3.3f)
/** 电流分辨率 [A/LSB] ≈ 8.06mA */
#define BOARD_CUR_LSB               (BOARD_ADC_VREF / 4096.0f / (BOARD_CSA_GAIN * BOARD_SHUNT_OHM))
#define BOARD_CUR_SIGN_U            (-1.0f)         /*!< meas.Iu 槽映射 W 相采样 */
#define BOARD_CUR_SIGN_V            (-1.0f)         /*!< meas.Iv 槽映射 V 相采样 */
#define BOARD_CUR_SIGN_BUS          (1.0f)
/** 半量程偏置时约 ±16.5A，软件峰值限幅见 motor_config.h */
#define BOARD_CUR_MEAS_RANGE        (16.5f)

/** ADC 注入通道映射（网表确认）：
    IV   = U17 OUT = PB1 = ADC1_IN12
    VBUS = 30k/3k 分压 = PB11 = ADC1_IN14
    IW   = U19 OUT = PB0 = ADC1_IN15
    IBUS = U20 OUT = PB2 = ADC2_IN12 */
#define BOARD_ADC1_JCH_IV           ADC_CHANNEL_12
#define BOARD_ADC1_JCH_VBUS         ADC_CHANNEL_14
#define BOARD_ADC1_JCH_IW           ADC_CHANNEL_15
#define BOARD_ADC2_JCH_IBUS         ADC_CHANNEL_12
/** 规则组：MOS NTC = PA2 = ADC1_IN3，电机 NTC = PA3 = ADC1_IN4（同 ADC1 双 rank） */
#define BOARD_ADC1_RCH_MOSNTC       ADC_CHANNEL_3
#define BOARD_ADC1_RCH_MOTNTC       ADC_CHANNEL_4
#define BOARD_ADC_NTC_REGULAR_RANK_MOS  (1U)
#define BOARD_ADC_NTC_REGULAR_RANK_MOT  (2U)

#define BOARD_ADC_CUR_SMPTIME       ADC_SAMPLETIME_6CYCLES_5
#define BOARD_ADC_VBUS_SMPTIME      ADC_SAMPLETIME_24CYCLES_5
#define BOARD_ADC_NTC_SMPTIME       ADC_SAMPLETIME_247CYCLES_5

#define BOARD_ADC_OFFSET_MIN        (300.0f)
#define BOARD_ADC_OFFSET_MAX        (3900.0f)
/** IBUS 零电流时可能接近 0V 而非半量程，单独放宽校验窗口 */
#define BOARD_ADC_OFFSET_BUS_MIN    (0.0f)
#define BOARD_ADC_OFFSET_BUS_MAX    (4095.0f)
#define BOARD_ADC_OFFSET_NOMINAL    (2048.0f)
#define BOARD_ADC_RAIL_LOW          (20U)
#define BOARD_ADC_RAIL_HIGH         (4075U)

/* ============================ 母线电压 ==================================== */
#define BOARD_VBUS_R_TOP            (30.0f)         /*!< 分压上电阻 30kΩ */
#define BOARD_VBUS_R_BOT            (3.0f)          /*!< 分压下电阻 3kΩ */
#define BOARD_VBUS_LSB              (BOARD_ADC_VREF / 4096.0f * ((BOARD_VBUS_R_TOP + BOARD_VBUS_R_BOT) / BOARD_VBUS_R_BOT))
#define BOARD_VBUS_LPF_ALPHA        (0.05f)

/* ============================== NTC ======================================= */
#define BOARD_NTC_PULLUP            (10000.0f)
#define BOARD_NTC_MOS_R25           (10000.0f)
#define BOARD_NTC_MOS_BETA          (3950.0f)
#define BOARD_NTC_MOT_R25           (10000.0f)
#define BOARD_NTC_MOT_BETA          (3950.0f)
#define BOARD_NTC_FAULT_TEMP        (250.0f)

/* ============================ 调试串口 ==================================== */
#define BOARD_UART_BAUDRATE         (921600UL)

/* ============================== CAN ======================================= */
#define BOARD_CAN_TERM_GPIO_ENABLE  (0)             /*!< 0 = 120Ω 固定焊接，无 GPIO 切换 */
#define BOARD_CAN_TERM_ACTIVE_HIGH  (1)
#define BOARD_CAN_BROADCAST_ID      (0x000U)
#define BOARD_CAN_NODE_ID_MAX       (63U)
#define BOARD_CAN_CFG_ID_BASE       (0x600U)
#define BOARD_CAN_CFG_REPLY_BASE    (0x680U)
#define BOARD_CAN_MASTER_ID         (0x000U)

/* ============================ WS2812 灯 =================================== */
#define BOARD_WS2812_ARR            ((uint32_t)(BOARD_SYSCLK_HZ / 800000UL - 1UL))
#define BOARD_WS2812_CCR_0          (58U)
#define BOARD_WS2812_CCR_1          (136U)
#define BOARD_WS2812_ORDER_RGB      (0)
#define BOARD_LED_BRIGHTNESS_DEFAULT (40U)

/* ========================= 编码器（MT6816 SPI3） ========================== */
#define BOARD_ENC_SPI_PRESCALER     SPI_BAUDRATEPRESCALER_32   /*!< ~5.3MHz @ 170MHz SPI 内核 */

/* =========================== 中断优先级 =================================== */
#define BOARD_IRQPRIO_FOC           (1U)
#define BOARD_IRQPRIO_CAN           (3U)
#define BOARD_IRQPRIO_UART          (6U)
#define BOARD_IRQPRIO_UART_DMA      (7U)
#define BOARD_IRQPRIO_LED_DMA       (8U)

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_CONFIG_H */
