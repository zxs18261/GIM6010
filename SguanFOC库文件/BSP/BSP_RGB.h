/**
  ******************************************************************************
  * @file    BSP_RGB.h
  * @brief   GIM6010 板载 WS2812 RGB 灯 BSP（TIM4_CH2 / PA12，单灯）
  * @note    纯驱动层：不读取电机状态。在 UserData_Status.h 各
  *          MOTOR_STATUS_xxx_Loop() 中调用本模块 API 定制灯语。
  *          主循环须周期性调用 BSP_RGB_Poll() 以完成 DMA 刷新。
  ******************************************************************************
  */

#ifndef __BSP_RGB_H
#define __BSP_RGB_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ============================ 硬件与时序 ================================== */
/** @note 系统时钟 170MHz，TIM4 专用于 WS2812，800kHz bit 时序 */
#define BSP_RGB_WS2812_ARR              (211U)
#define BSP_RGB_WS2812_CCR_0            (58U)
#define BSP_RGB_WS2812_CCR_1            (136U)
#define BSP_RGB_ORDER_RGB               (0U)    /*!< 0=GRB(WS2812)，1=RGB */

/** 默认全局亮度（0~255，运行时有效，不掉电保存） */
#define BSP_RGB_DEFAULT_BRIGHTNESS      (40U)

/* ============================ 常用颜色 ================================== */
#define BSP_RGB_COLOR_OFF               (0x000000UL)
#define BSP_RGB_COLOR_RED               (0xFF0000UL)
#define BSP_RGB_COLOR_GREEN             (0x00FF00UL)
#define BSP_RGB_COLOR_BLUE              (0x0000FFUL)
#define BSP_RGB_COLOR_CYAN              (0x00FFFFUL)
#define BSP_RGB_COLOR_YELLOW            (0xFFFF00UL)
#define BSP_RGB_COLOR_ORANGE            (0xFFA000UL)
#define BSP_RGB_COLOR_WHITE             (0xFFFFFFUL)

/* ============================ 驱动 API ================================== */

void    BSP_RGB_Init(void);
void    BSP_RGB_Poll(void);
void    BSP_RGB_SetColor(uint8_t r, uint8_t g, uint8_t b);
void    BSP_RGB_SetColorRGB(uint32_t rgb);
void    BSP_RGB_SetBrightness(uint8_t brightness);
uint8_t BSP_RGB_GetBrightness(void);
void    BSP_RGB_Off(void);

/** DMA1_Channel3 中断入口（由 stm32g4xx_it.c 转发） */
void    BSP_RGB_DmaIRQHandler(void);

/* ============================ 灯效用辅助（无状态机） ==================== */

/**
  * @brief  三角波呼吸亮度（16~255）
  * @param  tick      通常为 HAL_GetTick()
  * @param  periodMs  完整呼吸周期 [ms]
  */
uint8_t BSP_RGB_BreathLevel(uint32_t tick, uint32_t periodMs);

/**
  * @brief  方波闪烁
  * @param  tick           HAL_GetTick()
  * @param  halfPeriodMs   半周期 [ms]
  * @retval 1=亮段，0=灭段
  */
uint8_t BSP_RGB_BlinkOn(uint32_t tick, uint32_t halfPeriodMs);

/**
  * @brief  故障计数闪烁（N 次亮灭 + 组间停顿）
  * @param  blinkCount    闪烁次数（通常填故障码）
  * @param  halfPeriodMs  单次亮/灭半周期 [ms]
  * @param  pauseMs       每组结束后的熄灭间隔 [ms]
  * @retval 1=亮段，0=灭段
  */
uint8_t BSP_RGB_FaultBlinkOn(uint32_t tick, uint8_t blinkCount,
                             uint32_t halfPeriodMs, uint32_t pauseMs);

/**
  * @brief  按 level(0~255) 缩放 RGB 后输出（不改变全局亮度寄存器）
  */
void    BSP_RGB_SetColorLevel(uint8_t r, uint8_t g, uint8_t b, uint8_t level);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_RGB_H */
