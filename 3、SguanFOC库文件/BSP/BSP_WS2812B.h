#ifndef __BSP_WS2812B_H
#define __BSP_WS2812B_H
/* GIM6010 板载 WS2812B 单灯驱动 (PA12, 位带式 bit-bang, 170MHz DWT 定时) */
#include "main.h"
#include <stdint.h>

void BSP_WS2812B_Init(void);
void BSP_WS2812B_SetColor(uint8_t r, uint8_t g, uint8_t b);  /* 0~255 */
void BSP_WS2812B_Off(void);

#endif
