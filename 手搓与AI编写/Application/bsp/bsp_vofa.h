/**
  ******************************************************************************
  * @file    bsp_vofa.h
  * @brief   VOFA+ JustFloat 协议打包（经 bsp_uart DMA 环缓冲发送）
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 Z.Xusheng
  * SPDX-License-Identifier: MIT
  ******************************************************************************
  */

#ifndef __BSP_VOFA_H
#define __BSP_VOFA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#define BSP_VOFA_JUSTFLOAT_TAIL0    (0x00U)
#define BSP_VOFA_JUSTFLOAT_TAIL1    (0x00U)
#define BSP_VOFA_JUSTFLOAT_TAIL2    (0x80U)
#define BSP_VOFA_JUSTFLOAT_TAIL3    (0x7FU)

bool BSP_VOFA_SendJustFloat(const float *channels, uint8_t channel_count);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_VOFA_H */
