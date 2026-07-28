/**
  ******************************************************************************
  * @file    app_telemetry.h
  * @brief   VOFA+ 12 路 JustFloat 遥测（1 kHz，调试 FOC 三环）
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 Z.Xusheng
  * SPDX-License-Identifier: MIT
  ******************************************************************************
  */

#ifndef __APP_TELEMETRY_H
#define __APP_TELEMETRY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void APP_TELEM_Init(void);
void APP_TELEM_Poll(void);
void APP_TELEM_SetEnabled(uint8_t enable, uint32_t rateHz);
uint8_t APP_TELEM_IsEnabled(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_TELEMETRY_H */
