/**
  ******************************************************************************
  * @file    bsp_vofa.c
  * @brief   VOFA+ JustFloat 帧打包与 UART 非阻塞发送
  * @note    帧格式：[float0]..[floatN-1][0x00 0x00 0x80 0x7F]；经 BSP_UART_Write
  *          与 Shell 文本共用 USART1，VOFA 开启时避免同时 log/enclog 刷屏。
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 Z.Xusheng
  * SPDX-License-Identifier: MIT
  ******************************************************************************
  */

#include "bsp_vofa.h"
#include "bsp_uart.h"
#include "board_config.h"

#include <string.h>

/*
 * 作用：打包 JustFloat 并通过调试 UART 发出一帧。
 * 逻辑：校验通道数 → 拷贝 float 小端 → 追加帧尾 → BSP_UART_Write；
 *       TX 环缓冲空间不足则整帧丢弃（不阻塞电流环/主循环）。
 */
bool BSP_VOFA_SendJustFloat(const float *channels, uint8_t channel_count)
{
    uint8_t  frame[(BOARD_TELEM_VOFA_CH_COUNT * 4U) + 4U];
    uint32_t frame_len;
    uint32_t idx;

    if ((channels == NULL) || (channel_count == 0U) ||
        (channel_count > BOARD_TELEM_VOFA_CH_COUNT))
    {
        return false;
    }

    frame_len = ((uint32_t)channel_count * 4U) + 4U;
    if (BSP_UART_TxFree() < frame_len)
    {
        return false;
    }

    memcpy(frame, channels, (size_t)channel_count * sizeof(float));
    idx = (uint32_t)channel_count * 4U;
    frame[idx + 0U] = BSP_VOFA_JUSTFLOAT_TAIL0;
    frame[idx + 1U] = BSP_VOFA_JUSTFLOAT_TAIL1;
    frame[idx + 2U] = BSP_VOFA_JUSTFLOAT_TAIL2;
    frame[idx + 3U] = BSP_VOFA_JUSTFLOAT_TAIL3;

    return (BSP_UART_Write(frame, frame_len) == frame_len);
}
