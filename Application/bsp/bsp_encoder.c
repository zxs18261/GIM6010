/**
  ******************************************************************************
  * @file    bsp_encoder.c
  * @brief   MT6816 磁编码器 BSP（SPI3 + GPIO 片选）
  * @note    - 网表：CSN=PA15 / SCK=PB3 / MISO=PB4 / MOSI=PB5（SPI3 硬件复用）；
  *          - 14bit 绝对角度，偶校验 + 无磁铁位检测；
  *          - CubeMX 的 SPI3 参数为占位值，本模块在 Init 中重配 Mode3/16bit；
  *          - 单次读取约 20us（两次 CS 事务），可在 20kHz 电流环 ISR 调用；
  *          - 方向反转为运行时设置（预定位校准写入，参数持久化）。
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

/* Includes ------------------------------------------------------------------*/
#include "bsp_encoder.h"
#include "board_config.h"
#include "spi.h"
#include "main.h"

/* Private defines -----------------------------------------------------------*/
#define ENC_CONSECUTIVE_ERR_MAX (0xFFU)
#define MT6816_REG_ANGLE_H      (0x03U)
#define MT6816_REG_ANGLE_L      (0x04U)
#define MT6816_SPI_TIMEOUT_MS   (5U)
#define MT6816_READ_RETRIES     (3U)
#define MT6816_NO_MAGNET_BIT    (0x0002U)

/* Private variables ---------------------------------------------------------*/
static uint8_t encInvert = 0U;

/* Private function prototypes -----------------------------------------------*/
static void              BSP_ENC_CsLow(void);
static void              BSP_ENC_CsHigh(void);
static uint16_t          BSP_ENC_BuildReadCmd(uint8_t reg);
static uint8_t           BSP_ENC_ParityEven(uint16_t data);
static HAL_StatusTypeDef BSP_ENC_SpiTransfer16(uint16_t tx, uint16_t *rx);
static HAL_StatusTypeDef BSP_ENC_ReadRawOnce(uint16_t *raw14);

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  初始化 MT6816 SPI 接口
  * @note   须在 MX_SPI3_Init 与 MX_GPIO_Init 之后调用。
  */
void BSP_ENC_Init(void)
{
    hspi3.Init.Mode                = SPI_MODE_MASTER;
    hspi3.Init.Direction           = SPI_DIRECTION_2LINES;
    hspi3.Init.DataSize            = SPI_DATASIZE_16BIT;
    hspi3.Init.CLKPolarity         = SPI_POLARITY_HIGH;
    hspi3.Init.CLKPhase            = SPI_PHASE_2EDGE;
    hspi3.Init.NSS                 = SPI_NSS_SOFT;
    hspi3.Init.BaudRatePrescaler   = BOARD_ENC_SPI_PRESCALER;
    hspi3.Init.FirstBit            = SPI_FIRSTBIT_MSB;
    hspi3.Init.TIMode              = SPI_TIMODE_DISABLE;
    hspi3.Init.CRCCalculation      = SPI_CRCCALCULATION_DISABLE;
    hspi3.Init.NSSPMode            = SPI_NSS_PULSE_DISABLE;
    if (HAL_SPI_Init(&hspi3) != HAL_OK)
    {
        Error_Handler();
    }

    BSP_ENC_CsHigh();
}

/**
  * @brief  设置方向反转标志（预定位校准结果）
  */
void BSP_ENC_SetInvert(uint8_t invert)
{
    encInvert = (invert != 0U) ? 1U : 0U;
}

/**
  * @brief  读取方向反转标志
  */
uint8_t BSP_ENC_GetInvert(void)
{
    return encInvert;
}

/**
  * @brief  读取一次编码器角度（ISR 安全，阻塞约 20us）
  * @param  data 编码器数据结构体
  * @retval 本次读取状态
  */
ENC_StatusTypeDef BSP_ENC_Read(ENC_DataTypeDef *data)
{
    uint16_t raw;
    int32_t  delta;

    if (BSP_ENC_ReadRawOnce(&raw) != HAL_OK)
    {
        data->CrcErrCnt++;
        if (data->ConsecutiveErr < ENC_CONSECUTIVE_ERR_MAX)
        {
            data->ConsecutiveErr++;
        }
        data->Status = ENC_ERR_CRC;
        return ENC_ERR_CRC;
    }

    if (encInvert != 0U)
    {
        raw = (uint16_t)((ENC_CPR - raw) & ENC_CPR_MASK);
    }

    delta = (int32_t)raw - (int32_t)data->Raw;
    if (delta > (int32_t)(ENC_CPR / 2U))
    {
        data->Turns--;
    }
    else if (delta < -(int32_t)(ENC_CPR / 2U))
    {
        data->Turns++;
    }

    data->Raw            = raw;
    data->MgStatus       = 0U;
    data->ConsecutiveErr = 0U;
    data->Status         = ENC_OK;
    return ENC_OK;
}

/**
  * @brief  预置编码器状态（上电首读）
  */
void BSP_ENC_Preload(ENC_DataTypeDef *data)
{
    uint32_t retry;

    data->Turns          = 0;
    data->ConsecutiveErr = 0U;
    data->CrcErrCnt      = 0U;
    data->FieldErrCnt    = 0U;

    for (retry = 0U; retry < 8U; retry++)
    {
        ENC_DataTypeDef tmp = *data;
        ENC_StatusTypeDef st = BSP_ENC_Read(&tmp);

        if (st == ENC_OK)
        {
            data->Raw      = tmp.Raw;
            data->MgStatus = tmp.MgStatus;
            break;
        }
    }
}

/* Private functions ---------------------------------------------------------*/

static void BSP_ENC_CsLow(void)
{
    HAL_GPIO_WritePin(MT6816_CSN_GPIO_Port, MT6816_CSN_Pin, GPIO_PIN_RESET);
}

static void BSP_ENC_CsHigh(void)
{
    HAL_GPIO_WritePin(MT6816_CSN_GPIO_Port, MT6816_CSN_Pin, GPIO_PIN_SET);
}

static uint16_t BSP_ENC_BuildReadCmd(uint8_t reg)
{
    return (uint16_t)((0x80U | reg) << 8);
}

static uint8_t BSP_ENC_ParityEven(uint16_t sample)
{
    sample ^= (uint16_t)(sample >> 8);
    sample ^= (uint16_t)(sample >> 4);
    sample ^= (uint16_t)(sample >> 2);
    sample ^= (uint16_t)(sample >> 1);
    return ((~sample) & 1U) != 0U;
}

static HAL_StatusTypeDef BSP_ENC_SpiTransfer16(uint16_t tx, uint16_t *rx)
{
    if (rx == NULL)
    {
        return HAL_ERROR;
    }

    return HAL_SPI_TransmitReceive(&hspi3, (uint8_t *)&tx, (uint8_t *)rx,
                                   1U, MT6816_SPI_TIMEOUT_MS);
}

static HAL_StatusTypeDef BSP_ENC_ReadRawOnce(uint16_t *raw14)
{
    const uint16_t txReg03 = BSP_ENC_BuildReadCmd(MT6816_REG_ANGLE_H);
    const uint16_t txReg04 = BSP_ENC_BuildReadCmd(MT6816_REG_ANGLE_L);
    uint16_t rx03 = 0U;
    uint16_t rx04 = 0U;
    uint16_t sample = 0U;
    uint8_t attempt;

    if (raw14 == NULL)
    {
        return HAL_ERROR;
    }

    for (attempt = 0U; attempt < MT6816_READ_RETRIES; attempt++)
    {
        BSP_ENC_CsLow();
        if (BSP_ENC_SpiTransfer16(txReg03, &rx03) != HAL_OK)
        {
            BSP_ENC_CsHigh();
            return HAL_ERROR;
        }
        BSP_ENC_CsHigh();

        BSP_ENC_CsLow();
        if (BSP_ENC_SpiTransfer16(txReg04, &rx04) != HAL_OK)
        {
            BSP_ENC_CsHigh();
            return HAL_ERROR;
        }
        BSP_ENC_CsHigh();

        sample = (uint16_t)(((rx03 & 0x00FFU) << 8) | (rx04 & 0x00FFU));

        if (BSP_ENC_ParityEven(sample) == 0U)
        {
            continue;
        }

        if ((sample & MT6816_NO_MAGNET_BIT) != 0U)
        {
            return HAL_ERROR;
        }

        *raw14 = (uint16_t)(sample >> 2);
        return HAL_OK;
    }

    return HAL_ERROR;
}
