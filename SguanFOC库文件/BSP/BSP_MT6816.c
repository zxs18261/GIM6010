/**
  ******************************************************************************
  * @file    BSP_MT6816.c
  * @brief   GIM6010 板载 MT6816 磁编码器驱动（SPI3 + GPIO 片选）
  * @note    读寄存器时序参考「手搓与AI编写」bsp_encoder.c：
  *          Mode3 / 16bit 帧，读 0x03+0x04，偶校验 + NoMag 检测。
  ******************************************************************************
  */

#include "BSP_MT6816.h"
#include "spi.h"
#include "main.h"

#include <math.h>

/* Private defines -----------------------------------------------------------*/
#ifndef M_PI
#define M_PI                    3.14159265358979323846
#endif

#define BSP_MT6816_TWO_PI       (6.28318530717958647692f)
#define BSP_MT6816_RAD2DEG      (57.2957795130823208768f)
#define BSP_MT6816_RPS2RPM      (9.5492965855137211f)   /* rad/s -> rpm: 60/(2π) */

#define MT6816_REG_ANGLE_H      (0x03U)
#define MT6816_REG_ANGLE_L      (0x04U)
#define MT6816_SPI_TIMEOUT_MS   (5U)
#define MT6816_READ_RETRIES     (3U)
#define MT6816_NO_MAGNET_BIT    (0x0002U)
#define MT6816_PWRUP_DELAY_MS   (16U)
#define MT6816_SPI_PRESCALER    SPI_BAUDRATEPRESCALER_32  /* ~5.3MHz @170MHz */

/* Private variables ---------------------------------------------------------*/
static uint8_t  s_invert = 0U;
static uint8_t  s_inited = 0U;
static float    s_last_mech_rad = 0.0f;
static float    s_last_elec_rad = 0.0f;
static float    s_prev_cont_rad = 0.0f;
static uint32_t s_prev_tick_ms = 0U;
static uint8_t  s_speed_valid = 0U;
static int32_t  s_turns = 0;
static uint16_t s_prev_raw = 0U;

/* Private prototypes --------------------------------------------------------*/
static void              MT6816_CsLow(void);
static void              MT6816_CsHigh(void);
static uint16_t          MT6816_BuildReadCmd(uint8_t reg);
static uint8_t           MT6816_ParityEven(uint16_t sample);
static HAL_StatusTypeDef MT6816_SpiTransfer16(uint16_t tx, uint16_t *rx);
static BSP_MT6816_Status_t MT6816_ReadRawOnce(uint16_t *raw14);
static float             MT6816_WrapTwoPi(float rad);
static void              MT6816_FillAngles(BSP_MT6816_Data_t *data, uint16_t raw14);
static void              MT6816_FillSpeeds(BSP_MT6816_Data_t *data, float dt_s);

/* Exported functions --------------------------------------------------------*/

void BSP_MT6816_Init(void)
{
    /* CubeMX SPI3 常为占位值，此处重配为 MT6816 要求的 Mode3 + 16bit */
    hspi3.Init.Mode              = SPI_MODE_MASTER;
    hspi3.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi3.Init.DataSize          = SPI_DATASIZE_16BIT;
    hspi3.Init.CLKPolarity       = SPI_POLARITY_HIGH;
    hspi3.Init.CLKPhase          = SPI_PHASE_2EDGE;
    hspi3.Init.NSS               = SPI_NSS_SOFT;
    hspi3.Init.BaudRatePrescaler = MT6816_SPI_PRESCALER;
    hspi3.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi3.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi3.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi3.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;
    if (HAL_SPI_Init(&hspi3) != HAL_OK)
    {
        Error_Handler();
    }

    MT6816_CsHigh();
    HAL_Delay(MT6816_PWRUP_DELAY_MS);

    s_turns         = 0;
    s_prev_raw      = 0U;
    s_prev_cont_rad = 0.0f;
    s_prev_tick_ms  = HAL_GetTick();
    s_speed_valid   = 0U;
    s_inited        = 1U;

    /* 预读一次，建立基准 */
    {
        uint16_t raw = 0U;
        if (MT6816_ReadRawOnce(&raw) == BSP_MT6816_OK)
        {
            if (s_invert != 0U)
            {
                raw = (uint16_t)((BSP_MT6816_CPR - raw) & BSP_MT6816_CPR_MASK);
            }
            s_prev_raw      = raw;
            s_prev_cont_rad = ((float)raw / (float)BSP_MT6816_CPR) * BSP_MT6816_TWO_PI;
            s_last_mech_rad = s_prev_cont_rad;
            s_last_elec_rad = MT6816_WrapTwoPi(s_last_mech_rad * (float)BSP_MT6816_POLE_PAIRS);
            s_speed_valid   = 0U;
            s_prev_tick_ms  = HAL_GetTick();
        }
    }
}

void BSP_MT6816_SetInvert(uint8_t invert)
{
    s_invert = (invert != 0U) ? 1U : 0U;
}

uint8_t BSP_MT6816_GetInvert(void)
{
    return s_invert;
}

BSP_MT6816_Status_t BSP_MT6816_Update(BSP_MT6816_Data_t *data)
{
    uint32_t now;
    float dt_s;

    if (data == NULL)
    {
        return BSP_MT6816_ERR_SPI;
    }

    now = HAL_GetTick();
    if (s_speed_valid == 0U)
    {
        dt_s = 0.0f;
    }
    else
    {
        uint32_t elapsed = now - s_prev_tick_ms;
        if (elapsed == 0U)
        {
            elapsed = 1U; /* 同毫秒内连续调用，按 1ms 下限避免除零 */
        }
        dt_s = (float)elapsed * 0.001f;
    }
    s_prev_tick_ms = now;

    return BSP_MT6816_UpdateDt(data, dt_s);
}

BSP_MT6816_Status_t BSP_MT6816_UpdateDt(BSP_MT6816_Data_t *data, float dt_s)
{
    uint16_t raw = 0U;
    int32_t delta;

    if (data == NULL)
    {
        return BSP_MT6816_ERR_SPI;
    }
    if (s_inited == 0U)
    {
        data->status = BSP_MT6816_ERR_SPI;
        return BSP_MT6816_ERR_SPI;
    }

    {
        BSP_MT6816_Status_t rd = MT6816_ReadRawOnce(&raw);
        if (rd != BSP_MT6816_OK)
        {
            data->status = rd;
            if (rd == BSP_MT6816_ERR_PARITY)
            {
                data->parity_err_cnt++;
            }
            else if (rd == BSP_MT6816_ERR_NO_MAGNET)
            {
                data->magnet_err_cnt++;
            }
            data->mech_omega_rad_s  = 0.0f;
            data->mech_omega_rpm    = 0.0f;
            data->shaft_omega_rad_s = 0.0f;
            data->shaft_omega_rpm   = 0.0f;
            return rd;
        }
    }

    if (s_invert != 0U)
    {
        raw = (uint16_t)((BSP_MT6816_CPR - raw) & BSP_MT6816_CPR_MASK);
    }

    /* 多圈 unwrap */
    delta = (int32_t)raw - (int32_t)s_prev_raw;
    if (delta > (int32_t)(BSP_MT6816_CPR / 2U))
    {
        s_turns--;
    }
    else if (delta < -(int32_t)(BSP_MT6816_CPR / 2U))
    {
        s_turns++;
    }
    s_prev_raw = raw;

    data->raw14 = raw;
    data->turns = s_turns;
    MT6816_FillAngles(data, raw);
    MT6816_FillSpeeds(data, dt_s);

    s_last_mech_rad = data->mech_rad;
    s_last_elec_rad = data->elec_rad;
    s_prev_cont_rad = data->mech_cont_rad;
    s_speed_valid   = 1U;

    data->status = BSP_MT6816_OK;
    return BSP_MT6816_OK;
}

uint16_t BSP_MT6816_ReadRaw14(void)
{
    uint16_t raw = 0U;

    if (MT6816_ReadRawOnce(&raw) != BSP_MT6816_OK)
    {
        return 0xFFFFU;
    }
    if (s_invert != 0U)
    {
        raw = (uint16_t)((BSP_MT6816_CPR - raw) & BSP_MT6816_CPR_MASK);
    }
    return raw;
}

float BSP_MT6816_GetMechRad(void)
{
    uint16_t raw = BSP_MT6816_ReadRaw14();
    if (raw == 0xFFFFU)
    {
        return s_last_mech_rad;
    }
    s_last_mech_rad = ((float)raw / (float)BSP_MT6816_CPR) * BSP_MT6816_TWO_PI;
    return s_last_mech_rad;
}

float BSP_MT6816_GetElecRad(void)
{
    float mech = BSP_MT6816_GetMechRad();
    s_last_elec_rad = MT6816_WrapTwoPi(mech * (float)BSP_MT6816_POLE_PAIRS);
    return s_last_elec_rad;
}

/* Private functions ---------------------------------------------------------*/

static void MT6816_CsLow(void)
{
    HAL_GPIO_WritePin(MT6816_CSN_GPIO_Port, MT6816_CSN_Pin, GPIO_PIN_RESET);
}

static void MT6816_CsHigh(void)
{
    HAL_GPIO_WritePin(MT6816_CSN_GPIO_Port, MT6816_CSN_Pin, GPIO_PIN_SET);
}

static uint16_t MT6816_BuildReadCmd(uint8_t reg)
{
    return (uint16_t)((0x80U | reg) << 8);
}

static uint8_t MT6816_ParityEven(uint16_t sample)
{
    sample ^= (uint16_t)(sample >> 8);
    sample ^= (uint16_t)(sample >> 4);
    sample ^= (uint16_t)(sample >> 2);
    sample ^= (uint16_t)(sample >> 1);
    return ((~sample) & 1U) != 0U;
}

static HAL_StatusTypeDef MT6816_SpiTransfer16(uint16_t tx, uint16_t *rx)
{
    if (rx == NULL)
    {
        return HAL_ERROR;
    }
    return HAL_SPI_TransmitReceive(&hspi3, (uint8_t *)&tx, (uint8_t *)rx,
                                   1U, MT6816_SPI_TIMEOUT_MS);
}

static BSP_MT6816_Status_t MT6816_ReadRawOnce(uint16_t *raw14)
{
    const uint16_t tx03 = MT6816_BuildReadCmd(MT6816_REG_ANGLE_H);
    const uint16_t tx04 = MT6816_BuildReadCmd(MT6816_REG_ANGLE_L);
    uint16_t rx03 = 0U;
    uint16_t rx04 = 0U;
    uint16_t sample = 0U;
    uint8_t attempt;
    uint8_t saw_parity_fail = 0U;

    if (raw14 == NULL)
    {
        return BSP_MT6816_ERR_SPI;
    }

    for (attempt = 0U; attempt < MT6816_READ_RETRIES; attempt++)
    {
        MT6816_CsLow();
        if (MT6816_SpiTransfer16(tx03, &rx03) != HAL_OK)
        {
            MT6816_CsHigh();
            return BSP_MT6816_ERR_SPI;
        }
        MT6816_CsHigh();

        MT6816_CsLow();
        if (MT6816_SpiTransfer16(tx04, &rx04) != HAL_OK)
        {
            MT6816_CsHigh();
            return BSP_MT6816_ERR_SPI;
        }
        MT6816_CsHigh();

        sample = (uint16_t)(((rx03 & 0x00FFU) << 8) | (rx04 & 0x00FFU));

        if (MT6816_ParityEven(sample) == 0U)
        {
            saw_parity_fail = 1U;
            continue;
        }

        if ((sample & MT6816_NO_MAGNET_BIT) != 0U)
        {
            return BSP_MT6816_ERR_NO_MAGNET;
        }

        *raw14 = (uint16_t)(sample >> 2);
        return BSP_MT6816_OK;
    }

    return (saw_parity_fail != 0U) ? BSP_MT6816_ERR_PARITY : BSP_MT6816_ERR_SPI;
}

static float MT6816_WrapTwoPi(float rad)
{
    rad = fmodf(rad, BSP_MT6816_TWO_PI);
    if (rad < 0.0f)
    {
        rad += BSP_MT6816_TWO_PI;
    }
    return rad;
}

static void MT6816_FillAngles(BSP_MT6816_Data_t *data, uint16_t raw14)
{
    const float inv_cpr = 1.0f / (float)BSP_MT6816_CPR;
    const float inv_gear = 1.0f / BSP_MT6816_GEAR_RATIO;

    data->mech_rad      = ((float)raw14 * inv_cpr) * BSP_MT6816_TWO_PI;
    data->mech_deg      = ((float)raw14 * inv_cpr) * 360.0f;
    data->mech_cont_rad = ((float)data->turns * BSP_MT6816_TWO_PI) + data->mech_rad;
    data->mech_cont_deg = data->mech_cont_rad * BSP_MT6816_RAD2DEG;

    data->elec_rad = MT6816_WrapTwoPi(data->mech_rad * (float)BSP_MT6816_POLE_PAIRS);
    data->elec_deg = data->elec_rad * BSP_MT6816_RAD2DEG;

    data->shaft_cont_rad = data->mech_cont_rad * inv_gear;
    data->shaft_cont_deg = data->shaft_cont_rad * BSP_MT6816_RAD2DEG;
    data->shaft_rad      = MT6816_WrapTwoPi(data->shaft_cont_rad);
    data->shaft_deg      = data->shaft_rad * BSP_MT6816_RAD2DEG;
}

static void MT6816_FillSpeeds(BSP_MT6816_Data_t *data, float dt_s)
{
    float dtheta;
    float omega_mech;

    if ((s_speed_valid == 0U) || (dt_s <= 0.0f))
    {
        data->mech_omega_rad_s  = 0.0f;
        data->mech_omega_rpm    = 0.0f;
        data->shaft_omega_rad_s = 0.0f;
        data->shaft_omega_rpm   = 0.0f;
        return;
    }

    dtheta = data->mech_cont_rad - s_prev_cont_rad;
    omega_mech = dtheta / dt_s;

    data->mech_omega_rad_s  = omega_mech;
    data->mech_omega_rpm    = omega_mech * BSP_MT6816_RPS2RPM;
    data->shaft_omega_rad_s = omega_mech / BSP_MT6816_GEAR_RATIO;
    data->shaft_omega_rpm   = data->shaft_omega_rad_s * BSP_MT6816_RPS2RPM;
}
