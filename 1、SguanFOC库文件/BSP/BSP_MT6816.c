/**
  ******************************************************************************
  * @file    BSP_MT6816.c
  * @brief   GIM6010 板载 MT6816 磁编码器驱动（SPI3 DMA + 双缓冲快照）
  * @note    FOC ISR 只读取快照。SPI3 的两帧读操作由 DMA 状态机在低优先级完成，
  *          因而不会把 HAL_SPI 轮询或 5 ms 超时带入 20 kHz 电流环。
  ******************************************************************************
  */

#include "BSP_MT6816.h"
#include "spi.h"
#include "main.h"

#include <math.h>
#include <string.h>

/* Private defines -----------------------------------------------------------*/
#define BSP_MT6816_TWO_PI            (6.28318530717958647692f)
#define BSP_MT6816_RAD2DEG           (57.2957795130823208768f)
#define BSP_MT6816_RPS2RPM           (9.5492965855137211f)

#define MT6816_REG_ANGLE_H           (0x03U)
#define MT6816_REG_ANGLE_L           (0x04U)
#define MT6816_SPI_TIMEOUT_MS        (5U)       /* 仅用于上电预读，不进入 FOC ISR */
#define MT6816_READ_RETRIES          (3U)
#define MT6816_NO_MAGNET_BIT         (0x0002U)
#define MT6816_PWRUP_DELAY_MS        (16U)
#define MT6816_DMA_TIMEOUT_US        (1000U)
#define MT6816_DMA_RETRY_US          (50U)

typedef enum
{
    MT6816_DMA_IDLE = 0U,
    MT6816_DMA_READ_HIGH,
    MT6816_DMA_READ_LOW
} MT6816_DmaStage_t;

/* Private variables ---------------------------------------------------------*/
static uint8_t s_invert = 0U;
static volatile uint8_t s_inited = 0U;
static volatile uint8_t s_dma_ready = 0U;
static volatile uint8_t s_dma_busy = 0U;
static volatile MT6816_DmaStage_t s_dma_stage = MT6816_DMA_IDLE;

/* 生产者：SPI DMA 回调；消费者：ADC ISR / 主循环。仅切换 active 索引。 */
static BSP_MT6816_Data_t s_cache[2];
static volatile uint8_t s_active_cache = 0U;

static uint16_t s_prev_raw = 0U;
static int32_t s_turns = 0;
static float s_prev_cont_rad = 0.0f;
static uint8_t s_speed_valid = 0U;
static uint32_t s_sample_sequence = 0U;

static volatile uint32_t s_request_seq = 0U;
static volatile uint32_t s_started_request_seq = 0U;
static volatile uint8_t s_service_busy = 0U;
static volatile uint32_t s_request_cnt = 0U;
static volatile uint32_t s_started_cnt = 0U;
static volatile uint32_t s_completed_cnt = 0U;
static volatile uint32_t s_dropped_request_cnt = 0U;
static volatile uint32_t s_dma_error_cnt = 0U;

static uint32_t s_dma_start_cycles = 0U;
static uint32_t s_last_start_failure_cycles = 0U;
static uint32_t s_stale_timeout_cycles = 0U;
static float s_seconds_per_cycle = 0.0f;
static uint16_t s_dma_tx = 0U;
static uint16_t s_dma_rx = 0U;
static uint16_t s_dma_rx_high = 0U;

/* Private prototypes --------------------------------------------------------*/
static void MT6816_CsLow(void);
static void MT6816_CsHigh(void);
static void MT6816_DwtInit(void);
static uint32_t MT6816_CyclesNow(void);
static uint32_t MT6816_CyclesToUs(uint32_t cycles);
static uint16_t MT6816_BuildReadCmd(uint8_t reg);
static uint8_t MT6816_ParityEven(uint16_t sample);
static HAL_StatusTypeDef MT6816_SpiTransfer16Blocking(uint16_t tx, uint16_t *rx);
static BSP_MT6816_Status_t MT6816_ReadRawOnceBlocking(uint16_t *raw14);
static uint8_t MT6816_CubeInterfaceReady(void);
static HAL_StatusTypeDef MT6816_StartDmaTransfer(uint16_t command, MT6816_DmaStage_t stage);
static void MT6816_Publish(BSP_MT6816_Data_t *data);
static void MT6816_PublishError(BSP_MT6816_Status_t status);
static void MT6816_PublishRaw(uint16_t rx_high, uint16_t rx_low);
static void MT6816_RescheduleFailedRequest(void);
static float MT6816_WrapTwoPi(float rad);
static float MT6816_WrapTwoPiFast(float rad);
static void MT6816_UpdateFromRaw(BSP_MT6816_Data_t *data, uint16_t raw14, float dt_s);
static void MT6816_FillAngles(BSP_MT6816_Data_t *data, uint16_t raw14);
static void MT6816_FillSpeeds(BSP_MT6816_Data_t *data, float dt_s);

/* Exported functions --------------------------------------------------------*/

void BSP_MT6816_Init(void)
{
    BSP_MT6816_Data_t initial_data;
    BSP_MT6816_Status_t read_status;
    uint16_t raw = 0U;
    uint32_t now_cycles;

    s_inited = 0U;
    s_dma_ready = 0U;
    s_dma_busy = 0U;
    s_dma_stage = MT6816_DMA_IDLE;

    MT6816_DwtInit();
    s_stale_timeout_cycles = (SystemCoreClock / 1000000U) * BSP_MT6816_STALE_TIMEOUT_US;
    s_seconds_per_cycle = 1.0f / (float)SystemCoreClock;
    MT6816_CsHigh();
    HAL_Delay(MT6816_PWRUP_DELAY_MS);

    memset(&initial_data, 0, sizeof(initial_data));
    s_turns = 0;
    s_prev_raw = 0U;
    s_prev_cont_rad = 0.0f;
    s_speed_valid = 0U;
    s_sample_sequence = 0U;
    s_request_seq = 0U;
    s_started_request_seq = 0U;
    s_service_busy = 0U;
    s_request_cnt = 0U;
    s_started_cnt = 0U;
    s_completed_cnt = 0U;
    s_dropped_request_cnt = 0U;
    s_dma_error_cnt = 0U;
    s_last_start_failure_cycles = 0U;

    /* 上电只允许一次阻塞预读，建立缓存基准；运行阶段全部走 DMA。 */
    read_status = MT6816_ReadRawOnceBlocking(&raw);
    now_cycles = MT6816_CyclesNow();
    if (read_status == BSP_MT6816_OK)
    {
        if (s_invert != 0U)
        {
            raw = (uint16_t)((BSP_MT6816_CPR - raw) & BSP_MT6816_CPR_MASK);
        }
        s_prev_raw = raw;
        s_prev_cont_rad = ((float)raw / (float)BSP_MT6816_CPR) * BSP_MT6816_TWO_PI;
        initial_data.raw14 = raw;
        initial_data.turns = 0;
        MT6816_FillAngles(&initial_data, raw);
        MT6816_FillSpeeds(&initial_data, 0.0f);
        initial_data.status = BSP_MT6816_OK;
        initial_data.sequence = ++s_sample_sequence;
        initial_data.timestamp_cycles = now_cycles;
    }
    else
    {
        initial_data.status = read_status;
        initial_data.timestamp_cycles = 0U;
        if (read_status == BSP_MT6816_ERR_PARITY)
        {
            initial_data.parity_err_cnt = 1U;
        }
        else if (read_status == BSP_MT6816_ERR_NO_MAGNET)
        {
            initial_data.magnet_err_cnt = 1U;
        }
    }

    s_cache[0] = initial_data;
    s_cache[1] = initial_data;
    __DMB();
    s_active_cache = 0U;

    /* SPI3 和两路 DMA 由 CubeMX 独占配置；BSP 只校验并使用已链接句柄。 */
    s_dma_ready = MT6816_CubeInterfaceReady();
    s_inited = 1U;
}

void BSP_MT6816_SetInvert(uint8_t invert)
{
    s_invert = (invert != 0U) ? 1U : 0U;
    /* 方向切换后，下一帧重新建立速度/多圈基准，避免产生假速度尖峰。 */
    s_speed_valid = 0U;
    s_turns = 0;
    s_prev_raw = 0U;
    s_prev_cont_rad = 0.0f;
}

uint8_t BSP_MT6816_GetInvert(void)
{
    return s_invert;
}

void BSP_MT6816_RequestSampleFromISR(void)
{
    if ((s_inited != 0U) && (s_dma_ready != 0U))
    {
        /* 32 位自增在 Cortex-M4 上原子；服务端只保留最新请求并统计合并数。 */
        s_request_seq++;
        s_request_cnt++;
    }
}

void BSP_MT6816_Service(void)
{
    uint32_t request_seq;
    uint32_t timeout_cycles;
    uint32_t now_cycles;
    uint32_t request_gap;

    if ((s_inited == 0U) || (s_dma_ready == 0U))
    {
        return;
    }

    /* 由主循环及其校准延时调用；锁保护启动/恢复决策，不屏蔽 ADC ISR。 */
    if (s_service_busy != 0U)
    {
        return;
    }
    s_service_busy = 1U;

    timeout_cycles = (SystemCoreClock / 1000000U) * MT6816_DMA_TIMEOUT_US;
    now_cycles = MT6816_CyclesNow();
    if (s_dma_busy != 0U)
    {
        if ((uint32_t)(now_cycles - s_dma_start_cycles) > timeout_cycles)
        {
            /* 仅在主循环中恢复总线；绝不会阻塞 ADC 电流环。 */
            (void)HAL_SPI_Abort(&hspi3);
            MT6816_CsHigh();
            s_dma_busy = 0U;
            s_dma_stage = MT6816_DMA_IDLE;
            s_dma_error_cnt++;
            MT6816_PublishError(BSP_MT6816_ERR_SPI);
            MT6816_RescheduleFailedRequest();
        }
        s_service_busy = 0U;
        return;
    }

    request_seq = s_request_seq;
    if (request_seq == s_started_request_seq)
    {
        s_service_busy = 0U;
        return;
    }

    /* SPI 短暂 Busy 时限速重试，避免主循环紧密重试占满 CPU。 */
    if ((s_last_start_failure_cycles != 0U) &&
        ((uint32_t)(now_cycles - s_last_start_failure_cycles) <
         ((SystemCoreClock / 1000000U) * MT6816_DMA_RETRY_US)))
    {
        s_service_busy = 0U;
        return;
    }

    if (MT6816_StartDmaTransfer(MT6816_BuildReadCmd(MT6816_REG_ANGLE_H),
                                MT6816_DMA_READ_HIGH) == HAL_OK)
    {
        request_gap = (uint32_t)(request_seq - s_started_request_seq);
        if (request_gap > 1U)
        {
            /* 多个请求积压时，读最新时刻即可；丢弃的中间请求计入诊断。 */
            s_dropped_request_cnt += request_gap - 1U;
        }
        s_started_request_seq = request_seq;
        s_last_start_failure_cycles = 0U;
        s_started_cnt++;
    }
    else
    {
        s_last_start_failure_cycles = now_cycles;
        s_dma_error_cnt++;
        MT6816_PublishError(BSP_MT6816_ERR_SPI);
    }
    s_service_busy = 0U;
}

uint8_t BSP_MT6816_GetSnapshot(BSP_MT6816_Data_t *data)
{
    uint8_t active;

    if (data == NULL)
    {
        return 0U;
    }

    active = s_active_cache;
    __DMB();
    *data = s_cache[active];
    return (data->status == BSP_MT6816_OK) ? 1U : 0U;
}

uint8_t BSP_MT6816_IsSampleFresh(void)
{
    uint8_t active = s_active_cache;
    BSP_MT6816_Status_t status;
    uint32_t timestamp_cycles;
    uint32_t sequence;

    __DMB();
    status = s_cache[active].status;
    timestamp_cycles = s_cache[active].timestamp_cycles;
    sequence = s_cache[active].sequence;
    if ((status != BSP_MT6816_OK) || (sequence == 0U))
    {
        return 0U;
    }
    return ((uint32_t)(MT6816_CyclesNow() - timestamp_cycles) <=
            s_stale_timeout_cycles) ? 1U : 0U;
}

uint32_t BSP_MT6816_GetSampleAgeUs(void)
{
    uint8_t active = s_active_cache;
    uint32_t timestamp_cycles;
    uint32_t sequence;

    __DMB();
    timestamp_cycles = s_cache[active].timestamp_cycles;
    sequence = s_cache[active].sequence;
    if (sequence == 0U)
    {
        return UINT32_MAX;
    }
    return MT6816_CyclesToUs(MT6816_CyclesNow() - timestamp_cycles);
}

void BSP_MT6816_GetDiagnostics(BSP_MT6816_Diagnostics_t *diagnostics)
{
    if (diagnostics == NULL)
    {
        return;
    }

    diagnostics->request_cnt = s_request_cnt;
    diagnostics->started_cnt = s_started_cnt;
    diagnostics->completed_cnt = s_completed_cnt;
    diagnostics->dropped_request_cnt = s_dropped_request_cnt;
    diagnostics->dma_error_cnt = s_dma_error_cnt;
    diagnostics->sample_age_us = BSP_MT6816_GetSampleAgeUs();
}

BSP_MT6816_Status_t BSP_MT6816_Update(BSP_MT6816_Data_t *data)
{
    if (BSP_MT6816_GetSnapshot(data) == 0U)
    {
        return (data != NULL) ? data->status : BSP_MT6816_ERR_SPI;
    }
    return BSP_MT6816_OK;
}

BSP_MT6816_Status_t BSP_MT6816_UpdateDt(BSP_MT6816_Data_t *data, float dt_s)
{
    (void)dt_s;
    return BSP_MT6816_Update(data);
}

uint16_t BSP_MT6816_ReadRaw14(void)
{
    BSP_MT6816_Data_t data;

    if (BSP_MT6816_GetSnapshot(&data) == 0U)
    {
        return 0xFFFFU;
    }
    return data.raw14;
}

float BSP_MT6816_GetMechRad(void)
{
    uint8_t active = s_active_cache;

    /* 传输瞬时出错时仍保持最后一个有效角度，不能在 FOC 中突变为零。 */
    __DMB();
    return s_cache[active].mech_rad;
}

float BSP_MT6816_GetMechRadForControl(void)
{
    BSP_MT6816_Status_t status;
    float mech_rad;
    float mech_omega_rad_s;
    uint32_t timestamp_cycles;
    uint32_t sequence;
    uint32_t now_cycles;
    float age_s;
    uint8_t active = s_active_cache;

    __DMB();
    status = s_cache[active].status;
    mech_rad = s_cache[active].mech_rad;
    mech_omega_rad_s = s_cache[active].mech_omega_rad_s;
    timestamp_cycles = s_cache[active].timestamp_cycles;
    sequence = s_cache[active].sequence;
    now_cycles = MT6816_CyclesNow();

    if ((status != BSP_MT6816_OK) || (sequence == 0U) ||
        ((uint32_t)(now_cycles - timestamp_cycles) > s_stale_timeout_cycles))
    {
        return mech_rad;
    }

    age_s = (float)(now_cycles - timestamp_cycles) * s_seconds_per_cycle;
    return MT6816_WrapTwoPiFast(mech_rad + mech_omega_rad_s * age_s);
}

float BSP_MT6816_GetElecRad(void)
{
    uint8_t active = s_active_cache;

    __DMB();
    return s_cache[active].elec_rad;
}

/* HAL callbacks / IRQ handlers ---------------------------------------------*/

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi != &hspi3)
    {
        return;
    }

    MT6816_CsHigh();
    if (s_dma_stage == MT6816_DMA_READ_HIGH)
    {
        s_dma_rx_high = s_dma_rx;
        if (MT6816_StartDmaTransfer(MT6816_BuildReadCmd(MT6816_REG_ANGLE_L),
                                    MT6816_DMA_READ_LOW) != HAL_OK)
        {
            s_dma_busy = 0U;
            s_dma_stage = MT6816_DMA_IDLE;
            s_dma_error_cnt++;
            MT6816_PublishError(BSP_MT6816_ERR_SPI);
            MT6816_RescheduleFailedRequest();
        }
        return;
    }

    if (s_dma_stage == MT6816_DMA_READ_LOW)
    {
        s_dma_busy = 0U;
        s_dma_stage = MT6816_DMA_IDLE;
        MT6816_PublishRaw(s_dma_rx_high, s_dma_rx);
        return;
    }

    s_dma_busy = 0U;
    s_dma_stage = MT6816_DMA_IDLE;
    s_dma_error_cnt++;
    MT6816_PublishError(BSP_MT6816_ERR_SPI);
    MT6816_RescheduleFailedRequest();
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi3)
    {
        MT6816_CsHigh();
        s_dma_busy = 0U;
        s_dma_stage = MT6816_DMA_IDLE;
        s_dma_error_cnt++;
        MT6816_PublishError(BSP_MT6816_ERR_SPI);
        MT6816_RescheduleFailedRequest();
    }
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

static void MT6816_DwtInit(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static uint32_t MT6816_CyclesNow(void)
{
    return DWT->CYCCNT;
}

static uint32_t MT6816_CyclesToUs(uint32_t cycles)
{
    uint32_t cycles_per_us = SystemCoreClock / 1000000U;

    if (cycles_per_us == 0U)
    {
        return UINT32_MAX;
    }
    return cycles / cycles_per_us;
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

static HAL_StatusTypeDef MT6816_SpiTransfer16Blocking(uint16_t tx, uint16_t *rx)
{
    if (rx == NULL)
    {
        return HAL_ERROR;
    }
    return HAL_SPI_TransmitReceive(&hspi3, (uint8_t *)&tx, (uint8_t *)rx,
                                   1U, MT6816_SPI_TIMEOUT_MS);
}

static BSP_MT6816_Status_t MT6816_ReadRawOnceBlocking(uint16_t *raw14)
{
    const uint16_t tx03 = MT6816_BuildReadCmd(MT6816_REG_ANGLE_H);
    const uint16_t tx04 = MT6816_BuildReadCmd(MT6816_REG_ANGLE_L);
    uint16_t rx03 = 0U;
    uint16_t rx04 = 0U;
    uint16_t sample;
    uint8_t attempt;
    uint8_t saw_parity_fail = 0U;

    if (raw14 == NULL)
    {
        return BSP_MT6816_ERR_SPI;
    }

    for (attempt = 0U; attempt < MT6816_READ_RETRIES; attempt++)
    {
        MT6816_CsLow();
        if (MT6816_SpiTransfer16Blocking(tx03, &rx03) != HAL_OK)
        {
            MT6816_CsHigh();
            return BSP_MT6816_ERR_SPI;
        }
        MT6816_CsHigh();

        MT6816_CsLow();
        if (MT6816_SpiTransfer16Blocking(tx04, &rx04) != HAL_OK)
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

static uint8_t MT6816_CubeInterfaceReady(void)
{
    if ((hspi3.Instance != SPI3) ||
        (hspi3.Init.Mode != SPI_MODE_MASTER) ||
        (hspi3.Init.Direction != SPI_DIRECTION_2LINES) ||
        (hspi3.Init.DataSize != SPI_DATASIZE_16BIT) ||
        (hspi3.Init.CLKPolarity != SPI_POLARITY_HIGH) ||
        (hspi3.Init.CLKPhase != SPI_PHASE_2EDGE) ||
        (hspi3.Init.NSS != SPI_NSS_SOFT) ||
        (hspi3.Init.FirstBit != SPI_FIRSTBIT_MSB) ||
        (hspi3.hdmarx == NULL) ||
        (hspi3.hdmatx == NULL))
    {
        return 0U;
    }

    if ((hspi3.hdmarx->Init.Direction != DMA_PERIPH_TO_MEMORY) ||
        (hspi3.hdmatx->Init.Direction != DMA_MEMORY_TO_PERIPH) ||
        (hspi3.hdmarx->Init.PeriphDataAlignment != DMA_PDATAALIGN_HALFWORD) ||
        (hspi3.hdmarx->Init.MemDataAlignment != DMA_MDATAALIGN_HALFWORD) ||
        (hspi3.hdmatx->Init.PeriphDataAlignment != DMA_PDATAALIGN_HALFWORD) ||
        (hspi3.hdmatx->Init.MemDataAlignment != DMA_MDATAALIGN_HALFWORD) ||
        (hspi3.hdmarx->Init.Mode != DMA_NORMAL) ||
        (hspi3.hdmatx->Init.Mode != DMA_NORMAL))
    {
        return 0U;
    }

    return 1U;
}

static HAL_StatusTypeDef MT6816_StartDmaTransfer(uint16_t command, MT6816_DmaStage_t stage)
{
    HAL_StatusTypeDef result;

    s_dma_tx = command;
    s_dma_rx = 0U;
    s_dma_stage = stage;
    s_dma_busy = 1U;
    s_dma_start_cycles = MT6816_CyclesNow();
    MT6816_CsLow();
    result = HAL_SPI_TransmitReceive_DMA(&hspi3, (uint8_t *)&s_dma_tx,
                                         (uint8_t *)&s_dma_rx, 1U);
    if (result != HAL_OK)
    {
        MT6816_CsHigh();
        s_dma_busy = 0U;
        s_dma_stage = MT6816_DMA_IDLE;
    }
    return result;
}

static void MT6816_Publish(BSP_MT6816_Data_t *data)
{
    uint8_t inactive = (uint8_t)(s_active_cache ^ 1U);

    s_cache[inactive] = *data;
    __DMB();
    s_active_cache = inactive;
}

static void MT6816_PublishError(BSP_MT6816_Status_t status)
{
    BSP_MT6816_Data_t data;
    uint8_t active = s_active_cache;

    __DMB();
    data = s_cache[active];
    data.status = status;
    data.mech_omega_rad_s = 0.0f;
    data.mech_omega_rpm = 0.0f;
    data.shaft_omega_rad_s = 0.0f;
    data.shaft_omega_rpm = 0.0f;
    if (status == BSP_MT6816_ERR_PARITY)
    {
        data.parity_err_cnt++;
    }
    else if (status == BSP_MT6816_ERR_NO_MAGNET)
    {
        data.magnet_err_cnt++;
    }
    MT6816_Publish(&data);
}

static void MT6816_PublishRaw(uint16_t rx_high, uint16_t rx_low)
{
    BSP_MT6816_Data_t data;
    uint16_t sample;
    uint16_t raw;
    uint8_t active;
    uint32_t now_cycles;
    float dt_s;

    sample = (uint16_t)(((rx_high & 0x00FFU) << 8) | (rx_low & 0x00FFU));
    if (MT6816_ParityEven(sample) == 0U)
    {
        MT6816_PublishError(BSP_MT6816_ERR_PARITY);
        return;
    }
    if ((sample & MT6816_NO_MAGNET_BIT) != 0U)
    {
        MT6816_PublishError(BSP_MT6816_ERR_NO_MAGNET);
        return;
    }

    raw = (uint16_t)(sample >> 2);
    if (s_invert != 0U)
    {
        raw = (uint16_t)((BSP_MT6816_CPR - raw) & BSP_MT6816_CPR_MASK);
    }

    active = s_active_cache;
    __DMB();
    data = s_cache[active];
    now_cycles = MT6816_CyclesNow();
    if ((s_speed_valid == 0U) || (data.timestamp_cycles == 0U))
    {
        dt_s = 0.0f;
    }
    else
    {
        dt_s = (float)(now_cycles - data.timestamp_cycles) / (float)SystemCoreClock;
    }

    MT6816_UpdateFromRaw(&data, raw, dt_s);
    data.status = BSP_MT6816_OK;
    data.sequence = ++s_sample_sequence;
    data.timestamp_cycles = now_cycles;
    MT6816_Publish(&data);
    s_completed_cnt++;
}

static void MT6816_RescheduleFailedRequest(void)
{
    /* 若当前 DMA 对应最新请求，则让下一次 Service 立即重试该请求。 */
    if (s_started_request_seq == s_request_seq)
    {
        s_started_request_seq--;
    }
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

/* FOC 每周期调用：避免 libm fmodf 落入 20 kHz ADC ISR。 */
static float MT6816_WrapTwoPiFast(float rad)
{
    const float inv_two_pi = 1.0f / BSP_MT6816_TWO_PI;
    int32_t turns = (int32_t)(rad * inv_two_pi);

    rad -= (float)turns * BSP_MT6816_TWO_PI;
    if (rad < 0.0f)
    {
        rad += BSP_MT6816_TWO_PI;
    }
    else if (rad >= BSP_MT6816_TWO_PI)
    {
        rad -= BSP_MT6816_TWO_PI;
    }
    return rad;
}

static void MT6816_UpdateFromRaw(BSP_MT6816_Data_t *data, uint16_t raw14, float dt_s)
{
    int32_t delta;

    /* 初始化或运行中切换方向后，第一帧只建立基准，不能参与 unwrap。 */
    if (s_speed_valid == 0U)
    {
        s_turns = 0;
        s_prev_raw = raw14;
        data->raw14 = raw14;
        data->turns = s_turns;
        MT6816_FillAngles(data, raw14);
        MT6816_FillSpeeds(data, 0.0f);
        s_prev_cont_rad = data->mech_cont_rad;
        s_speed_valid = 1U;
        return;
    }

    delta = (int32_t)raw14 - (int32_t)s_prev_raw;
    if (delta > (int32_t)(BSP_MT6816_CPR / 2U))
    {
        s_turns--;
    }
    else if (delta < -(int32_t)(BSP_MT6816_CPR / 2U))
    {
        s_turns++;
    }
    s_prev_raw = raw14;

    data->raw14 = raw14;
    data->turns = s_turns;
    MT6816_FillAngles(data, raw14);
    MT6816_FillSpeeds(data, dt_s);
    s_prev_cont_rad = data->mech_cont_rad;
    s_speed_valid = 1U;
}

static void MT6816_FillAngles(BSP_MT6816_Data_t *data, uint16_t raw14)
{
    const float inv_cpr = 1.0f / (float)BSP_MT6816_CPR;
    const float inv_gear = 1.0f / BSP_MT6816_GEAR_RATIO;

    data->mech_rad = ((float)raw14 * inv_cpr) * BSP_MT6816_TWO_PI;
    data->mech_deg = ((float)raw14 * inv_cpr) * 360.0f;
    data->mech_cont_rad = ((float)data->turns * BSP_MT6816_TWO_PI) + data->mech_rad;
    data->mech_cont_deg = data->mech_cont_rad * BSP_MT6816_RAD2DEG;

    data->elec_rad = MT6816_WrapTwoPi(data->mech_rad * (float)BSP_MT6816_POLE_PAIRS);
    data->elec_deg = data->elec_rad * BSP_MT6816_RAD2DEG;

    data->shaft_cont_rad = data->mech_cont_rad * inv_gear;
    data->shaft_cont_deg = data->shaft_cont_rad * BSP_MT6816_RAD2DEG;
    data->shaft_rad = MT6816_WrapTwoPi(data->shaft_cont_rad);
    data->shaft_deg = data->shaft_rad * BSP_MT6816_RAD2DEG;
}

static void MT6816_FillSpeeds(BSP_MT6816_Data_t *data, float dt_s)
{
    float dtheta;
    float omega_mech;

    if ((s_speed_valid == 0U) || (dt_s <= 0.0f))
    {
        data->mech_omega_rad_s = 0.0f;
        data->mech_omega_rpm = 0.0f;
        data->shaft_omega_rad_s = 0.0f;
        data->shaft_omega_rpm = 0.0f;
        return;
    }

    dtheta = data->mech_cont_rad - s_prev_cont_rad;
    omega_mech = dtheta / dt_s;
    data->mech_omega_rad_s = omega_mech;
    data->mech_omega_rpm = omega_mech * BSP_MT6816_RPS2RPM;
    data->shaft_omega_rad_s = omega_mech / BSP_MT6816_GEAR_RATIO;
    data->shaft_omega_rpm = data->shaft_omega_rad_s * BSP_MT6816_RPS2RPM;
}
