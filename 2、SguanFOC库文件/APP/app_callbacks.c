/**
  ******************************************************************************
  * @file    app_callbacks.c
  * @brief   GIM6010 HAL 回调聚合实现（移植自旧工程 Hardware/Timer.c 并扩展）
  * @note    - ADC 注入回调发生在 PWM 采样窗口：先锁存注入 Rank，再进电流环
  *          - MT6816 由主循环阻塞读取；20kHz ISR 不访问 SPI
  *          - 低频状态机/保护固定在 1kHz TIM2，绝不在此重复运行 High_Loop
  ******************************************************************************
  */
#include "app_callbacks.h"
#include "adc.h"
#include "SguanFOC.h"
#include "Sguan_printf.h"
#include "BSP_MT6816.h"
#include "BSP_ADC.h"
#include "canfd.h"
#include "UserData_Function.h"
#include "UserData_UserControl.h"
#include "usart.h"
#include "main.h"

/* ============================ 对外状态 ==================================== */
volatile uint32_t ADC_InjectedValues[4] = {0};

volatile uint32_t g_high_loop_cnt = 0;
volatile uint32_t g_high_loop_last_cycles = 0U;
volatile uint32_t g_high_loop_max_cycles = 0U;
volatile uint32_t g_high_loop_deadline_miss_cnt = 0U;
volatile APP_FocFaultSnapshot_t g_foc_fault_snapshot = {0};
volatile uint32_t g_uart_rx_drop_cnt = 0U;
volatile uint32_t g_uart_rx_error_cnt = 0U;
volatile uint32_t g_uart_rx_restart_fail_cnt = 0U;

#define APP_UART_RX_QUEUE_DEPTH  (4U)
static uint8_t s_uart_rx_queue[APP_UART_RX_QUEUE_DEPTH][sizeof(Sguan_PrintfBuff)];
static uint16_t s_uart_rx_length[APP_UART_RX_QUEUE_DEPTH];
static volatile uint8_t s_uart_rx_head = 0U;
static volatile uint8_t s_uart_rx_tail = 0U;
static volatile uint8_t s_uart_rx_restart_needed = 0U;
static volatile uint32_t s_uart_stop_sequence = 0U;
static uint32_t s_uart_stop_serviced = 0U;
static volatile uint32_t s_uart_rx_next_retry_ms = 0U;

/* TIM2(1kHz) 监视 ADC 注入完成计数，覆盖“HAL 启动成功但后续硬件触发/IRQ 停摆”。 */
static uint32_t s_adc_high_loop_last_count = 0U;
static uint16_t s_adc_high_loop_stall_ms = 0U;
static uint8_t s_adc_high_loop_monitor_active = 0U;
static uint8_t s_high_loop_deadline_miss_streak = 0U;

static void APP_LatchAdcSensorFault(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    if (Sguan.status < MOTOR_STATUS_OVERVOLTAGE) {
        Sguan.status = MOTOR_STATUS_SENSOR_ERROR;
    }
    User_PwmOutputDisable();
    if (primask == 0U) __enable_irq();
}

static void APP_CheckAdcHighLoopHeartbeat(void)
{
    uint32_t count = g_high_loop_cnt;

    if ((Sguan.status >= MOTOR_STATUS_CALIBRATING) &&
        (Sguan.status < MOTOR_STATUS_OVERVOLTAGE)) {
        if (s_adc_high_loop_monitor_active == 0U) {
            s_adc_high_loop_monitor_active = 1U;
            s_adc_high_loop_last_count = count;
            s_adc_high_loop_stall_ms = 0U;
            return;
        }
        if (count != s_adc_high_loop_last_count) {
            s_adc_high_loop_last_count = count;
            s_adc_high_loop_stall_ms = 0U;
            return;
        }
        if (s_adc_high_loop_stall_ms < USER_ADC_HIGH_LOOP_TIMEOUT_MS) {
            s_adc_high_loop_stall_ms++;
        }
        if (s_adc_high_loop_stall_ms >= USER_ADC_HIGH_LOOP_TIMEOUT_MS) {
            uint32_t primask = __get_PRIMASK();
            __disable_irq();
            /* ADC IRQ 可能在首次检查后抢占并更新计数；关中断后必须复核再锁故障。 */
            if ((Sguan.status >= MOTOR_STATUS_CALIBRATING) &&
                (Sguan.status < MOTOR_STATUS_OVERVOLTAGE) &&
                (g_high_loop_cnt == count)) {
                Sguan.status = MOTOR_STATUS_SENSOR_ERROR;
                User_PwmOutputDisable();
            } else {
                s_adc_high_loop_last_count = g_high_loop_cnt;
                s_adc_high_loop_stall_ms = 0U;
            }
            if (primask == 0U) __enable_irq();
        }
    } else {
        s_adc_high_loop_monitor_active = 0U;
        s_adc_high_loop_last_count = count;
        s_adc_high_loop_stall_ms = 0U;
    }
}

uint8_t APP_UartRxStart(void)
{
    /* 先清重启请求再启动 DMA；若启动过程中又发生错误，错误 ISR 写入的 1
     * 必须保留下来，不能在 HAL 返回后被无条件覆盖。 */
    s_uart_rx_restart_needed = 0U;
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, Sguan_PrintfBuff,
                                     sizeof(Sguan_PrintfBuff)) != HAL_OK) {
        s_uart_rx_restart_needed = 1U;
        return 0U;
    }
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    return 1U;
}

uint8_t APP_UartStopPending(void)
{
    return (s_uart_stop_sequence != s_uart_stop_serviced) ? 1U : 0U;
}

/* ============================ 定时器回调 ================================== */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        /* 1kHz：先确认 20kHz ADC 注入链仍有心跳，再运行低频状态机/保护。 */
        APP_CheckAdcHighLoopHeartbeat();
        SguanFOC_Low_Loop();
    }
    /* TIM4 的 PWM DMA 完成（WS2812B）由 BSP_WS2812B.c 的
     * HAL_TIM_PWM_PulseFinishedCallback 处理，此处不重复实现。 */
}

/* ============================ ADC 注入完成回调 ============================ */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        uint32_t start_cycles = DWT->CYCCNT;
        uint32_t used_cycles;
        static uint32_t deadline_cycles = 0U;

        if (deadline_cycles == 0U) {
            deadline_cycles = SystemCoreClock / USER_FOC_CONTROL_HZ;
        }

        /* 先锁存注入 Rank：R1=IV, R2=IW, R3=VBUS(快速通道) */
        ADC_InjectedValues[0] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
        ADC_InjectedValues[1] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
        ADC_InjectedValues[2] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_3);
        BSP_ADC_FeedVBUSFast(ADC_InjectedValues[2]);
        /* 1kHz 低环只看最新电压，短脉冲可能在下一样本恢复后被覆盖；因此在
         * 20kHz 采样现场直接关桥并锁故障。状态锁存后正常样本不会自动清除。 */
        if (ADC_InjectedValues[2] >= BSP_ADC_VBUS_SAT_RAW) {
            User_PwmOutputDisable();
            if (Sguan.status < MOTOR_STATUS_OVERVOLTAGE) {
                Sguan.status = MOTOR_STATUS_OVERVOLTAGE;
            }
        }

        g_high_loop_cnt++;
        /* 编码器采样：DMA 投递链在 FOC 使能后不稳定（2026-08-30 台架，见
         * BSP_MT6816_PollBlocking 注释），现由主循环 1kHz 阻塞直读，ISR 不再投递。 */

        SguanFOC_High_Loop();

        used_cycles = DWT->CYCCNT - start_cycles;
        g_high_loop_last_cycles = used_cycles;
        if (used_cycles > g_high_loop_max_cycles) {
            g_high_loop_max_cycles = used_cycles;
        }
        if (used_cycles >= deadline_cycles) {
            g_high_loop_deadline_miss_cnt++;
            if ((Sguan.status >= MOTOR_STATUS_CALIBRATING) &&
                (Sguan.status < MOTOR_STATUS_OVERVOLTAGE)) {
                if (s_high_loop_deadline_miss_streak < USER_FOC_DEADLINE_MISS_LIMIT) {
                    s_high_loop_deadline_miss_streak++;
                }
                if (s_high_loop_deadline_miss_streak >= USER_FOC_DEADLINE_MISS_LIMIT) {
                    Sguan.status = MOTOR_STATUS_PWM_CALC_FAULT;
                    User_PwmOutputDisable();
                    /* 下一次显式故障恢复必须重新累计完整的连续超期次数。 */
                    s_high_loop_deadline_miss_streak = 0U;
                }
            } else {
                s_high_loop_deadline_miss_streak = 0U;
            }
        } else {
            s_high_loop_deadline_miss_streak = 0U;
        }
    }
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        APP_LatchAdcSensorFault();
    }
}

void HAL_ADCEx_InjectedQueueOverflowCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        APP_LatchAdcSensorFault();
    }
}

/* ============================ 串口回调 ==================================== */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1) {
        uint8_t head = s_uart_rx_head;
        uint8_t next = (uint8_t)((head + 1U) % APP_UART_RX_QUEUE_DEPTH);
        if (Size > sizeof(Sguan_PrintfBuff)) Size = (uint16_t)sizeof(Sguan_PrintfBuff);
        if (next != s_uart_rx_tail) {
            for (uint16_t i = 0U; i < Size; i++) s_uart_rx_queue[head][i] = Sguan_PrintfBuff[i];
            s_uart_rx_length[head] = Size;
            __DMB();
            s_uart_rx_head = next;
        } else {
            /* 命令块积压时无法证明被丢弃的数据不含 CO=0；与 CAN 队列
             * 溢出保持同一失效安全策略，立即关桥并由主循环清状态/旧命令。 */
            g_uart_rx_drop_cnt++;
            User_PwmOutputDisable();
            s_uart_stop_sequence++;
        }
        if (APP_UartRxStart() == 0U) {
            /* 接收链一旦未能重启，继续保持旧目标会失去停机通道。 */
            User_PwmOutputDisable();
            s_uart_stop_sequence++;
            s_uart_rx_restart_needed = 1U;
            s_uart_rx_next_retry_ms = HAL_GetTick();
            g_uart_rx_restart_fail_cnt++;
        }
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        /* ISR 中只做确定性的硬件关断和锁存；DMA abort/rearm 留给主循环。 */
        User_PwmOutputDisable();
        s_uart_stop_sequence++;
        s_uart_rx_restart_needed = 1U;
        s_uart_rx_next_retry_ms = HAL_GetTick();
        g_uart_rx_error_cnt++;
    }
}

static uint8_t APP_UartServiceStop(void)
{
    uint32_t primask;
    uint32_t stop_sequence;

    primask = __get_PRIMASK();
    __disable_irq();
    stop_sequence = s_uart_stop_sequence;
    if (stop_sequence != s_uart_stop_serviced) {
        /* 错误前排队的命令和半帧都不得在恢复后继续执行。 */
        s_uart_rx_tail = s_uart_rx_head;
    }
    if (primask == 0U) __enable_irq();

    if (stop_sequence == s_uart_stop_serviced) {
        return 0U;
    }
    /* 队列溢出意味着字节流已经断裂；必须同时清掉此前已消费的半帧，
     * 否则后续一个 '?' 可能把旧的 CO=1 前缀拼成重新启动命令。 */
    Sguan_Printf_ResetReceive();
    User_CO_Adjust(0.0f);
    s_uart_stop_serviced = stop_sequence;
    return 1U;
}

void APP_UartCommandPoll(void)
{
    uint32_t now;

    (void)APP_UartServiceStop();

    now = HAL_GetTick();
    if ((s_uart_rx_restart_needed != 0U) &&
        ((int32_t)(now - s_uart_rx_next_retry_ms) >= 0)) {
        (void)HAL_UART_AbortReceive(&huart1);
        Sguan_Printf_ResetReceive();
        if (APP_UartRxStart() == 0U) {
            g_uart_rx_restart_fail_cnt++;
            s_uart_rx_next_retry_ms = now + 10U;
        }
    }

    while (s_uart_rx_tail != s_uart_rx_head) {
        uint8_t tail = s_uart_rx_tail;

        /* ISR 可在本轮 drain 期间发布新停机；一旦出现就先清队列/半帧，
         * 禁止继续解释停机事件之前已经排队的控制命令。 */
        if (APP_UartStopPending() != 0U) {
            (void)APP_UartServiceStop();
            break;
        }
        __DMB();
        if (APP_UartStopPending() != 0U) {
            (void)APP_UartServiceStop();
            break;
        }
        SguanFOC_Printf_Loop(s_uart_rx_queue[tail], s_uart_rx_length[tail]);
        if (APP_UartStopPending() != 0U) {
            (void)APP_UartServiceStop();
            break;
        }
        __DMB();
        s_uart_rx_tail = (uint8_t)((tail + 1U) % APP_UART_RX_QUEUE_DEPTH);
    }
}

/* ============================ FDCAN 回调 ================================== */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((hfdcan->Instance == FDCAN1) &&
        ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0U)) {
        CANFD_OnRxIsr();
    }
}
