/**
  ******************************************************************************
  * @file    canfd.c
  * @brief   GIM6010 多轴 FDCAN 协议实现（基础子集）
  * @note    - RX 中断先校验完整帧再入队；DIS 只做锁存并立即关闭 TIM1 MOE，
  *            其余语义解析全部留在 CANFD_Poll（主循环）。
  *          - [0..3] 目标随模式解释：开环 mV / 转矩 mA / 位置输出轴 µrad。
  ******************************************************************************
  */
#include "canfd.h"
#include "fdcan.h"
#include "SguanFOC.h"
#include "UserData_Config.h"
#include "UserData_Function.h"
#include "UserData_UserControl.h"
#include "main.h"
#include <string.h>

/* ============================ 内部状态 ==================================== */
#define CANFD_RX_QUEUE_DEPTH  (8U)

static uint8_t           s_rx_queue[CANFD_RX_QUEUE_DEPTH][8];
static volatile uint8_t  s_rx_head = 0U;
static volatile uint8_t  s_rx_tail = 0U;
static volatile uint32_t s_stop_sequence = 0U;
static uint32_t          s_stop_serviced = 0U;
static volatile uint32_t s_rx_drop_count = 0U;
static uint32_t          s_last_telem_ms = 0U;
static volatile uint8_t  s_online = 0U;

/* 连续位置在额定速度下很快会超出 int32 微弧度量程。所有遥测窄化转换
 * 必须先处理非有限值并饱和，禁止让越界 float->int 触发未定义行为。 */
static int32_t CANFD_EncodePositionUrad(float shaft_rad)
{
    double scaled;

    if (Value_isnan(shaft_rad) || Value_isinf(shaft_rad)) {
        return 0;
    }
    scaled = (double)shaft_rad * (double)CANFD_URAD_PER_RAD;
    if (scaled >= 2147483647.0) {
        return INT32_MAX;
    }
    if (scaled <= -2147483648.0) {
        return INT32_MIN;
    }
    return (int32_t)scaled;
}

static int16_t CANFD_EncodeRpm(float shaft_rpm)
{
    if (Value_isnan(shaft_rpm) || Value_isinf(shaft_rpm)) {
        return 0;
    }
    if (shaft_rpm >= 32767.0f) {
        return INT16_MAX;
    }
    if (shaft_rpm <= -32768.0f) {
        return INT16_MIN;
    }
    return (int16_t)shaft_rpm;
}

/* ============================ 初始化 ====================================== */
void CANFD_Init(void)
{
    FDCAN_FilterTypeDef filter;

    s_rx_head = 0U;
    s_rx_tail = 0U;
    s_stop_sequence = 0U;
    s_stop_serviced = 0U;
    s_rx_drop_count = 0U;
    s_online = 0U;

    /* 只接收本节点命令帧；广播同步帧滤波器后续迭代再加 */
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0U;
    filter.FilterType = FDCAN_FILTER_RANGE;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = CANFD_ID_CMD;
    filter.FilterID2 = CANFD_ID_CMD;
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK) {
        Error_Handler();
    }
    /* 非匹配帧全部拒收（标准/扩展），远程帧拒收 */
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        Error_Handler();
    }
}

/* ============================ 接收（中断） ================================ */
void CANFD_OnRxIsr(void)
{
    FDCAN_RxHeaderTypeDef header;
    uint8_t rx_data[8];

    while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0U) {
        uint16_t cmd;
        uint8_t head;
        uint8_t next;

        if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &header, rx_data) != HAL_OK) {
            break;
        }
        if ((header.Identifier != CANFD_ID_CMD) ||
            (header.IdType != FDCAN_STANDARD_ID) ||
            (header.RxFrameType != FDCAN_DATA_FRAME) ||
            (header.FDFormat != FDCAN_CLASSIC_CAN) ||
            (header.DataLength != FDCAN_DLC_BYTES_8)) {
            continue;
        }

        s_online = 1U;
        cmd = (uint16_t)((uint16_t)rx_data[6] | ((uint16_t)rx_data[7] << 8));
        if ((cmd & CANFD_CMD_DIS) != 0U) {
            /* 停机不能被后续目标覆盖，也不能等待阻塞对齐结束。ISR 只关硬件
             * 主输出并发布序号，控制器/状态清理仍由主循环完成。 */
            User_PwmOutputDisable();
            s_stop_sequence++;
            continue;
        }

        head = s_rx_head;
        next = (uint8_t)((head + 1U) % CANFD_RX_QUEUE_DEPTH);
        if (next == s_rx_tail) {
            /* 命令积压说明时序已不可控，采取失效安全停机而不是覆盖旧帧。 */
            s_rx_drop_count++;
            User_PwmOutputDisable();
            s_stop_sequence++;
            continue;
        }
        (void)memcpy(s_rx_queue[head], rx_data, sizeof(rx_data));
        __DMB();
        s_rx_head = next;
    }
}

uint8_t CANFD_IsStopPending(void)
{
    return (s_stop_sequence != s_stop_serviced) ? 1U : 0U;
}

uint8_t CANFD_SafetyPoll(void)
{
    uint32_t primask;
    uint32_t stop_sequence;

    primask = __get_PRIMASK();
    __disable_irq();
    stop_sequence = s_stop_sequence;
    if (stop_sequence != s_stop_serviced) {
        /* 丢弃停机前排队的目标，防止清理后被旧命令重新使能。 */
        s_rx_tail = s_rx_head;
    }
    if (primask == 0U) __enable_irq();

    if (stop_sequence == s_stop_serviced) {
        return 0U;
    }
    User_CO_Adjust(0.0f);
    s_stop_serviced = stop_sequence;
    return 1U;
}

void CANFD_BlockingSafetyPoll(void)
{
    uint32_t primask;

    (void)CANFD_SafetyPoll();
    /* 初始化/校准期间除 DIS 外的命令均无效；及时丢弃，避免主循环阻塞时队列溢出。 */
    primask = __get_PRIMASK();
    __disable_irq();
    s_rx_tail = s_rx_head;
    if (primask == 0U) __enable_irq();
}

/* ============================ 命令解析（主循环） =========================== */
static void CANFD_ParseCmd(const uint8_t *d)
{
    int32_t target_i32;
    int16_t speed_rpm;
    uint8_t start_only = 0U;
    uint16_t cmd = (uint16_t)((uint16_t)d[6] | ((uint16_t)d[7] << 8));

    /* DIS 优先于模式和所有保留位；防御性保留该分支，正常 DIS 已由 ISR 锁存。 */
    if ((cmd & CANFD_CMD_DIS) != 0U) {
        User_CO_Adjust(0.0f);
        return;  /* DIS 最高优先级；同帧的 EN/目标不得重新打开功率级。 */
    }
    if ((cmd & CANFD_CMD_RESERVED_MASK) != 0U) {
        return;
    }

    /* 1.模式位（与 VOFA BO 一致的四种模式） */
    User_BO_Adjust((float)((cmd & CANFD_CMD_MODE_MASK) >> CANFD_CMD_MODE_SHIFT));

    /* 2.控制字。带 EN/FAULT_CLEAR 的帧只负责启动或清故障；目标必须由
     * 主机在随后确认状态=IDLE 后单独发送，避免同帧重新打开功率桥。 */
    if ((cmd & CANFD_CMD_FAULT_CLEAR) != 0U) {
        User_CO_Adjust(1.0f);  /* 复用 UART 的条件检查、目标清零和安全重启路径。 */
        start_only = 1U;
    }
    if ((cmd & CANFD_CMD_EN) != 0U) {
        if (Sguan.status == MOTOR_STATUS_STANDBY) {
            User_CO_Adjust(1.0f);
        }
        start_only = 1U;
    }
    if (start_only != 0U) {
        return;
    }

    /* 3.模式目标。 */
    speed_rpm = (int16_t)((uint16_t)d[4] | ((uint16_t)d[5] << 8));
    (void)memcpy(&target_i32, d, 4);
    if (Sguan.mode == Velocity_OPEN_MODE) {
        User_VO_Adjust((float)speed_rpm / USER_RADS2RPM);
        User_AO_Adjust((float)target_i32 / CANFD_MILLI_PER_UNIT);
    } else if (Sguan.mode == Current_SINGLE_MODE) {
        User_AO_Adjust((float)target_i32 / CANFD_MILLI_PER_UNIT);
    } else if (Sguan.mode == VelCur_DOUBLE_MODE) {
        User_AO_Adjust((float)speed_rpm / USER_RADS2RPM);
    } else if (Sguan.mode == PosVelCur_THREE_MODE) {
        float shaft_rad = (float)target_i32 / CANFD_URAD_PER_RAD;
        User_AO_Adjust(shaft_rad * GIM6010_GEAR_RATIO);
    }
}

/* ============================ 遥测发送（主循环） =========================== */
void CANFD_Poll(void)
{
    uint32_t now;
    uint32_t primask;
    FDCAN_TxHeaderTypeDef tx;
    uint8_t data[8];
    uint8_t rx_data[8];
    uint8_t rx_ready;
    int32_t pos_urad;
    int16_t rpm;
    uint16_t stat;
    float shaft_rad;
    float shaft_rpm;

    /* 有停机先清目标；随后按接收顺序逐帧解析。每轮都复查停机序号，
     * User_PwmOutputEnable 也会在硬件使能前检查同一锁存，封住竞态窗口。 */
    if (CANFD_SafetyPoll() == 0U) {
        for (;;) {
            if (CANFD_SafetyPoll() != 0U) break;
            rx_ready = 0U;
            primask = __get_PRIMASK();
            __disable_irq();
            if (s_rx_tail != s_rx_head) {
                uint8_t tail = s_rx_tail;
                (void)memcpy(rx_data, s_rx_queue[tail], sizeof(rx_data));
                s_rx_tail = (uint8_t)((tail + 1U) % CANFD_RX_QUEUE_DEPTH);
                rx_ready = 1U;
            }
            if (primask == 0U) __enable_irq();
            if (rx_ready == 0U) break;
            CANFD_ParseCmd(rx_data);
        }
    }

    now = HAL_GetTick();
    if ((now - s_last_telem_ms) < CANFD_TELEM_PERIOD_MS) {
        return;
    }
    s_last_telem_ms = now;

    /* PLL 连续机械位置/速度（电机轴）→ 输出轴 */
    shaft_rad = (float)(Sguan.encoder.Real_Pos / GIM6010_GEAR_RATIO);
    shaft_rpm = Sguan.encoder.Real_Speed * USER_RADS2RPM / GIM6010_GEAR_RATIO;

    pos_urad = CANFD_EncodePositionUrad(shaft_rad);
    rpm = CANFD_EncodeRpm(shaft_rpm);
    stat = (uint16_t)(((uint16_t)Sguan.mode << 8) | (Sguan.status & 0xFFU));

    (void)memcpy(data, &pos_urad, 4);
    data[4] = (uint8_t)((uint16_t)rpm & 0xFFU);
    data[5] = (uint8_t)(((uint16_t)rpm >> 8) & 0xFFU);
    data[6] = (uint8_t)(stat & 0xFFU);
    data[7] = (uint8_t)((stat >> 8) & 0xFFU);

    tx.Identifier = CANFD_ID_TELEM;
    tx.IdType = FDCAN_STANDARD_ID;
    tx.TxFrameType = FDCAN_DATA_FRAME;
    tx.DataLength = FDCAN_DLC_BYTES_8;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch = FDCAN_BRS_OFF;
    tx.FDFormat = FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker = 0U;

    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0U) {
        (void)HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx, data);
    }
}

uint8_t CANFD_IsOnline(void)
{
    return s_online;
}

uint32_t CANFD_GetRxDropCount(void)
{
    return s_rx_drop_count;
}
