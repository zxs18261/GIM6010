/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-01-27 00:07:53
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-08-25
 * @FilePath: \SguanFOC_Debug\SguanFOC\Sguan_printf.c
 * @Description: USART1调试通信：TEXT事件/查询与JustFloat手动切换，共用唯一TX DMA
 *
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved.
 */
#include "Sguan_printf.h"
/* 外部函数文件声明 */
#include "Timer.h"                 /* Ch13使用20 kHz High_Loop计数换算实际发送频率。 */
#include "UserData_Function.h"
#include "UserData_UserControl.h"
#include "UserData_Calibration.h"
#include "BSP_ADC.h"
#include <stdio.h>
#include <string.h>

/* USART1 RX DMA原始缓冲；命令按问号终止符重新分帧。 */
uint8_t Sguan_PrintfBuff[200];

/* Ch13固定为JustFloat实际发送频率；首个成功帧按约定为0 Hz。 */
#define VOFA_TX_FREQUENCY_CHANNEL       (13U)
#define PRINTF_PROTOCOL_VERSION          (2U)

typedef struct {
    uint32_t sequence;
    uint32_t tick_ms;
    uint16_t code;
} PRINTF_EVENT_RECORD;

typedef struct {
    PRINTF_EVENT_CODE code;
    const char *ascii_code;
    const char *chinese_text;
    uint8_t is_fault;
} PRINTF_EVENT_DESCRIPTOR;

typedef enum {
    PRINTF_QUERY_NONE = 0,
    PRINTF_QUERY_HELP,
    PRINTF_QUERY_INFO,
    PRINTF_QUERY_CAL,
    PRINTF_QUERY_LOG,
    PRINTF_QUERY_STREAM
} PRINTF_QUERY_TYPE;

/* 事件字符串只在主循环查询，事件记录函数本身不访问这些文本。 */
static const PRINTF_EVENT_DESCRIPTOR s_event_descriptors[] = {
    {PRINTF_EVENT_BOOT, "BOOT", "固件启动", 0U},
    {PRINTF_EVENT_INIT_START, "INIT_START", "开始初始化", 0U},
    {PRINTF_EVENT_CAL_WAIT_VBUS, "CAL_WAIT_VBUS", "等待有效母线电压", 0U},
    {PRINTF_EVENT_CAL_FLASH_LOAD, "CAL_FLASH_LOAD", "读取Flash校准数据", 0U},
    {PRINTF_EVENT_CAL_START, "CAL_START", "正在预定位", 0U},
    {PRINTF_EVENT_CAL_PHASE_CHECK, "CAL_PHASE_CHECK", "正在检查相序方向", 0U},
    {PRINTF_EVENT_CAL_SAVE_OK, "CAL_SAVE_OK", "校准数据保存成功", 0U},
    {PRINTF_EVENT_CAL_ENCODER_ERROR, "CAL_ENCODER_ERROR", "校准时编码器数据无效", 1U},
    {PRINTF_EVENT_CAL_PHASE_ERROR, "CAL_PHASE_ERROR", "相序方向检查失败", 1U},
    {PRINTF_EVENT_CAL_FLASH_ERROR, "CAL_FLASH_ERROR", "校准Flash保存或回读失败", 1U},
    {PRINTF_EVENT_IDLE, "IDLE", "进入安全待机", 0U},
    {PRINTF_EVENT_STANDBY, "STANDBY", "进入停机状态", 0U},
    {PRINTF_EVENT_MODE_OPEN, "MODE_OPEN", "进入开环FOC模式", 0U},
    {PRINTF_EVENT_MODE_CURRENT, "MODE_CURRENT", "进入电流环模式", 0U},
    {PRINTF_EVENT_MODE_SPEED, "MODE_SPEED", "进入速度电流双环模式", 0U},
    {PRINTF_EVENT_MODE_POSITION, "MODE_POSITION", "进入位置速度电流三环模式", 0U},
    {PRINTF_EVENT_STREAM_STARTING, "STREAM_STARTING", "正在启动JustFloat", 0U},
    {PRINTF_EVENT_STREAM_ON, "STREAM_ON", "JustFloat已经启动", 0U},
    {PRINTF_EVENT_STREAM_STOPPING, "STREAM_STOPPING", "正在停止JustFloat", 0U},
    {PRINTF_EVENT_STREAM_OFF, "STREAM_OFF", "已经恢复TEXT模式", 0U},
    {PRINTF_EVENT_CMD_AO_ACCEPTED, "CMD_AO_OK", "AO命令已接受", 0U},
    {PRINTF_EVENT_CMD_BO_ACCEPTED, "CMD_BO_OK", "BO命令已接受", 0U},
    {PRINTF_EVENT_CMD_CO_ACCEPTED, "CMD_CO_OK", "CO命令已接受", 0U},
    {PRINTF_EVENT_CMD_VO_ACCEPTED, "CMD_VO_OK", "VO命令已接受", 0U},
    {PRINTF_EVENT_CMD_REJECTED, "CMD_REJECTED", "命令已拒绝且未改变控制量", 0U},
    {PRINTF_EVENT_QUERY_REJECTED_STREAM, "QUERY_REJECTED_STREAM", "JustFloat期间拒绝文本查询", 0U},
    {PRINTF_EVENT_FAULT_OVERVOLTAGE, "OVERVOLTAGE", "母线过压", 1U},
    {PRINTF_EVENT_FAULT_UNDERVOLTAGE, "UNDERVOLTAGE", "母线欠压", 1U},
    {PRINTF_EVENT_FAULT_OVERTEMPERATURE, "OVERTEMPERATURE", "驱动器过温", 1U},
    {PRINTF_EVENT_FAULT_UNDERTEMPERATURE, "UNDERTEMPERATURE", "驱动器低温", 1U},
    {PRINTF_EVENT_FAULT_OVERCURRENT, "OVERCURRENT", "电机过流", 1U},
    {PRINTF_EVENT_FAULT_ENCODER, "ENCODER_ERROR", "编码器故障", 1U},
    {PRINTF_EVENT_FAULT_SENSOR, "SENSOR_ERROR", "传感器故障", 1U},
    {PRINTF_EVENT_FAULT_PWM, "PWM_FAULT", "PWM计算故障", 1U},
    {PRINTF_EVENT_FAULT_EMERGENCY_STOP, "EMERGENCY_STOP", "急停触发", 1U},
    {PRINTF_EVENT_FAULT_DISABLED, "DISABLED", "控制器已失能", 1U}
};

/* UART TX DMA只能读取稳定缓冲；文本和JustFloat均由本文件统一拥有。 */
static PRINTF_STRUCT s_vofa_tx_buf[2];
static uint8_t s_text_tx_buf[USER_DEBUG_TEXT_BUFFER_SIZE];
static uint8_t s_printf_debug_char = 0U;
static volatile uint8_t s_tx_busy = 0U;
static uint8_t s_vofa_tx_index = 0U;
static uint32_t s_vofa_last_tick = 0U;
static volatile uint32_t s_vofa_tx_drop_cnt = 0U;
static uint32_t s_vofa_last_tx_high_loop_count = 0U;
static uint8_t s_vofa_tx_frequency_valid = 0U;
static volatile PRINTF_STREAM_STATE s_stream_state = PRINTF_STREAM_TEXT;
static uint8_t s_comm_initialized = 0U;

/* RAM事件环只保存本次上电数据；sequence用于覆盖后仍能发现缺口。 */
static PRINTF_EVENT_RECORD s_event_ring[USER_DEBUG_EVENT_CAPACITY];
static volatile uint16_t s_event_head = 0U;
static volatile uint16_t s_event_count = 0U;
static volatile uint32_t s_event_sequence = 0U;
static volatile uint32_t s_event_overflow_count = 0U;
static uint32_t s_live_next_sequence = 1U;

/* 查询由RX中断只提交枚举，所有文本和Flash诊断均在主循环生成。 */
static volatile PRINTF_QUERY_TYPE s_pending_query = PRINTF_QUERY_NONE;
static uint8_t s_query_line = 0U;
static USER_CALIBRATION_DIAGNOSTIC s_query_calibration;
static uint8_t s_log_started = 0U;
static uint32_t s_log_next_sequence = 0U;
static uint32_t s_log_end_sequence = 0U;
static uint16_t s_log_snapshot_count = 0U;
static uint32_t s_log_snapshot_overflow = 0U;

/* Receive-to-Idle边界不等于命令边界，按问号重新组装完整命令。 */
static uint8_t s_rx_command[USER_DEBUG_RX_COMMAND_MAX];
static uint16_t s_rx_command_length = 0U;
static uint8_t s_rx_discard_until_end = 0U;

static uint8_t Get_Data(const uint8_t *data, uint16_t length, float *data_return);
static uint8_t Printf_TxStart(uint8_t *data, uint16_t length);
static const PRINTF_EVENT_DESCRIPTOR *Printf_EventDescriptorGet(uint16_t code);
static uint8_t Printf_EventCopyBySequence(uint32_t sequence,
                                          PRINTF_EVENT_RECORD *event);
static uint8_t Printf_TextSendEvent(const PRINTF_EVENT_RECORD *event);
static uint8_t Printf_TextSendNextLiveEvent(void);
static uint8_t Printf_QuerySchedule(PRINTF_QUERY_TYPE query);
static void Printf_QueryService(void);
static void Printf_LogService(void);
static const char *Printf_StreamStateName(PRINTF_STREAM_STATE state);
static void Printf_StreamStateService(void);
static void Printf_RecordStatusEvent(uint8_t status);
static void Printf_RecordModeEvent(uint8_t mode);

/* ================= 重定向设计 BEGIN ================= */
// 支持printf函数，而无需选择MicroLIB
#if 0
#pragma import(__use_no_semihosting)
struct __FILE{
    int handle;
};
FILE __stdout;
void _sys_exit(int x){
    x = x;
}
#endif

// Debug printf仅在TEXT空闲时发送单字节，不能插入JustFloat或覆盖DMA缓冲。
int fputc(int ch,FILE *f){
    (void)f;
    if ((s_stream_state == PRINTF_STREAM_TEXT) &&
        (s_tx_busy == 0U) && (huart1.gState == HAL_UART_STATE_READY)) {
        s_printf_debug_char = (uint8_t)ch;
        (void)Printf_TxStart(&s_printf_debug_char, 1U);
    }
    return ch;
}
/* ================= 重定向设计 END ================= */

/*
 * [接收]严格解析十进制浮点数，格式：AO=13.14?
 * 只接受可选正负号、数字和一个小数点；拒绝%f、NaN、Inf、指数及残缺命令。
 * 返回0时调用方不得修改任何控制量。
 */
static uint8_t Get_Data(const uint8_t *data, uint16_t length, float *data_return){
    const float data_abs_max = 1000000.0f;
    float value = 0.0f;
    float decimal_weight = 0.1f;
    uint16_t index = 3U;
    const uint16_t end = (length > 0U) ? (uint16_t)(length - 1U) : 0U;
    uint8_t minus_flag = 0U;
    uint8_t decimal_flag = 0U;
    uint8_t integer_digit_seen = 0U;
    uint8_t decimal_digit_seen = 0U;

    if ((data == NULL) || (data_return == NULL) ||
        (length < 5U) || (length > USER_DEBUG_RX_COMMAND_MAX) ||
        (data[2] != '=') || (data[end] != '?')) {
        return 0U;
    }

    if ((data[index] == '-') || (data[index] == '+')) {
        minus_flag = (data[index] == '-') ? 1U : 0U;
        index++;
    }

    for (; index < end; index++) {
        uint8_t ch = data[index];
        if (ch == '.') {
            if ((decimal_flag != 0U) || (integer_digit_seen == 0U)) {
                return 0U;
            }
            decimal_flag = 1U;
            continue;
        }

        if ((ch < '0') || (ch > '9')) {
            return 0U;
        }

        if (decimal_flag == 0U) {
            float digit = (float)(ch - '0');
            integer_digit_seen = 1U;
            if (value > ((data_abs_max - digit) * 0.1f)) {
                return 0U;
            }
            value = value * 10.0f + digit;
        } else {
            decimal_digit_seen = 1U;
            value += (float)(ch - '0') * decimal_weight;
            if (value > data_abs_max) {
                return 0U;
            }
            decimal_weight *= 0.1f;
        }
    }

    if ((integer_digit_seen == 0U) ||
        ((decimal_flag != 0U) && (decimal_digit_seen == 0U))) {
        return 0U;
    }

    *data_return = (minus_flag != 0U) ? -value : value;
    return 1U;
}

static uint8_t Printf_CommandEqual(const uint8_t *data,
                                   uint16_t length,
                                   const char *command)
{
    size_t command_length;

    if ((data == NULL) || (command == NULL)) {
        return 0U;
    }
    command_length = strlen(command);
    if ((size_t)length != command_length) {
        return 0U;
    }
    return (memcmp(data, command, command_length) == 0) ? 1U : 0U;
}

static uint8_t Printf_TxStart(uint8_t *data, uint16_t length)
{
    if ((data == NULL) || (length == 0U) ||
        (s_tx_busy != 0U) || (huart1.gState != HAL_UART_STATE_READY)) {
        return 0U;
    }

    s_tx_busy = 1U;
    if (User_CorrespondSet(data, length) == HAL_OK) {
        return 1U;
    }
    s_tx_busy = 0U;
    s_vofa_tx_drop_cnt++;
    return 0U;
}

static uint16_t Printf_TextLengthGet(int formatted_length)
{
    if (formatted_length <= 0) {
        return 0U;
    }
    if ((uint32_t)formatted_length >= USER_DEBUG_TEXT_BUFFER_SIZE) {
        /* 截断时保留字符串末尾，禁止DMA越过静态文本缓冲区。 */
        return (uint16_t)(USER_DEBUG_TEXT_BUFFER_SIZE - 1U);
    }
    return (uint16_t)formatted_length;
}

// [初始化]初始化JustFloat帧尾，并仅在MCU本次启动首次调用时初始化通信状态。
void Printf_Init(PRINTF_STRUCT *str){
    if (str == NULL) {
        return;
    }
    str->tail[0] = 0x00;
    str->tail[1] = 0x00;
    str->tail[2] = 0x80;
    str->tail[3] = 0x7f;

    if (s_comm_initialized == 0U) {
        s_stream_state = PRINTF_STREAM_TEXT;
        s_pending_query = PRINTF_QUERY_NONE;
        s_comm_initialized = 1U;
        Printf_EventRecord(PRINTF_EVENT_BOOT);
    }
}

/* 仅JustFloat状态使用：到期且UART/DMA空闲时才构造一帧，不补发过期帧。 */
uint8_t Printf_TxSlotDue(void){
    uint32_t now = HAL_GetTick();

    if (s_stream_state != PRINTF_STREAM_JUSTFLOAT) {
        return 0U;
    }
    if ((uint32_t)(now - s_vofa_last_tick) < USER_VOFA_TX_PERIOD_MS) {
        return 0U;
    }
    s_vofa_last_tick = now;
    if ((s_tx_busy != 0U) || (huart1.gState != HAL_UART_STATE_READY)) {
        s_vofa_tx_drop_cnt++;
        return 0U;
    }
    return 1U;
}

/* UART TX DMA完成ISR只释放所有权，不在中断里格式化或续发。 */
void Printf_TxCpltISR(void){
    s_tx_busy = 0U;
}

/* UART TX DMA错误/中止时释放所有权，避免通信永久停在busy。 */
void Printf_TxErrorISR(void){
    if (s_tx_busy != 0U) {
        s_tx_busy = 0U;
        s_vofa_tx_drop_cnt++;
    }
}

uint32_t Printf_GetTxDropCount(void){
    return s_vofa_tx_drop_cnt;
}

uint32_t Printf_GetEventOverflowCount(void){
    return s_event_overflow_count;
}

PRINTF_STREAM_STATE Printf_GetStreamState(void){
    return s_stream_state;
}

// [发送]JustFloat DMA双缓冲发送；调用方须先通过Printf_TxSlotDue()取得时隙。
void Printf_Loop(const PRINTF_STRUCT *str){
    PRINTF_STRUCT *tx_buf;
    uint32_t current_high_loop_count;
    uint32_t high_loop_delta;
    float vofa_tx_frequency_hz = 0.0f;

    if ((str == NULL) || (s_stream_state != PRINTF_STREAM_JUSTFLOAT)) {
        return;
    }
    if ((s_tx_busy != 0U) || (huart1.gState != HAL_UART_STATE_READY)) {
        s_vofa_tx_drop_cnt++;
        return;
    }

    tx_buf = &s_vofa_tx_buf[s_vofa_tx_index];
    memcpy(tx_buf, str, sizeof(*tx_buf));
    /*
     * 用相邻两次成功DMA启动之间的20 kHz计数差计算真实帧率。
     * 统计放在发送层，避免把构造失败或HAL_BUSY误算成已发送。
     */
    current_high_loop_count = g_high_loop_cnt;
    high_loop_delta = current_high_loop_count - s_vofa_last_tx_high_loop_count;
    if ((s_vofa_tx_frequency_valid != 0U) && (high_loop_delta != 0U)) {
        vofa_tx_frequency_hz = (float)USER_FOC_CONTROL_HZ /
                               (float)high_loop_delta;
    }
    tx_buf->fdata[VOFA_TX_FREQUENCY_CHANNEL] = vofa_tx_frequency_hz;

    if (Printf_TxStart((uint8_t *)tx_buf, (uint16_t)sizeof(*tx_buf)) != 0U) {
        s_vofa_last_tx_high_loop_count = current_high_loop_count;
        s_vofa_tx_frequency_valid = 1U;
        s_vofa_tx_index ^= 1U;
    }
}

void Printf_EventRecord(PRINTF_EVENT_CODE code)
{
    PRINTF_EVENT_RECORD event;
    uint32_t primask;

    event.tick_ms = HAL_GetTick();
    event.code = (uint16_t)code;

    /* 事件可能由UART/控制ISR提交；临界区只复制固定长度记录。 */
    primask = __get_PRIMASK();
    __disable_irq();
    s_event_sequence++;
    event.sequence = s_event_sequence;
    s_event_ring[s_event_head] = event;
    s_event_head = (uint16_t)((s_event_head + 1U) % USER_DEBUG_EVENT_CAPACITY);
    if (s_event_count < USER_DEBUG_EVENT_CAPACITY) {
        s_event_count++;
    } else {
        s_event_overflow_count++;
    }
    if (primask == 0U) {
        __enable_irq();
    }
}

static const PRINTF_EVENT_DESCRIPTOR *Printf_EventDescriptorGet(uint16_t code)
{
    uint32_t index;

    for (index = 0U;
         index < (sizeof(s_event_descriptors) / sizeof(s_event_descriptors[0]));
         index++) {
        if ((uint16_t)s_event_descriptors[index].code == code) {
            return &s_event_descriptors[index];
        }
    }
    return NULL;
}

static uint8_t Printf_EventCopyBySequence(uint32_t sequence,
                                          PRINTF_EVENT_RECORD *event)
{
    uint32_t primask;
    uint16_t count;
    uint16_t oldest_index;
    uint32_t oldest_sequence;
    uint32_t newest_sequence;
    uint32_t offset;

    if (event == NULL) {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    count = s_event_count;
    if (count == 0U) {
        if (primask == 0U) {
            __enable_irq();
        }
        return 0U;
    }

    oldest_index = (uint16_t)((s_event_head + USER_DEBUG_EVENT_CAPACITY - count) %
                              USER_DEBUG_EVENT_CAPACITY);
    oldest_sequence = s_event_ring[oldest_index].sequence;
    newest_sequence = s_event_sequence;
    if (sequence < oldest_sequence) {
        /* 请求序号已被覆盖时从当前最旧记录继续，并由overflow字段说明缺口。 */
        sequence = oldest_sequence;
    }
    if (sequence > newest_sequence) {
        if (primask == 0U) {
            __enable_irq();
        }
        return 0U;
    }

    offset = sequence - oldest_sequence;
    if (offset >= count) {
        if (primask == 0U) {
            __enable_irq();
        }
        return 0U;
    }
    *event = s_event_ring[(oldest_index + (uint16_t)offset) %
                          USER_DEBUG_EVENT_CAPACITY];
    if (primask == 0U) {
        __enable_irq();
    }
    return 1U;
}

static uint8_t Printf_TextSendEvent(const PRINTF_EVENT_RECORD *event)
{
    const PRINTF_EVENT_DESCRIPTOR *descriptor;
    const char *message_type;
    int length;

    if ((event == NULL) || (s_stream_state != PRINTF_STREAM_TEXT)) {
        return 0U;
    }
    descriptor = Printf_EventDescriptorGet(event->code);
    if (descriptor == NULL) {
        length = snprintf((char *)s_text_tx_buf, sizeof(s_text_tx_buf),
                          "@EVT,%lu,UNKNOWN_EVENT,未知事件码%u\r\n",
                          (unsigned long)event->tick_ms,
                          (unsigned int)event->code);
    } else {
        message_type = (descriptor->is_fault != 0U) ? "@FAULT" : "@EVT";
        length = snprintf((char *)s_text_tx_buf, sizeof(s_text_tx_buf),
                          "%s,%lu,%s,%s\r\n",
                          message_type,
                          (unsigned long)event->tick_ms,
                          descriptor->ascii_code,
                          descriptor->chinese_text);
    }
    return Printf_TxStart(s_text_tx_buf, Printf_TextLengthGet(length));
}

static uint8_t Printf_TextSendNextLiveEvent(void)
{
    PRINTF_EVENT_RECORD event;

    if (Printf_EventCopyBySequence(s_live_next_sequence, &event) == 0U) {
        return 0U;
    }
    if (Printf_TextSendEvent(&event) == 0U) {
        return 0U;
    }
    s_live_next_sequence = event.sequence + 1U;
    return 1U;
}

static uint8_t Printf_QuerySchedule(PRINTF_QUERY_TYPE query)
{
    uint32_t primask;
    uint8_t accepted = 0U;

    primask = __get_PRIMASK();
    __disable_irq();
    if (s_pending_query == PRINTF_QUERY_NONE) {
        s_pending_query = query;
        s_query_line = 0U;
        s_log_started = 0U;
        accepted = 1U;
    }
    if (primask == 0U) {
        __enable_irq();
    }
    return accepted;
}

static const char *Printf_StreamStateName(PRINTF_STREAM_STATE state)
{
    switch (state) {
    case PRINTF_STREAM_TEXT:
        return "TEXT";
    case PRINTF_STREAM_STARTING:
        return "STARTING";
    case PRINTF_STREAM_JUSTFLOAT:
        return "JUSTFLOAT";
    case PRINTF_STREAM_STOPPING:
        return "STOPPING";
    default:
        return "UNKNOWN";
    }
}

static uint8_t Printf_QueryTextSend(int length)
{
    uint16_t tx_length = Printf_TextLengthGet(length);

    if (tx_length == 0U) {
        return 0U;
    }
    return Printf_TxStart(s_text_tx_buf, tx_length);
}

static void Printf_HelpService(void)
{
    int length = 0;

    switch (s_query_line) {
    case 0U:
        length = snprintf((char *)s_text_tx_buf, sizeof(s_text_tx_buf),
                          "@RSP,%lu,HELP_QUERY,HELP?|INFO?|CAL?|LOG?|STREAM?,只读查询命令\r\n",
                          (unsigned long)HAL_GetTick());
        break;
    case 1U:
        length = snprintf((char *)s_text_tx_buf, sizeof(s_text_tx_buf),
                          "@RSP,%lu,HELP_STREAM,STREAM=1?|STREAM=0?,手动启停JustFloat且不改变电机控制量\r\n",
                          (unsigned long)HAL_GetTick());
        break;
    case 2U:
        length = snprintf((char *)s_text_tx_buf, sizeof(s_text_tx_buf),
                          "@RSP,%lu,HELP_CONTROL,AO/BO/CO/VO=<decimal>?,速度统一为输出轴rad/s\r\n",
                          (unsigned long)HAL_GetTick());
        break;
    default:
        length = snprintf((char *)s_text_tx_buf, sizeof(s_text_tx_buf),
                          "@RSP,%lu,HELP_SAFETY,先确认status=4且Ud=Uq=MOE=0再启动数据流,调试安全提示\r\n",
                          (unsigned long)HAL_GetTick());
        break;
    }

    if (Printf_QueryTextSend(length) != 0U) {
        s_query_line++;
        if (s_query_line >= 4U) {
            s_pending_query = PRINTF_QUERY_NONE;
        }
    }
}

static void Printf_InfoService(void)
{
    float vbus = User_VBUS_DataGet();
    int length = 0;

    if (s_query_line == 0U) {
        (void)User_Calibration_ReadDiagnostic(&s_query_calibration);
    }

    switch (s_query_line) {
    case 0U:
        length = snprintf((char *)s_text_tx_buf, sizeof(s_text_tx_buf),
                          "@RSP,%lu,INFO_SYS,fw=GIM6010_V0.0,protocol=%u,baud=921600,status=%u,mode=%u,stream=%s,vbus_v=%.6f,moe=%u,ud_v=%.6f,uq_v=%.6f,系统状态\r\n",
                          (unsigned long)HAL_GetTick(),
                          (unsigned int)PRINTF_PROTOCOL_VERSION,
                          (unsigned int)Sguan.status,
                          (unsigned int)Sguan.mode,
                          Printf_StreamStateName(s_stream_state),
                          (double)vbus,
                          (unsigned int)User_PwmOutputIsEnabled(),
                          (double)Sguan.foc.Ud_in,
                          (double)Sguan.foc.Uq_in);
        break;
    case 1U:
        length = snprintf((char *)s_text_tx_buf, sizeof(s_text_tx_buf),
                          "@RSP,%lu,INFO_MOTOR,poles=%u,gear_ratio=%.3f,rated_current_a=%.3f,rated_torque_nm=%.3f,peak_torque_nm=%.3f,电机固定参数\r\n",
                          (unsigned long)HAL_GetTick(),
                          (unsigned int)Sguan.motor.Poles,
                          (double)GIM6010_GEAR_RATIO,
                          (double)GIM6010_RATED_CURRENT_A,
                          (double)GIM6010_OUTPUT_RATED_TORQUE_NM,
                          (double)GIM6010_OUTPUT_PEAK_TORQUE_NM);
        break;
    case 2U:
        length = snprintf((char *)s_text_tx_buf, sizeof(s_text_tx_buf),
                          "@RSP,%lu,INFO_LIMITS,shaft_rated_rad_s=%.6f,shaft_max_rad_s=%.6f,openloop_uq_max_v=%.6f,current_limit_a=%.3f,限幅参数\r\n",
                          (unsigned long)HAL_GetTick(),
                          (double)GIM6010_SHAFT_RATED_RAD_S,
                          (double)GIM6010_SHAFT_MAX_RAD_S,
                          (double)(USER_OPENLOOP_UQ_MAX_RATIO * Sguan.motor.VBUS),
                          (double)GIM6010_RATED_CURRENT_A);
        break;
    default:
        length = snprintf((char *)s_text_tx_buf, sizeof(s_text_tx_buf),
                          "@RSP,%lu,INFO_COMM,channels=%u,frame_bytes=%u,target_hz=%u,tx_drop=%lu,log_overflow=%lu,fw_signature=%08lX,通信统计\r\n",
                          (unsigned long)HAL_GetTick(),
                          (unsigned int)CH_COUNT,
                          (unsigned int)sizeof(PRINTF_STRUCT),
                          (unsigned int)USER_VOFA_TX_HZ,
                          (unsigned long)s_vofa_tx_drop_cnt,
                          (unsigned long)s_event_overflow_count,
                          (unsigned long)s_query_calibration.calculated_firmware_signature);
        break;
    }

    if (Printf_QueryTextSend(length) != 0U) {
        s_query_line++;
        if (s_query_line >= 4U) {
            s_pending_query = PRINTF_QUERY_NONE;
        }
    }
}

static void Printf_CalService(void)
{
    int length = 0;

    if (s_query_line == 0U) {
        (void)User_Calibration_ReadDiagnostic(&s_query_calibration);
    }

    switch (s_query_line) {
    case 0U:
        length = snprintf((char *)s_text_tx_buf, sizeof(s_text_tx_buf),
                          "@RSP,%lu,CAL_META,valid=%u,address=%08lX,format_expected=%08lX,format_stored=%08lX,signature_calc=%08lX,signature_stored=%08lX,checksum_calc=%08lX,checksum_stored=%08lX,commit=%u,Flash校验结果\r\n",
                          (unsigned long)HAL_GetTick(),
                          (unsigned int)s_query_calibration.record_valid,
                          (unsigned long)s_query_calibration.flash_address,
                          (unsigned long)s_query_calibration.expected_format_version,
                          (unsigned long)s_query_calibration.stored_format_version,
                          (unsigned long)s_query_calibration.calculated_firmware_signature,
                          (unsigned long)s_query_calibration.stored_firmware_signature,
                          (unsigned long)s_query_calibration.calculated_checksum,
                          (unsigned long)s_query_calibration.stored_checksum,
                          (unsigned int)s_query_calibration.commit_valid);
        break;
    case 1U:
        if (s_query_calibration.data_valid != 0U) {
            length = snprintf((char *)s_text_tx_buf, sizeof(s_text_tx_buf),
                              "@RSP,%lu,CAL_DATA,available=1,offset_rad=%.6f,encoder_dir=%d,motor_dir=%d,pwm_dir=%d,current_dir0=%d,current_dir1=%d,current_num=%u,poles=%u,校准参数\r\n",
                              (unsigned long)HAL_GetTick(),
                              (double)s_query_calibration.data.position_offset_rad,
                              (int)s_query_calibration.data.encoder_dir,
                              (int)s_query_calibration.data.motor_dir,
                              (int)s_query_calibration.data.pwm_dir,
                              (int)s_query_calibration.data.current_dir0,
                              (int)s_query_calibration.data.current_dir1,
                              (unsigned int)s_query_calibration.data.current_num,
                              (unsigned int)s_query_calibration.data.pole_pairs);
        } else {
            length = snprintf((char *)s_text_tx_buf, sizeof(s_text_tx_buf),
                              "@RSP,%lu,CAL_DATA,available=0,magic=%u,format=%u,firmware=%u,checksum=%u,commit=%u,data=%u,Flash数据不可应用\r\n",
                              (unsigned long)HAL_GetTick(),
                              (unsigned int)s_query_calibration.magic_valid,
                              (unsigned int)s_query_calibration.format_valid,
                              (unsigned int)s_query_calibration.firmware_valid,
                              (unsigned int)s_query_calibration.checksum_valid,
                              (unsigned int)s_query_calibration.commit_valid,
                              (unsigned int)s_query_calibration.data_valid);
        }
        break;
    default:
        length = snprintf((char *)s_text_tx_buf, sizeof(s_text_tx_buf),
                          "@RSP,%lu,CAL_RUNTIME,result=%u,flash_valid=%u,align_ud_v=%.6f,phase_check=%d,当前上电校准状态\r\n",
                          (unsigned long)HAL_GetTick(),
                          (unsigned int)Sguan.calibration.result,
                          (unsigned int)Sguan.calibration.flash_valid,
                          (double)Sguan.calibration.align_ud_v,
                          (int)Sguan.calibration.phase_check_result);
        break;
    }

    if (Printf_QueryTextSend(length) != 0U) {
        s_query_line++;
        if (s_query_line >= 3U) {
            s_pending_query = PRINTF_QUERY_NONE;
        }
    }
}

static void Printf_LogService(void)
{
    PRINTF_EVENT_RECORD event;
    int length;

    if (s_log_started == 0U) {
        uint32_t primask = __get_PRIMASK();
        uint16_t count;
        uint16_t oldest_index;

        __disable_irq();
        count = s_event_count;
        s_log_snapshot_count = count;
        s_log_snapshot_overflow = s_event_overflow_count;
        s_log_end_sequence = s_event_sequence;
        if (count > 0U) {
            oldest_index = (uint16_t)((s_event_head + USER_DEBUG_EVENT_CAPACITY - count) %
                                      USER_DEBUG_EVENT_CAPACITY);
            s_log_next_sequence = s_event_ring[oldest_index].sequence;
        } else {
            s_log_next_sequence = s_log_end_sequence + 1U;
        }
        if (primask == 0U) {
            __enable_irq();
        }

        length = snprintf((char *)s_text_tx_buf, sizeof(s_text_tx_buf),
                          "@RSP,%lu,LOG_BEGIN,count=%u,overflow=%lu,事件回放开始\r\n",
                          (unsigned long)HAL_GetTick(),
                          (unsigned int)s_log_snapshot_count,
                          (unsigned long)s_log_snapshot_overflow);
        if (Printf_QueryTextSend(length) != 0U) {
            s_log_started = 1U;
        }
        return;
    }

    if (s_log_next_sequence <= s_log_end_sequence) {
        if (Printf_EventCopyBySequence(s_log_next_sequence, &event) != 0U) {
            if (Printf_TextSendEvent(&event) != 0U) {
                s_log_next_sequence = event.sequence + 1U;
            }
            return;
        }
        /* 快照中的记录若被新事件覆盖，结束本次回放并保留overflow证据。 */
        s_log_next_sequence = s_log_end_sequence + 1U;
    }

    length = snprintf((char *)s_text_tx_buf, sizeof(s_text_tx_buf),
                      "@RSP,%lu,LOG_END,count=%u,overflow=%lu,事件回放结束\r\n",
                      (unsigned long)HAL_GetTick(),
                      (unsigned int)s_log_snapshot_count,
                      (unsigned long)s_log_snapshot_overflow);
    if (Printf_QueryTextSend(length) != 0U) {
        if (s_live_next_sequence <= s_log_end_sequence) {
            s_live_next_sequence = s_log_end_sequence + 1U;
        }
        s_log_started = 0U;
        s_pending_query = PRINTF_QUERY_NONE;
    }
}

static void Printf_StreamQueryService(void)
{
    int length = snprintf((char *)s_text_tx_buf, sizeof(s_text_tx_buf),
                          "@RSP,%lu,STREAM,state=%s,channels=%u,target_hz=%u,数据流状态\r\n",
                          (unsigned long)HAL_GetTick(),
                          Printf_StreamStateName(s_stream_state),
                          (unsigned int)CH_COUNT,
                          (unsigned int)USER_VOFA_TX_HZ);

    if (Printf_QueryTextSend(length) != 0U) {
        s_pending_query = PRINTF_QUERY_NONE;
    }
}

static void Printf_QueryService(void)
{
    if ((s_pending_query == PRINTF_QUERY_NONE) || (s_tx_busy != 0U) ||
        (s_stream_state != PRINTF_STREAM_TEXT)) {
        return;
    }

    switch (s_pending_query) {
    case PRINTF_QUERY_HELP:
        Printf_HelpService();
        break;
    case PRINTF_QUERY_INFO:
        Printf_InfoService();
        break;
    case PRINTF_QUERY_CAL:
        Printf_CalService();
        break;
    case PRINTF_QUERY_LOG:
        Printf_LogService();
        break;
    case PRINTF_QUERY_STREAM:
        Printf_StreamQueryService();
        break;
    default:
        s_pending_query = PRINTF_QUERY_NONE;
        break;
    }
}

static void Printf_StreamStateService(void)
{
    /* 故障和重新初始化自动停止二进制流；普通STANDBY保留流用于观察停机动态。 */
    if (((Sguan.status >= MOTOR_STATUS_OVERVOLTAGE) ||
         ((Sguan.status >= MOTOR_STATUS_UNINITIALIZED) &&
          (Sguan.status <= MOTOR_STATUS_CALIBRATING))) &&
        ((s_stream_state == PRINTF_STREAM_STARTING) ||
         (s_stream_state == PRINTF_STREAM_JUSTFLOAT))) {
        s_stream_state = PRINTF_STREAM_STOPPING;
        Printf_EventRecord(PRINTF_EVENT_STREAM_STOPPING);
    }

    if ((s_stream_state == PRINTF_STREAM_STARTING) && (s_tx_busy == 0U)) {
        /* 确认上一条文本DMA已经结束后再发二进制，禁止两种格式交叉。 */
        s_stream_state = PRINTF_STREAM_JUSTFLOAT;
        s_vofa_last_tick = HAL_GetTick() - USER_VOFA_TX_PERIOD_MS;
        s_vofa_tx_frequency_valid = 0U;
        Printf_EventRecord(PRINTF_EVENT_STREAM_ON);
    } else if ((s_stream_state == PRINTF_STREAM_STOPPING) && (s_tx_busy == 0U)) {
        /* 等最后一帧DMA结束后才恢复TEXT，禁止覆盖正在发送的JustFloat缓冲。 */
        s_stream_state = PRINTF_STREAM_TEXT;
        s_pending_query = PRINTF_QUERY_NONE;
        s_query_line = 0U;
        s_log_started = 0U;
        Printf_EventRecord(PRINTF_EVENT_STREAM_OFF);
    }
}

void Printf_BackgroundService(void)
{
    Printf_StreamStateService();
    if ((s_stream_state != PRINTF_STREAM_TEXT) || (s_tx_busy != 0U)) {
        return;
    }

    /* 主动查询优先于自动事件，查询结束后继续发送尚未实时输出的事件。 */
    if (s_pending_query != PRINTF_QUERY_NONE) {
        Printf_QueryService();
        return;
    }
    (void)Printf_TextSendNextLiveEvent();
}

void Printf_MainLoop(const PRINTF_STRUCT *str)
{
    Printf_BackgroundService();
    if ((s_stream_state == PRINTF_STREAM_JUSTFLOAT) &&
        (Printf_TxSlotDue() != 0U)) {
        /* 只在发送时隙到达后采集14个通道，避免主循环无意义重复打包。 */
        User_UserTX();
        Printf_Loop(str);
    }
}

static void Printf_RecordStatusEvent(uint8_t status)
{
    switch (status) {
    case MOTOR_STATUS_STANDBY:
        Printf_EventRecord(PRINTF_EVENT_STANDBY);
        break;
    case MOTOR_STATUS_IDLE:
        Printf_EventRecord(PRINTF_EVENT_IDLE);
        break;
    case MOTOR_STATUS_OVERVOLTAGE:
        Printf_EventRecord(PRINTF_EVENT_FAULT_OVERVOLTAGE);
        break;
    case MOTOR_STATUS_UNDERVOLTAGE:
        Printf_EventRecord(PRINTF_EVENT_FAULT_UNDERVOLTAGE);
        break;
    case MOTOR_STATUS_OVERTEMPERATURE:
        Printf_EventRecord(PRINTF_EVENT_FAULT_OVERTEMPERATURE);
        break;
    case MOTOR_STATUS_UNDERTEMPERATURE:
        Printf_EventRecord(PRINTF_EVENT_FAULT_UNDERTEMPERATURE);
        break;
    case MOTOR_STATUS_OVERCURRENT:
        Printf_EventRecord(PRINTF_EVENT_FAULT_OVERCURRENT);
        break;
    case MOTOR_STATUS_ENCODER_ERROR:
        Printf_EventRecord(PRINTF_EVENT_FAULT_ENCODER);
        break;
    case MOTOR_STATUS_SENSOR_ERROR:
        Printf_EventRecord(PRINTF_EVENT_FAULT_SENSOR);
        break;
    case MOTOR_STATUS_PWM_CALC_FAULT:
        Printf_EventRecord(PRINTF_EVENT_FAULT_PWM);
        break;
    case MOTOR_STATUS_EMERGENCY_STOP:
        Printf_EventRecord(PRINTF_EVENT_FAULT_EMERGENCY_STOP);
        break;
    case MOTOR_STATUS_DISABLED:
        Printf_EventRecord(PRINTF_EVENT_FAULT_DISABLED);
        break;
    default:
        break;
    }
}

static void Printf_RecordModeEvent(uint8_t mode)
{
    switch (mode) {
    case Velocity_OPEN_MODE:
        Printf_EventRecord(PRINTF_EVENT_MODE_OPEN);
        break;
    case Current_SINGLE_MODE:
        Printf_EventRecord(PRINTF_EVENT_MODE_CURRENT);
        break;
    case VelCur_DOUBLE_MODE:
        Printf_EventRecord(PRINTF_EVENT_MODE_SPEED);
        break;
    case PosVelCur_THREE_MODE:
        Printf_EventRecord(PRINTF_EVENT_MODE_POSITION);
        break;
    default:
        break;
    }
}

void Printf_RuntimeObserve(uint8_t status, uint8_t mode)
{
    static uint8_t observation_valid = 0U;
    static uint8_t last_status = 0U;
    static uint8_t last_mode = 0U;

    if (observation_valid == 0U) {
        observation_valid = 1U;
        last_status = status;
        last_mode = mode;
        Printf_RecordStatusEvent(status);
        Printf_RecordModeEvent(mode);
        return;
    }

    if (status != last_status) {
        last_status = status;
        Printf_RecordStatusEvent(status);
    }
    if (mode != last_mode) {
        last_mode = mode;
        Printf_RecordModeEvent(mode);
    }
}

static uint8_t Printf_ControlCommandAllowed(uint8_t command)
{
    if (command == 'C') {
        /* CO=0必须在任何状态均可停机；CO=1仍只请求既有初始化流程。 */
        return 1U;
    }
    if ((Sguan.status < MOTOR_STATUS_IDLE) ||
        (Sguan.status > MOTOR_STATUS_POSITION_HOLD)) {
        return 0U;
    }
    if ((command == 'V') && (Sguan.mode != Velocity_OPEN_MODE)) {
        return 0U;
    }
    return 1U;
}

// [接收]完整命令解析；所有合法性检查完成后才一次性提交控制量。
uint8_t Printf_Adjust(const uint8_t *data, uint16_t length){
    float data_get = 0.0f;
    uint8_t command_valid;

    if ((data == NULL) || (length < 2U)) {
        Printf_EventRecord(PRINTF_EVENT_CMD_REJECTED);
        return 0U;
    }

    if (Printf_CommandEqual(data, length, "STREAM=0?") != 0U) {
        /* 停止命令在任何状态幂等；只改变通信状态，不触碰电机变量。 */
        if ((s_stream_state == PRINTF_STREAM_JUSTFLOAT) ||
            (s_stream_state == PRINTF_STREAM_STARTING)) {
            s_stream_state = PRINTF_STREAM_STOPPING;
            Printf_EventRecord(PRINTF_EVENT_STREAM_STOPPING);
        } else if (s_stream_state == PRINTF_STREAM_TEXT) {
            Printf_EventRecord(PRINTF_EVENT_STREAM_OFF);
        }
        return 1U;
    }

    if (Printf_CommandEqual(data, length, "STREAM=1?") != 0U) {
        if ((s_stream_state == PRINTF_STREAM_TEXT) &&
            (Sguan.status == MOTOR_STATUS_IDLE)) {
            /* 仅状态4允许进入STARTING，且不预置任何运动目标。 */
            s_stream_state = PRINTF_STREAM_STARTING;
            Printf_EventRecord(PRINTF_EVENT_STREAM_STARTING);
            return 1U;
        }
        Printf_EventRecord(PRINTF_EVENT_CMD_REJECTED);
        return 0U;
    }

    if ((Printf_CommandEqual(data, length, "HELP?") != 0U) ||
        (Printf_CommandEqual(data, length, "INFO?") != 0U) ||
        (Printf_CommandEqual(data, length, "CAL?") != 0U) ||
        (Printf_CommandEqual(data, length, "LOG?") != 0U) ||
        (Printf_CommandEqual(data, length, "STREAM?") != 0U)) {
        PRINTF_QUERY_TYPE query = PRINTF_QUERY_HELP;

        if (s_stream_state != PRINTF_STREAM_TEXT) {
            /* JustFloat期间不得插入查询文本，拒绝动作保存在RAM供LOG?回看。 */
            Printf_EventRecord(PRINTF_EVENT_QUERY_REJECTED_STREAM);
            return 0U;
        }
        if (data[0] == 'I') {
            query = PRINTF_QUERY_INFO;
        } else if (data[0] == 'C') {
            query = PRINTF_QUERY_CAL;
        } else if (data[0] == 'L') {
            query = PRINTF_QUERY_LOG;
        } else if (data[0] == 'S') {
            query = PRINTF_QUERY_STREAM;
        }
        if (Printf_QuerySchedule(query) == 0U) {
            Printf_EventRecord(PRINTF_EVENT_CMD_REJECTED);
            return 0U;
        }
        return 1U;
    }

    command_valid = (uint8_t)(
        (data[1] == 'O') &&
        ((data[0] == 'A') || (data[0] == 'B') ||
         (data[0] == 'C') || (data[0] == 'V')));

    /* 非法载荷（包括字面量VO=%f?）直接丢弃，不修改任何控制量。 */
    if ((command_valid == 0U) ||
        (Get_Data(data, length, &data_get) == 0U) ||
        (Printf_ControlCommandAllowed(data[0]) == 0U)) {
        Printf_EventRecord(PRINTF_EVENT_CMD_REJECTED);
        return 0U;
    }

    if (data[0] == 'A') {
        User_AO_Adjust(data_get);
        Printf_EventRecord(PRINTF_EVENT_CMD_AO_ACCEPTED);
    } else if (data[0] == 'B') {
        User_BO_Adjust(data_get);
        Printf_EventRecord(PRINTF_EVENT_CMD_BO_ACCEPTED);
    } else if (data[0] == 'C') {
        User_CO_Adjust(data_get);
        Printf_EventRecord(PRINTF_EVENT_CMD_CO_ACCEPTED);
    } else {
        User_VO_Adjust(data_get);
        Printf_EventRecord(PRINTF_EVENT_CMD_VO_ACCEPTED);
    }
    return 1U;
}

void Printf_RxFeed(const uint8_t *data, uint16_t length)
{
    uint16_t index;

    if ((data == NULL) || (length == 0U)) {
        return;
    }

    for (index = 0U; index < length; index++) {
        uint8_t ch = data[index];

        if (s_rx_discard_until_end != 0U) {
            if (ch == '?') {
                s_rx_discard_until_end = 0U;
                s_rx_command_length = 0U;
                Printf_EventRecord(PRINTF_EVENT_CMD_REJECTED);
            }
            continue;
        }

        /* VOFA文本控件可能附加换行；仅在命令为空时忽略行结束符。 */
        if (((ch == '\r') || (ch == '\n')) && (s_rx_command_length == 0U)) {
            continue;
        }

        if (ch == '?') {
            if (s_rx_command_length < USER_DEBUG_RX_COMMAND_MAX) {
                s_rx_command[s_rx_command_length++] = ch;
                (void)Printf_Adjust(s_rx_command, s_rx_command_length);
            } else {
                Printf_EventRecord(PRINTF_EVENT_CMD_REJECTED);
            }
            s_rx_command_length = 0U;
            continue;
        }

        if (s_rx_command_length < (USER_DEBUG_RX_COMMAND_MAX - 1U)) {
            s_rx_command[s_rx_command_length++] = ch;
        } else {
            /* 超长命令一直丢弃到下一个问号，禁止尾部被误识别成新命令。 */
            s_rx_command_length = 0U;
            s_rx_discard_until_end = 1U;
        }
    }
}
