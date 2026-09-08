#ifndef __SGUAN_PRINTF_H
#define __SGUAN_PRINTF_H

/* SguanFOC配置文件声明 */
#include "Sguan_Config.h"

/**
 * @description: 当前开环调试只发送14个动态通道，静态参数改由INFO?/CAL?查询
 * @return {*}
 */
#define CH_COUNT 14U // UART JustFloat：14个float加4字节帧尾，共60字节

typedef struct{
    float fdata[CH_COUNT];
    uint8_t tail[4];
}PRINTF_STRUCT;

/* USART1上行协议四态；数值固定，便于STREAM?和调试日志长期解析。 */
typedef enum {
    PRINTF_STREAM_TEXT = 0,
    PRINTF_STREAM_STARTING = 1,
    PRINTF_STREAM_JUSTFLOAT = 2,
    PRINTF_STREAM_STOPPING = 3
} PRINTF_STREAM_STATE;

/* 事件队列只保存固定事件码和时间戳，禁止在中断中格式化中文。 */
typedef enum {
    PRINTF_EVENT_BOOT = 1,
    PRINTF_EVENT_INIT_START,
    PRINTF_EVENT_CAL_WAIT_VBUS,
    PRINTF_EVENT_CAL_FLASH_LOAD,
    PRINTF_EVENT_CAL_START,
    PRINTF_EVENT_CAL_PHASE_CHECK,
    PRINTF_EVENT_CAL_SAVE_OK,
    PRINTF_EVENT_CAL_ENCODER_ERROR,
    PRINTF_EVENT_CAL_PHASE_ERROR,
    PRINTF_EVENT_CAL_FLASH_ERROR,
    PRINTF_EVENT_IDLE,
    PRINTF_EVENT_STANDBY,
    PRINTF_EVENT_MODE_OPEN,
    PRINTF_EVENT_MODE_CURRENT,
    PRINTF_EVENT_MODE_SPEED,
    PRINTF_EVENT_MODE_POSITION,
    PRINTF_EVENT_STREAM_STARTING,
    PRINTF_EVENT_STREAM_ON,
    PRINTF_EVENT_STREAM_STOPPING,
    PRINTF_EVENT_STREAM_OFF,
    PRINTF_EVENT_CMD_AO_ACCEPTED,
    PRINTF_EVENT_CMD_BO_ACCEPTED,
    PRINTF_EVENT_CMD_CO_ACCEPTED,
    PRINTF_EVENT_CMD_VO_ACCEPTED,
    PRINTF_EVENT_CMD_REJECTED,
    PRINTF_EVENT_QUERY_REJECTED_STREAM,
    PRINTF_EVENT_FAULT_OVERVOLTAGE,
    PRINTF_EVENT_FAULT_UNDERVOLTAGE,
    PRINTF_EVENT_FAULT_OVERTEMPERATURE,
    PRINTF_EVENT_FAULT_UNDERTEMPERATURE,
    PRINTF_EVENT_FAULT_OVERCURRENT,
    PRINTF_EVENT_FAULT_ENCODER,
    PRINTF_EVENT_FAULT_SENSOR,
    PRINTF_EVENT_FAULT_PWM,
    PRINTF_EVENT_FAULT_EMERGENCY_STOP,
    PRINTF_EVENT_FAULT_DISABLED
} PRINTF_EVENT_CODE;

extern uint8_t Sguan_PrintfBuff[200];

// 函数定义声明
void Printf_Init(PRINTF_STRUCT *str);
void Printf_Loop(const PRINTF_STRUCT *str);
uint8_t Printf_TxSlotDue(void);
void Printf_TxCpltISR(void);
void Printf_TxErrorISR(void);
uint32_t Printf_GetTxDropCount(void);
uint32_t Printf_GetEventOverflowCount(void);
PRINTF_STREAM_STATE Printf_GetStreamState(void);
void Printf_MainLoop(const PRINTF_STRUCT *str);
void Printf_BackgroundService(void);
void Printf_RuntimeObserve(uint8_t status, uint8_t mode);
void Printf_EventRecord(PRINTF_EVENT_CODE code);
void Printf_RxFeed(const uint8_t *data, uint16_t length);
uint8_t Printf_Adjust(const uint8_t *data, uint16_t length);


#endif // SGUAN_PRINTF_H
