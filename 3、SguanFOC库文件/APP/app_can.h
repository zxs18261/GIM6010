#ifndef __APP_CAN_H
#define __APP_CAN_H
/* GIM6010 多节点 CAN 协议 + ID 分配状态机 (FDCAN1, 经典1M, 轮询)
 * 帧编码见 02_设计文档\01_ */
#include <stdint.h>

#define CAN_MAX_NODES       32u
#define CAN_NODE_UNASSIGNED 0xFFu
#define CAN_COORD_ID        0u

/* 超时/重试参数(宏, 可配) */
#define CAN_GRANT_TIMEOUT_MS  500u
#define CAN_GRANT_RETRY       5u
#define CAN_COORD_HB_MS       100u
#define CAN_NODE_HB_MS        500u
#define CAN_OFFLINE_CYCLES    3u

typedef enum { CAN_ST_UNASSIGNED = 0, CAN_ST_WAITING_GRANT, CAN_ST_ASSIGNED } can_state_t;

/* 业务目标类型(CMD 帧 B1) */
#define CMD_TYPE_IQ      0u
#define CMD_TYPE_SPEED   1u
#define CMD_TYPE_POS     2u
#define CMD_TYPE_ENABLE  3u
#define CMD_TYPE_CALIB   4u
#define CMD_TYPE_FACTORY 5u
#define CMD_TYPE_PID     6u

void     APP_CAN_Init(void);
void     APP_CAN_Task(void);          /* 主循环轮询 */
void     APP_CAN_RequestId(void);     /* 按键触发申请 ID */
void     APP_CAN_ClearId(void);       /* 长按5s清 ID */
int      APP_CAN_GetState(void);
uint8_t  APP_CAN_GetNodeId(void);
uint16_t APP_CAN_GetSessionToken(void);
uint32_t APP_CAN_GetHbCount(void);
int      APP_CAN_GetPendingTarget(uint8_t *type, float *value);  /* 1=有 */
void     APP_CAN_SetTelemetry(float actual, uint8_t mode);

#endif
