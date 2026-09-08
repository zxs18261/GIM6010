#ifndef __APP_API_H
#define __APP_API_H
/* APP 层对外访问接口(供 UserData_*.h 调用, 避免库直接依赖 APP 实现) */
#include <stdint.h>

uint32_t APP_GetHeartbeat(void);    /* 主循环自增心跳计数 */
int      APP_GetCalibFlag(void);    /* 标定标志: 0未标定 1标定中 2成功 3失败 */
int      APP_GetCanNodeId(void);    /* CAN节点ID, 0xFF=未分配 */
uint32_t APP_GetCanHbCount(void);   /* CAN 节点心跳计数 */

#endif
