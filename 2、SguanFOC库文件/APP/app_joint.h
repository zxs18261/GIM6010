/**
  ******************************************************************************
  * @file    app_joint.h
  * @brief   GIM6010 关节应用层：应用状态机 + 命令仲裁 + LED 主题（DOC/GIM6010.md §10.3）
  * @note    应用层不重复实现保护（FOC 内核状态机已含过压/欠压/过温/过流/编码器错），
  *          只做：上电自检→手动使能等待→LED 指示→FDCAN/串口命令仲裁。
  ******************************************************************************
  */

#ifndef __APP_JOINT_H
#define __APP_JOINT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ============================ 应用层状态机 ============================ */
typedef enum
{
    APP_JOINT_INIT = 0,     /* 上电自检（外设/CAN 上线） */
    APP_JOINT_STANDBY,      /* 待命（不使能闭环；等 CO=1? 或 CAN 使能命令） */
    APP_JOINT_CALIBRATING,  /* FOC 初始化+对齐中（状态 0x01~0x03） */
    APP_JOINT_IDLE,         /* FOC 待命（状态 0x04，等待目标） */
    APP_JOINT_RUNNING,      /* 位置/速度/电流闭环运行（状态 0x05~0x13） */
    APP_JOINT_FAULT,        /* 故障锁定（状态 0x0E~0x18，等待清除） */
    APP_JOINT_STATE_COUNT
} APP_JointState_t;

/* 上电默认控制模式：开环；仍须 CO=1 完成初始化且收到非零 AO 才会打开功率桥。 */
#define APP_DEFAULT_MODE        (0U)    /* 0开环 1电流 2速度 3位置(PosVelCur_THREE_MODE) */

/* ============================ 接口 ==================================== */
void APP_JointInit(void);        /* main [USER 2] 调用：初始化应用层并保持高阻待命 */
void APP_JointPoll(void);        /* main [WHILE] 调用：状态机推进 + LED + CAN 遥测 */

APP_JointState_t APP_JointGetState(void);
uint8_t APP_JointGetFocStatus(void);    /* Sguan.status 快照 */

#ifdef __cplusplus
}
#endif

#endif /* __APP_JOINT_H */
