/**
  ******************************************************************************
  * @file    canfd.h
  * @brief   GIM6010 多轴 FDCAN 协议（基础子集，DOC/GIM6010.md §12.2）
  * @note    总线：1 Mbps 经典帧（位时序见 GIM6010_V0.0.ioc/Core/Src/fdcan.c）
  *          帧 ID：命令 0x100+ID / 遥测 0x200+ID / 同步 0x7FF(预留)
  ******************************************************************************
  */

#ifndef __CANFD_H
#define __CANFD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ============================ 节点与帧 ID ============================ */
#ifndef CANFD_NODE_ID
#define CANFD_NODE_ID           (1U)    /* 本板节点号（机械臂 1~6 轴，后续改 Flash/拨码） */
#endif

#define CANFD_ID_CMD            (0x100U + CANFD_NODE_ID)  /* 主→从 命令帧 */
#define CANFD_ID_TELEM          (0x200U + CANFD_NODE_ID)  /* 从→主 遥测帧 */
#define CANFD_ID_SYNC           (0x7FFU)                  /* 主→从 广播同步(预留) */

#define CANFD_TELEM_PERIOD_MS   (2U)    /* 遥测周期 500Hz（1kHz 时隙方案后续迭代） */

/* ============================ 命令帧 (8B) ============================ */
/*  [0..3] int32  模式目标：mode0=Uq[mV]，mode1=Iq[mA]，
 *                       mode2=保留(写0)，mode3=输出轴位置[µrad]
 *  [4..5] int16  输出轴速度 rpm：mode0/2 生效
 *  [6..7] uint16 命令位：
 *      bit0     使能（触发初始化/对齐）
 *      bit1     故障复位（STANDBY/锁定 → UNINITIALIZED）
 *      bit2     失能（软停）
 *      bit3..4  模式：0开环 1电流 2速度 3位置
 *      bit5     保留，必须写 0（换相零位仅允许由上电对齐建立）      */
#define CANFD_CMD_EN            (0x0001U)
#define CANFD_CMD_FAULT_CLEAR   (0x0002U)
#define CANFD_CMD_DIS           (0x0004U)
#define CANFD_CMD_MODE_MASK     (0x0018U)
#define CANFD_CMD_MODE_SHIFT    (3U)
#define CANFD_CMD_RESERVED_5    (0x0020U)
#define CANFD_CMD_RESERVED_MASK (0xFFE0U)

/* ============================ 遥测帧 (8B) ============================ */
/*  [0..3] int32  输出轴位置 µrad（超量程饱和，非有限值编码为0）
 *  [4..5] int16  输出轴转速 rpm（超量程饱和，非有限值编码为0）
 *  [6..7] uint16 bit0..7=Sguan.status，bit8..11=Sguan.mode，bit12..15=保留 */
#define CANFD_URAD_PER_RAD      (1000000.0f)
#define CANFD_MILLI_PER_UNIT    (1000.0f)

/* ============================ 接口 ==================================== */
void CANFD_Init(void);          /* 滤波器 + 启动 + 开 RX 中断 */
void CANFD_OnRxIsr(void);       /* RX FIFO0 中断入口：校验/入队；DIS 立即关 MOE */
uint8_t CANFD_SafetyPoll(void); /* 主循环消费锁存 DIS，清状态与旧命令 */
void CANFD_BlockingSafetyPoll(void); /* 校准阻塞等待内仅处理 DIS，丢弃其他命令 */
uint8_t CANFD_IsStopPending(void);   /* 供最终 MOE 使能边沿复查 */
void CANFD_Poll(void);          /* 主循环：顺序解析命令 + 遥测发送节拍 */

uint8_t CANFD_IsOnline(void);   /* 1=上电后收到过命令帧（联调指示） */
uint32_t CANFD_GetRxDropCount(void);

#ifdef __cplusplus
}
#endif

#endif /* __CANFD_H */
