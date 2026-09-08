/**
  ******************************************************************************
  * @file    app_joint.c
  * @brief   GIM6010 关节应用层实现（应用状态机 + LED 主题 + 遥测调度）
  * @note    LED 主题（DOC §10.3）：
  *          INIT=白色呼吸 / CALIBRATING=黄色闪 / IDLE=绿色呼吸 /
  *          RUNNING=青色呼吸 / FAULT=红色快闪 / DISABLED(0x17)=蓝色慢闪
  ******************************************************************************
  */
#include "app_joint.h"
#include "SguanFOC.h"
#include "Sguan_MotorStatus.h"
#include "Sguan_printf.h"
#include "app_callbacks.h"       /* User_UserTX 引用 g_high_loop_cnt（Ch6） */
#include "UserData_Function.h"   /* User_UserControl 内联路径使用 PWM 使能/失能接口 */
#include "UserData_UserControl.h"
#include "BSP_WS2812B.h"
#include "BSP_ADC.h"
#include "BSP_MT6816.h"
#include "canfd.h"
#include "main.h"

/* ============================ 灯效配置（本地预设） ======================== */
static const BSP_RGB_BlinkCfg_t s_blink_yellow_fast =
    { .count = 3U, .onMs = 80U, .intervalMs = 160U, .groupPauseMs = 500U };
static const BSP_RGB_BlinkCfg_t s_blink_red_fast =
    { .count = 3U, .onMs = 60U, .intervalMs = 120U, .groupPauseMs = 300U };
static const BSP_RGB_BlinkCfg_t s_blink_green_slow =
    { .count = 1U, .onMs = 300U, .intervalMs = 1000U, .groupPauseMs = 0U };
static const BSP_RGB_BreathCfg_t s_breath_white =
    { .colorId = BSP_RGB_ID_WHITE, .minLevel = 0U, .maxLevel = 80U, .periodMs = 1500U };
static const BSP_RGB_BreathCfg_t s_breath_green =
    { .colorId = BSP_RGB_ID_GREEN, .minLevel = 0U, .maxLevel = 120U, .periodMs = 1200U };
static const BSP_RGB_BreathCfg_t s_breath_cyan =
    { .colorId = BSP_RGB_ID_CYAN, .minLevel = 20U, .maxLevel = 150U, .periodMs = 800U };
static const BSP_RGB_BreathCfg_t s_breath_blue_slow =
    { .colorId = BSP_RGB_ID_BLUE, .minLevel = 0U, .maxLevel = 60U, .periodMs = 2000U };

/* ============================ 内部状态 ==================================== */
static APP_JointState_t s_state = APP_JOINT_INIT;
static uint32_t s_init_tick = 0U;
static uint8_t s_foc_started = 0U;    /* 已触发过 FOC 初始化 */

/* ============================ 状态映射 ==================================== */
/* FOC 内核状态 → 应用层状态（编号见 Sguan_MotorStatus.h）
 * 注意：s_state 每圈先被此映射覆盖，再由 APP_StateMachine 做边沿转移；
 * status=0(STANDBY) 且从未使能时保留 INIT→STANDBY 的自有时序。 */
static APP_JointState_t APP_MapFocStatus(uint8_t foc_status)
{
    if (foc_status == MOTOR_STATUS_STANDBY) {                 /* 0：未使能 */
        return (s_foc_started != 0U) ? APP_JOINT_STANDBY
               : ((s_state == APP_JOINT_INIT) ? APP_JOINT_INIT : APP_JOINT_STANDBY);
    }
    if (foc_status <= MOTOR_STATUS_CALIBRATING) {             /* 1~3 */
        return APP_JOINT_CALIBRATING;
    }
    if (foc_status == MOTOR_STATUS_IDLE) {                    /* 4 */
        return APP_JOINT_IDLE;
    }
    if (foc_status == MOTOR_STATUS_DISABLED) {                /* 0x17 软失能 */
        return APP_JOINT_IDLE;
    }
    if (foc_status >= MOTOR_STATUS_OVERVOLTAGE) {             /* 0x0E~0x18 故障 */
        return APP_JOINT_FAULT;
    }
    return APP_JOINT_RUNNING;                                 /* 5~13 运行态 */
}

/* ============================ LED 主题 ==================================== */
static void APP_LedUpdate(uint32_t tick)
{
    switch (s_state) {
    case APP_JOINT_INIT:
        BSP_RGB_Breath(tick, &s_breath_white);
        break;
    case APP_JOINT_STANDBY:
        BSP_RGB_Blink(tick, &s_blink_green_slow, BSP_RGB_ID_GREEN, 255U);
        break;
    case APP_JOINT_CALIBRATING:
        BSP_RGB_Blink(tick, &s_blink_yellow_fast, BSP_RGB_ID_YELLOW, 255U);
        break;
    case APP_JOINT_IDLE:
        BSP_RGB_Breath(tick, &s_breath_green);
        break;
    case APP_JOINT_RUNNING:
        BSP_RGB_Breath(tick, &s_breath_cyan);
        break;
    case APP_JOINT_FAULT:
        BSP_RGB_Blink(tick, &s_blink_red_fast, BSP_RGB_ID_RED, 255U);
        break;
    default:
        BSP_RGB_Breath(tick, &s_breath_blue_slow);
        break;
    }
}

/* ============================ 初始化 ====================================== */
void APP_JointInit(void)
{
    s_state = APP_JOINT_INIT;
    s_init_tick = HAL_GetTick();
    s_foc_started = 0U;

    /* 默认开环；CO=1? 仅执行初始化/对齐，非零 AO 才输出 Uq 并启动。 */
    Sguan.mode = APP_DEFAULT_MODE;
    Printf_Init(&Sguan.TXdata);      /* JustFloat 帧尾初始化（库内要等首次 Start_Tick 才做，
                                       待命态 APP 补发数据需要它先就绪） */
    BSP_MT6816_Init();               /* 编码器提前初始化：待命态即可手转观察角度（FOC 使能时
                                       User_InitialInit 会再 Init 一次重新预读，无害） */
    BSP_RGB_Init();
    BSP_ADC_Init();
    CANFD_Init();
}

/**
 * @brief  上电待命流程：自检 500ms → STANDBY 待命（不自动使能闭环！）
 * @note   ⚠️ 首次闭环方向符号未经台架验证，上电自动进闭环曾导致电流环正反馈、
 *         电源 OCP（2026-08-30 台架记录）。现改为手动使能：串口 "CO=1?" 或
 *         CAN 使能命令触发初始化+对齐；故障后同样手动复位。
 *         默认模式为开环；初始化回到 IDLE 后必须显式发送 VO 速度和 AO/Uq。
 *         若需逐环校准，可先发 "BO=1?" 切电流模式。
 */
static void APP_StateMachine(void)
{
    uint8_t foc = Sguan.status;
    uint32_t now = HAL_GetTick();

    switch (s_state) {
    case APP_JOINT_INIT:
        if ((now - s_init_tick) >= 500U) {
            s_state = APP_JOINT_STANDBY;   /* 仅待命，不触发 FOC 初始化 */
        }
        break;

    case APP_JOINT_STANDBY:
        /* 等待外部使能：串口 CO=1?（User_CO_Adjust 置 UNINITIALIZED）或 CAN EN 命令 */
        if (foc > MOTOR_STATUS_STANDBY) {
            s_foc_started = 1U;               /* 模式保持用户 BO 预选（默认开环） */
            s_state = APP_JOINT_CALIBRATING;
        }
        break;

    case APP_JOINT_CALIBRATING:
        if (foc == MOTOR_STATUS_IDLE) {
            s_state = APP_JOINT_IDLE;
        } else if (foc >= MOTOR_STATUS_OVERVOLTAGE) {
            s_state = APP_JOINT_FAULT;
        } else if (foc == MOTOR_STATUS_STANDBY) {
            /* 冻结输出安全网（同 RUNNING 态注释）：回待机即急停清零 PWM */
            Sguan.status = MOTOR_STATUS_EMERGENCY_STOP;
            s_foc_started = 0U;
            s_state = APP_JOINT_FAULT;
        } else {
            /* 0x01~0x03 继续等待（对齐阻塞在 SguanFOC_main_Loop 内） */
        }
        break;

    case APP_JOINT_IDLE:
        if (foc >= MOTOR_STATUS_OVERVOLTAGE) {
            s_state = APP_JOINT_FAULT;
        } else if (foc > MOTOR_STATUS_IDLE) {
            s_state = APP_JOINT_RUNNING;   /* 收到目标进入运行态 */
        } else {
            /* 保持待命 */
        }
        break;

    case APP_JOINT_RUNNING:
        if (foc >= MOTOR_STATUS_OVERVOLTAGE) {
            s_state = APP_JOINT_FAULT;
        } else if ((foc == MOTOR_STATUS_IDLE) || (foc == MOTOR_STATUS_DISABLED)) {
            s_state = APP_JOINT_IDLE;
        } else if (foc == MOTOR_STATUS_STANDBY) {
            /* ⚠️ 冻结输出安全网：库回到 STANDBY 时不清零 PWM（2026-08-30 台架曾以
             * 冻结的非零矢量持续输出 4.2A 加热电机到 57℃）。检测到"曾在运行却回到
             * 待机"立即置急停（库 ≥19 分支会将电压/占空比清零），防烧电机。 */
            Sguan.status = MOTOR_STATUS_EMERGENCY_STOP;
            s_foc_started = 0U;
            s_state = APP_JOINT_FAULT;
        } else {
            /* 位置保持/跟踪中 */
        }
        break;

    case APP_JOINT_FAULT:
        /* 等待 CO=1 或 CAN FAULT_CLEAR 将 Sguan.status 复位为 UNINITIALIZED */
        if ((foc > MOTOR_STATUS_STANDBY) && (foc <= MOTOR_STATUS_CALIBRATING)) {
            s_state = APP_JOINT_CALIBRATING;
        }
        break;

    default:
        s_state = APP_JOINT_INIT;
        break;
    }
}

/* ============================ 主循环 ====================================== */
void APP_JointPoll(void)
{
    uint32_t tick = HAL_GetTick();

    /* UART 中断只入队；所有会改控制器/模式的命令在非中断上下文执行。 */
    APP_UartCommandPoll();

    /* 0.编码器采样：主循环 1kHz 阻塞直读（台架期稳健方案，替代 DMA 链——DMA 在
     *    FOC 使能后全部失败导致电角度冻结→电流环失控→OCP，详见 BSP 注释） */
    BSP_MT6816_PollBlocking();

    /* 0.5 待命态遥测补发（库 main_Loop 只在 status>3 或 ≥19 时发送 VOFA；
     *    编码器采样已由上面的 1kHz 阻塞直读覆盖，手转电机 Ch14 实时可观察） */
    if (Sguan.status <= MOTOR_STATUS_CALIBRATING) {
        User_UserTX();
        Printf_Loop(&Sguan.TXdata);
    }

    /* 1.应用状态机推进（同步 FOC 内核状态映射） */
    s_state = APP_MapFocStatus(Sguan.status);
    APP_StateMachine();

    /* 2.LED 主题 */
    APP_LedUpdate(tick);
    BSP_RGB_Poll();

    /* 3.慢速模拟量（VBUS 规则组后备 + NTC 100Hz） */
    BSP_ADC_Poll();

    /* 4.FDCAN 遥测与命令 */
    CANFD_Poll();
}

APP_JointState_t APP_JointGetState(void)
{
    return s_state;
}

uint8_t APP_JointGetFocStatus(void)
{
    return Sguan.status;
}
