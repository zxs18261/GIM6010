#ifndef __USERDATA_USERCONTROL_H
#define __USERDATA_USERCONTROL_H
#include "SguanFOC.h"
/* 电机控制User用户设置·实时参数控制页面
 * GIM6010 关节控制器：VOFA AO/BO/CO/VO 命令语义 + JustFloat 24 通道
 * 单位约定：内部与位置/速度命令使用 rad/rad·s⁻¹；VOFA 角度为 rad、输出轴速度为 rpm
 */

/* 用户自己的CODE BEGIN Includes */
#include "BSP_MT6816.h"
#include "BSP_ADC.h"
#include "Sguan_IQmath.h"
#include "app_callbacks.h"
/* 用户自己的CODE END Includes */

#ifndef USER_OPENLOOP_UQ_MAX_RATIO
#define USER_OPENLOOP_UQ_MAX_RATIO  (0.35f)   /* 开环 |Uq| 上限比例（×VBUS） */
#endif
#ifndef USER_OPENLOOP_DT
#define USER_OPENLOOP_DT            (TIM_T)   /* High_Loop 已与 ADC 注入完成的 20 kHz 同步 */
#endif
#ifndef USER_COMMAND_ENABLE_EPS
#define USER_COMMAND_ENABLE_EPS      (0.001f) /* 模式0～2近零 AO 不开桥；位置模式不适用 */
#endif
#ifndef USER_POSITION_ABS_MAX_RAD
#define USER_POSITION_ABS_MAX_RAD    (512.0f) /* 电机轴连续位置命令绝对上限 */
#endif

/* 最近命令内部记录；当前 24 通道诊断帧不直接输出这些缓存。 */
static float s_vofa_last_AO = 0.0f;
static float s_vofa_last_BO = 0.0f;
static float s_vofa_last_CO = 0.0f;
static float s_vofa_last_VO = 0.0f;

/* ---------------- 开环（模式0）电角度积分器 ----------------
 * 实体只在 SguanFOC.c 定义；本头会被多个翻译单元包含，不能定义 static 状态。 */
extern float g_user_openloop_eangle;
extern volatile uint8_t g_user_openloop_angle_initialized;

static inline void User_ClearMotionCommand(void);
static inline void User_VO_Adjust(float VO);
static inline void User_ResetControllerState(void);
static inline uint8_t User_FaultClearAllowed(void);

/* 输出轴速度命令限幅 [rad/s]（DOC §6 软件上限=额定×1.2） */
static inline float User_LimitShaftSpeed(float shaft_speed){
    if (shaft_speed > GIM6010_SHAFT_MAX_RAD_S) {
        return GIM6010_SHAFT_MAX_RAD_S;
    }
    if (shaft_speed < -GIM6010_SHAFT_MAX_RAD_S) {
        return -GIM6010_SHAFT_MAX_RAD_S;
    }
    return shaft_speed;
}

/* IDLE 是校准后的安全待机：显式有效 AO 才能离开；位置 AO=0 也是有效目标。 */
static inline void User_StartMotionFromIdle(float command){
    float motion_error = command;
    uint16_t neutral_duty = (uint16_t)((USER_TIM1_PWM_ARR + 1U) / 2U);
    uint32_t primask;
    if (Sguan.mode == PosVelCur_THREE_MODE) {
        motion_error = command - Sguan.encoder.Real_Pos;
    }
    if (Sguan.status != MOTOR_STATUS_IDLE) {
        return;
    }
    /* 位置 0 rad 也是有效目标；其余三种模式用近零命令保持高阻。 */
    if ((Sguan.mode != PosVelCur_THREE_MODE) &&
        (motion_error <= USER_COMMAND_ENABLE_EPS && motion_error >= -USER_COMMAND_ENABLE_EPS)) {
        return;
    }

    /* TIM2 保护 ISR 也会改 status/关 MOE；从最终 IDLE 复核、写中点、发布
     * 运行状态到 MOE 使能必须构成一个短临界边沿，不能覆盖刚锁存的故障。 */
    primask = __get_PRIMASK();
    __disable_irq();
    if (Sguan.status != MOTOR_STATUS_IDLE) {
        if (primask == 0U) __enable_irq();
        return;
    }

    /* 先覆盖上个模式遗留的 CCR，再发布运行状态，最后才打开 MOE。 */
    Sguan.foc.Duty_u = neutral_duty;
    Sguan.foc.Duty_v = neutral_duty;
    Sguan.foc.Duty_w = neutral_duty;
    User_PwmDuty_Set(neutral_duty, neutral_duty, neutral_duty);
    switch (Sguan.mode) {
    case Current_SINGLE_MODE:
    case Velocity_OPEN_MODE:
        Sguan.status = MOTOR_STATUS_TORQUE_CONTROL;
        break;
    case VelCur_DOUBLE_MODE:
        Sguan.status = MOTOR_STATUS_ACCELERATING;
        break;
    case PosVelCur_THREE_MODE:
        Sguan.status = (motion_error >= 0.0f) ? MOTOR_STATUS_POSITION_INCREASING
                                              : MOTOR_STATUS_POSITION_DECREASING;
        break;
    default:
        User_PwmOutputDisable();
        if (primask == 0U) __enable_irq();
        return;
    }
    User_PwmOutputEnable();
    if (primask == 0U) __enable_irq();
}

/**
 * 开环 FOC：θe 由 Target_Speed 积分，Ud=0，Uq 由 AO 独立给定。
 * Target_Speed 保持库内语义：电机轴机械角速度 [rad/s]。
 * 注意：Calculate 先算编码器角；模式0 每个高频周期都用积分角覆盖，
 * 使 SVPWM 使用的角度不参与任何编码器速度闭环。
 */
static inline void User_UserControl(void){
    if (Sguan.mode != Velocity_OPEN_MODE) {
        g_user_openloop_angle_initialized = 0U; /* 再次进入开环时从当前转子角度平滑接管。 */
        return;
    }
    if (Sguan.status <= MOTOR_STATUS_CALIBRATING ||
        Sguan.status >= MOTOR_STATUS_OVERVOLTAGE) {
        return;
    }

    /* Target_Speed（电机轴机械角速度）→ 电角速度 */
    float we = Sguan.foc.Target_Speed * (float)Sguan.motor.Poles;
    float awe = (we >= 0.0f) ? we : -we;
    float we_hard_max = GIM6010_SHAFT_MAX_RAD_S * GIM6010_GEAR_RATIO *
                        (float)Sguan.motor.Poles;

    /* 防御性硬限：不得越过输出轴命令上限 */
    if (awe > we_hard_max) {
        we = (we >= 0.0f) ? we_hard_max : -we_hard_max;
    }

    /* 首次进入开环时从当前编码器电角度接管，避免电压矢量突跳。 */
    if (g_user_openloop_angle_initialized == 0U) {
        g_user_openloop_eangle = Sguan.encoder.Real_Erad;
        g_user_openloop_angle_initialized = 1U;
    }

    g_user_openloop_eangle += we * USER_OPENLOOP_DT;
    while (g_user_openloop_eangle >= Value_2PI) {
        g_user_openloop_eangle -= Value_2PI;
    }
    while (g_user_openloop_eangle < 0.0f) {
        g_user_openloop_eangle += Value_2PI;
    }

    /* 纯开环每周期都强制 Ud=0，并用积分角覆盖编码器角供 SVPWM 使用。 */
    fast_sin_cos(g_user_openloop_eangle, &Sguan.foc.sine, &Sguan.foc.cosine);
    Sguan.foc.Ud_in = 0.0f;
    Sguan.encoder.Real_Erad = g_user_openloop_eangle; /* VOFA Ch13 看开环电角度 */
}

/* AO：按模式写目标（位置模式=最近一次成功对齐零点下的电机轴连续绝对目标 rad，
 * 不是相对当前位置的增量） */
static inline void User_AO_Adjust(float AO){
    if (Value_isnan(AO) || Value_isinf(AO) || (Sguan.status < MOTOR_STATUS_IDLE) ||
        (Sguan.status >= MOTOR_STATUS_OVERVOLTAGE)) return;
    s_vofa_last_AO = AO;
    switch (Sguan.mode){
    case Velocity_OPEN_MODE:
        {
            float umax = USER_OPENLOOP_UQ_MAX_RATIO * Sguan.motor.VBUS;
            if (AO > umax) {
                AO = umax;
            } else if (AO < -umax) {
                AO = -umax;
            }
            Sguan.foc.Uq_in = AO;
            Sguan.foc.Ud_in = 0.0f;
        }
        break;
    case Current_SINGLE_MODE:
        /* 普通电流/转矩指令限制在额定连续电流，峰值能力不在串口命令中开放。 */
        if (AO > GIM6010_RATED_CURRENT_A) {
            AO = GIM6010_RATED_CURRENT_A;
        } else if (AO < -GIM6010_RATED_CURRENT_A) {
            AO = -GIM6010_RATED_CURRENT_A;
        }
        Sguan.foc.Target_Iq = AO;
        break;
    case VelCur_DOUBLE_MODE:
        /* AO 按输出轴机械 rad/s 接收，库内换算为电机轴速度。 */
        Sguan.foc.Target_Speed = User_LimitShaftSpeed(AO) * GIM6010_GEAR_RATIO;
        break;
    case PosVelCur_THREE_MODE:
        /* 位置环：目标为相对上电对齐零位的电机轴连续机械角 [rad]，可多圈。 */
        if (AO > USER_POSITION_ABS_MAX_RAD) {
            AO = USER_POSITION_ABS_MAX_RAD;
        } else if (AO < -USER_POSITION_ABS_MAX_RAD) {
            AO = -USER_POSITION_ABS_MAX_RAD;
        }
        Sguan.foc.Target_Pos = AO;
        break;
    default:
        break;
    }
    User_StartMotionFromIdle(AO);
}

static inline void User_BO_Adjust(float BO){
    s_vofa_last_BO = BO;
    uint8_t next_mode;
    uint32_t primask;
    if (Value_isnan(BO) || Value_isinf(BO) || (BO < 0.0f) || (BO > 3.0f) ||
        (BO != (float)(uint8_t)BO)) return;
    if ((Sguan.status >= MOTOR_STATUS_UNINITIALIZED) &&
        (Sguan.status <= MOTOR_STATUS_CALIBRATING)) return;
    next_mode = (uint8_t)BO;
    if (next_mode == Sguan.mode) return;

    /* 模式切换是安全边沿：先断桥并清除旧模式目标，随后等待新模式的显式 AO。 */
    User_PwmOutputDisable();
    primask = __get_PRIMASK();
    __disable_irq();
    if ((Sguan.status >= MOTOR_STATUS_IDLE) && (Sguan.status < MOTOR_STATUS_OVERVOLTAGE)) {
        Sguan.status = MOTOR_STATUS_IDLE;
    }
    if (primask == 0U) __enable_irq();
    User_ClearMotionCommand();
    User_ResetControllerState();
    g_user_openloop_angle_initialized = 0U;
    Sguan.mode = next_mode;
}

/* VO：仅在模式0设置开环输出轴速度 [rad/s]，AO 独立设置 Uq [V]。 */
static inline void User_VO_Adjust(float VO){
    s_vofa_last_VO = VO;
    if ((Sguan.mode == Velocity_OPEN_MODE) && !Value_isnan(VO) && !Value_isinf(VO) &&
        (Sguan.status >= MOTOR_STATUS_IDLE) &&
        (Sguan.status < MOTOR_STATUS_OVERVOLTAGE)) {
        Sguan.foc.Target_Speed = User_LimitShaftSpeed(VO) * GIM6010_GEAR_RATIO;
    }
}

static inline void User_ClearMotionCommand(void){
    /* 命令在使能边沿也必须清零：不能让上一次 AO/BO 的残留值直接进入新电流环。 */
    Sguan.foc.Ud_in = 0.0f;
    Sguan.foc.Uq_in = 0.0f;
    Sguan.foc.Target_Id = 0.0f;
    Sguan.foc.Target_Iq = 0.0f;
    Sguan.foc.Target_Speed = 0.0f;
    Sguan.foc.Target_Pos = 0.0f;
    #if CONFIG_Q31
    Sguan.foc.Ud_in_q31 = 0;
    Sguan.foc.Uq_in_q31 = 0;
    Sguan.foc.Target_Id_q31 = 0;
    Sguan.foc.Target_Iq_q31 = 0;
    Sguan.foc.Target_Speed_q31 = 0;
    Sguan.foc.Target_Pos_q31 = 0;
    #endif // CONFIG_Q31
    s_vofa_last_AO = 0.0f;
    g_user_openloop_angle_initialized = 0U;
}

static inline void User_ResetControllerState(void){
    /* 防止上一模式的 PI/STA 积分残留在下一模式重新使能时形成输出阶跃。 */
    #if !CONFIG_Q31
    PID_Init(&Sguan.control.Current_D);
    PID_Init(&Sguan.control.Current_Q);
    #if CONFIG_PI
    PID_Init(&Sguan.control.Velocity);
    #else
    STA_Init(&Sguan.control.Speed);
    #endif
    PID_Init(&Sguan.control.Position);
    #else // CONFIG_Q31
    PID_Init_q31(&Sguan.control.Current_D);
    PID_Init_q31(&Sguan.control.Current_Q);
    #if CONFIG_PI
    PID_Init_q31(&Sguan.control.Velocity);
    #else
    STA_Init_q31(&Sguan.control.Speed);
    #endif
    PID_Init_q31(&Sguan.control.Position);
    #endif // CONFIG_Q31
}

static inline uint8_t User_FaultClearAllowed(void){
    float value;

    /* 电压、温度和编码器类故障只有在信号恢复后才能清锁；过流现场在关桥后
     * 已冻结且偏置会重新校准，因此允许显式重启，再由启动预检二次把关。 */
    switch (Sguan.status) {
    case MOTOR_STATUS_OVERVOLTAGE:
    case MOTOR_STATUS_UNDERVOLTAGE:
        value = User_VBUS_DataGet();
        return (!Value_isnan(value) && !Value_isinf(value) &&
                (value >= Sguan.safe.VBUS_MIM) &&
                (value <= Sguan.safe.VBUS_MAX)) ? 1U : 0U;
    case MOTOR_STATUS_OVERTEMPERATURE:
    case MOTOR_STATUS_UNDERTEMPERATURE:
        value = User_Temperature_DataGet();
        return (!Value_isnan(value) && !Value_isinf(value) &&
                (value >= Sguan.safe.Temp_MIN) &&
                (value <= Sguan.safe.Temp_MAX)) ? 1U : 0U;
    case MOTOR_STATUS_ENCODER_ERROR:
        return BSP_MT6816_IsSampleFresh();
    case MOTOR_STATUS_SENSOR_ERROR:
        return (MOTOR_STATUS_SENSOR_ERROR_Signal() == 0U) ? 1U : 0U;
    case MOTOR_STATUS_EMERGENCY_STOP:
        return (MOTOR_STATUS_EMERGENCY_STOP_Signal() == 0U) ? 1U : 0U;
    case MOTOR_STATUS_OVERCURRENT:
    case MOTOR_STATUS_PWM_CALC_FAULT:
        return 1U;
    default:
        return (Sguan.status < MOTOR_STATUS_OVERVOLTAGE) ? 1U : 0U;
    }
}

static inline void User_CO_Adjust(float CO){
    uint8_t status_snapshot;
    uint8_t restart_required;
    uint32_t primask;

    /* CO 只接受精确二值；CO=0 可任意时刻停机，初始化/对齐中重复 CO=1 会被忽略。 */
    if (Value_isnan(CO) || Value_isinf(CO) ||
        ((CO != 0.0f) && (CO != 1.0f))) return;
    s_vofa_last_CO = CO;

    if (CO == 0.0f){
        /* 停机必须物理关闭功率桥；已有硬件故障码继续锁存，不能借 STOP
         * 退到 STANDBY 后再用普通 EN 绕过故障条件检查。 */
        User_PwmOutputDisable();
        primask = __get_PRIMASK();
        __disable_irq();
        if (Sguan.status < MOTOR_STATUS_OVERVOLTAGE) {
            Sguan.status = MOTOR_STATUS_STANDBY;
        }
        if (primask == 0U) __enable_irq();
        User_ClearMotionCommand();
        User_ResetControllerState();
        return;
    }

    status_snapshot = Sguan.status;
    if ((status_snapshot >= MOTOR_STATUS_UNINITIALIZED) &&
        (status_snapshot <= MOTOR_STATUS_CALIBRATING)) {
        return; /* 初始化/对齐已经在进行，重复 CO=1 不得中断校准。 */
    }
    if ((status_snapshot >= MOTOR_STATUS_OVERVOLTAGE) &&
        (User_FaultClearAllowed() == 0U)) {
        return;
    }

    /* 故障清除条件检查可能耗时；之后关桥并在短临界区确认状态未被 ISR 更新，
     * 再一次性清目标/控制器并降低状态严重度，绝不覆盖新锁存的故障。 */
    User_PwmOutputDisable();
    primask = __get_PRIMASK();
    __disable_irq();
    if (Sguan.status != status_snapshot) {
        if (primask == 0U) __enable_irq();
        return;
    }
    restart_required = ((status_snapshot == MOTOR_STATUS_STANDBY) ||
                        (status_snapshot >= MOTOR_STATUS_OVERVOLTAGE)) ? 1U : 0U;
    if ((status_snapshot >= MOTOR_STATUS_IDLE) &&
        (status_snapshot < MOTOR_STATUS_OVERVOLTAGE)) {
        Sguan.status = MOTOR_STATUS_IDLE;
    }
    if (restart_required != 0U) {
        Sguan.status = MOTOR_STATUS_UNINITIALIZED;
    }
    if (primask == 0U) __enable_irq();

    User_ClearMotionCommand();
    User_ResetControllerState();

    /* CO=1 会清零所有目标；开环速度 VO 与电压 AO 都必须在回到 IDLE 后
     * 由上位机重新显式发送，任何旧值或隐藏默认值都不得跨使能边沿恢复。 */

    /* 重新初始化：STANDBY/任意硬件故障 → UNINITIALIZED → 自动对齐+进 IDLE。
     * MOTOR_STATUS_OVERCURRENT 的编号为 18，不能再用“>=19”漏掉它。 */
    if (restart_required != 0U) {
        g_foc_fault_snapshot.valid = 0U; /* 新一轮 CO 启动前清除上一轮冻结现场。 */
    }
}

/**
 * JustFloat 24 通道（500Hz，DOC §12.1）
 * 单位：Ch9/13/14 角度均为 rad；Ch1/2 速度为输出轴 rpm。
 */
static inline void User_UserTX(void){
    const float RADS2RPM = USER_RADS2RPM;
    float mech_rad = BSP_MT6816_GetMechRad();  /* 仅取最近快照，VOFA 不直接访问 SPI3 */
    float elec_rad = Sguan.encoder.Real_Erad;
    float vbus_show = (Sguan.foc.Real_VBUS > 1.0f) ?
                      Sguan.foc.Real_VBUS : Sguan.motor.VBUS;
    float current0_live = ((float)ADC_InjectedValues[0] - (float)Sguan.current.Current_offset0) *
                          Sguan.current.Final_Gain * (float)Sguan.motor.Current_Dir0;
    float current1_live = ((float)ADC_InjectedValues[1] - (float)Sguan.current.Current_offset1) *
                          Sguan.current.Final_Gain * (float)Sguan.motor.Current_Dir1;
    uint8_t fault_frozen = g_foc_fault_snapshot.valid;

    Sguan.TXdata.fdata[0]  = (float)Sguan.status;       /* Ch0：0待命，1~3初始化，4 IDLE，5~13运行，14~23故障/安全态。 */
    Sguan.TXdata.fdata[1]  = Sguan.encoder.Real_Speed / GIM6010_GEAR_RATIO * RADS2RPM; /* Ch1：输出轴实测转速 [rpm]。 */
    Sguan.TXdata.fdata[2]  = Sguan.foc.Target_Speed / GIM6010_GEAR_RATIO * RADS2RPM;   /* Ch2：输出轴目标转速 [rpm]。 */
    Sguan.TXdata.fdata[3]  = Sguan.current.Real_Id;     /* Ch3：d 轴电流 Id [A]。 */
    Sguan.TXdata.fdata[4]  = Sguan.current.Real_Iq;     /* Ch4：q 轴电流 Iq [A]。 */
    Sguan.TXdata.fdata[5]  = (float)Sguan.foc.Duty_u;   /* Ch5：U 相逻辑 PWM 比较值 [count]，0~4249。 */
    /* Ch6 相邻帧增量应约为 40：20 kHz 高环 / 500 Hz VOFA。 */
    Sguan.TXdata.fdata[6]  = (float)(g_high_loop_cnt & 0xFFFFU);
    Sguan.TXdata.fdata[7]  = Sguan.foc.Uq_in;           /* Ch7：q 轴电压给定 Uq [V]。 */
    Sguan.TXdata.fdata[8]  = Sguan.current.Real_Ia;     /* Ch8：A 相实际电流 Ia [A]。 */
    Sguan.TXdata.fdata[9]  = Sguan.encoder.Real_Pos;    /* Ch9：PLL 连续机械位置（电机轴，位置环反馈） [rad]。 */
    Sguan.TXdata.fdata[10] = Sguan.encoder.Pos_offset;  /* Ch10：上电对齐后机械零位偏置 [rad]。 */
    Sguan.TXdata.fdata[11] = (float)Sguan.mode;         /* Ch11：控制模式：0开环、1电流、2速度、3位置。 */

    Sguan.TXdata.fdata[12] = vbus_show;                 /* Ch12：母线电压 [V]。 */
    Sguan.TXdata.fdata[13] = elec_rad;                  /* Ch13：SVPWM 实际电角度 θe [rad]。 */
    Sguan.TXdata.fdata[14] = mech_rad;                  /* Ch14：MT6816 单圈机械角 [rad]，0~2π。 */
    /* Ch15~23 为电流环诊断页。fault_frozen=1 时保留首次故障前的快照，不再显示已清零的实时量。 */
    Sguan.TXdata.fdata[15] = fault_frozen ? (float)g_foc_fault_snapshot.raw_iv : (float)ADC_InjectedValues[0];
    Sguan.TXdata.fdata[16] = fault_frozen ? (float)g_foc_fault_snapshot.raw_iw : (float)ADC_InjectedValues[1];
    Sguan.TXdata.fdata[17] = fault_frozen ? g_foc_fault_snapshot.offset0 : (float)Sguan.current.Current_offset0;
    Sguan.TXdata.fdata[18] = fault_frozen ? g_foc_fault_snapshot.offset1 : (float)Sguan.current.Current_offset1;
    Sguan.TXdata.fdata[19] = fault_frozen ? g_foc_fault_snapshot.current0 : current0_live;
    Sguan.TXdata.fdata[20] = fault_frozen ? g_foc_fault_snapshot.current1 : current1_live;
    Sguan.TXdata.fdata[21] = fault_frozen ? g_foc_fault_snapshot.pid_d_out : Sguan.control.Current_D.run.Output;
    Sguan.TXdata.fdata[22] = fault_frozen ? g_foc_fault_snapshot.pid_q_out : Sguan.control.Current_Q.run.Output;
    Sguan.TXdata.fdata[23] = fault_frozen ? g_foc_fault_snapshot.target_iq : Sguan.foc.Target_Iq;
    (void)s_vofa_last_BO;
    (void)s_vofa_last_CO;
    (void)s_vofa_last_VO;
}


#endif // __USERDATA_USERCONTROL_H
