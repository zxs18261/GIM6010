#ifndef __USERDATA_USERCONTROL_H
#define __USERDATA_USERCONTROL_H
#include "SguanFOC.h"
/* 电机控制User用户设置·实时参数控制页面
 * GIM6010 关节控制器：VOFA AO/BO/CO 命令语义 + JustFloat 21 通道（DOC §12.1/§13）
 * 单位约定：内部 rad/rad·s⁻¹；显示层换算 deg / rpm（输出轴 = 电机轴 / 减速比8）
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
#ifndef USER_OPENLOOP_DEF_SHAFT_SPEED
#define USER_OPENLOOP_DEF_SHAFT_SPEED (5.0f)  /* 进入开环模式时默认输出轴速度 [rad/s] */
#endif
#ifndef USER_COMMAND_ENABLE_EPS
#define USER_COMMAND_ENABLE_EPS      (0.001f) /* 小于该值的 AO 不打开功率桥 */
#endif

/* 最近命令缓存（User_UserTX Ch17 观测） */
static float s_vofa_last_AO = 0.0f;
static float s_vofa_last_BO = 0.0f;
static float s_vofa_last_CO = 0.0f;

/* ---------------- 开环（模式0）电角度积分器 ---------------- */
static float s_ol_eangle = 0.0f;
static uint8_t s_ol_angle_initialized = 0U;

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

/* IDLE 是校准后的安全待机：只有显式的非零 AO 命令才能离开该状态。 */
static inline void User_StartMotionFromIdle(float command){
    if ((Sguan.status != MOTOR_STATUS_IDLE) ||
        (command <= USER_COMMAND_ENABLE_EPS && command >= -USER_COMMAND_ENABLE_EPS)) {
        return;
    }

    User_PwmOutputEnable();
    switch (Sguan.mode) {
    case Current_SINGLE_MODE:
    case Velocity_OPEN_MODE:
        Sguan.status = MOTOR_STATUS_TORQUE_CONTROL;
        break;
    case VelCur_DOUBLE_MODE:
        Sguan.status = MOTOR_STATUS_ACCELERATING;
        break;
    case PosVelCur_THREE_MODE:
    default:
        Sguan.status = MOTOR_STATUS_POSITION_INCREASING;
        break;
    }
}

/**
 * 开环 FOC：θe 由 Target_Speed 积分，Ud=0，Uq 由 AO 独立给定。
 * Target_Speed 保持库内语义：电机轴机械角速度 [rad/s]。
 * 注意：Calculate 先算编码器角；模式0 每个高频周期都用积分角覆盖，
 * 使 SVPWM 使用的角度不参与任何编码器速度闭环。
 */
static inline void User_UserControl(void){
    if (Sguan.mode != Velocity_OPEN_MODE) {
        s_ol_angle_initialized = 0U; /* 再次进入开环时从当前转子角度平滑接管。 */
        return;
    }
    if (Sguan.status <= 3 || Sguan.status >= 19) {
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
    if (s_ol_angle_initialized == 0U) {
        s_ol_eangle = Sguan.encoder.Real_Erad;
        s_ol_angle_initialized = 1U;
    }

    s_ol_eangle += we * USER_OPENLOOP_DT;
    while (s_ol_eangle >= Value_2PI) {
        s_ol_eangle -= Value_2PI;
    }
    while (s_ol_eangle < 0.0f) {
        s_ol_eangle += Value_2PI;
    }

    /* 纯开环每周期都强制 Ud=0，并用积分角覆盖编码器角供 SVPWM 使用。 */
    fast_sin_cos(s_ol_eangle, &Sguan.foc.sine, &Sguan.foc.cosine);
    Sguan.foc.Ud_in = 0.0f;
    Sguan.encoder.Real_Erad = s_ol_eangle; /* VOFA Ch13 看开环电角度 */
}

/* AO：按模式写目标（位置模式=本期主用：目标位置 rad，电机轴连续角，相对定位） */
static inline void User_AO_Adjust(float AO){
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
        /* 位置环：目标为电机轴连续机械角 [rad]（相对进入模式时的位置，可多圈） */
        Sguan.foc.Target_Pos = AO;
        break;
    default:
        break;
    }
    User_StartMotionFromIdle(AO);
}

static inline void User_BO_Adjust(float BO){
    s_vofa_last_BO = BO;
    float prev_mode = (float)Sguan.mode;
    if (BO < 1.0f){
        Sguan.mode = Velocity_OPEN_MODE;
    } else if (BO < 2.0f){
        Sguan.mode = Current_SINGLE_MODE;
    } else if (BO < 3.0f){
        Sguan.mode = VelCur_DOUBLE_MODE;
    } else {
        Sguan.mode = PosVelCur_THREE_MODE;
    }
    /* 进入开环时给一个温和默认转速（原版串口协议无 VO 通道） */
    if ((Sguan.mode == Velocity_OPEN_MODE) && (prev_mode != (float)Velocity_OPEN_MODE)) {
        Sguan.foc.Target_Speed = USER_OPENLOOP_DEF_SHAFT_SPEED * GIM6010_GEAR_RATIO;
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
    s_vofa_last_AO = 0.0f;
}

static inline void User_CO_Adjust(float CO){
    s_vofa_last_CO = CO;
    User_ClearMotionCommand();

    if (CO < 0.5f){
        /* 串口停机必须物理关闭功率桥；仅把 Ud/Uq 写零仍会保留 PWM 输出。 */
        Sguan.status = MOTOR_STATUS_STANDBY;
        User_PwmOutputDisable();
        return;
    }

    /* 重新初始化：STANDBY/任意硬件故障 → UNINITIALIZED → 自动对齐+进 IDLE。
     * MOTOR_STATUS_OVERCURRENT 的编号为 18，不能再用“>=19”漏掉它。 */
    if ((Sguan.status == MOTOR_STATUS_STANDBY) ||
        (Sguan.status >= MOTOR_STATUS_OVERVOLTAGE)) {
        g_foc_fault_snapshot.valid = 0U; /* 新一轮 CO 启动前清除上一轮冻结现场。 */
        Sguan.status = MOTOR_STATUS_UNINITIALIZED;
    }
}

/**
 * JustFloat 24 通道（500Hz，DOC §12.1）
 * 单位：角度显示 deg、速度显示 rpm（输出轴）；Ch13/14 保留 rad 原值便于波形对齐。
 */
static inline void User_UserTX(void){
    const float RAD2DEG = USER_RAD2DEG;
    const float RADS2RPM = USER_RADS2RPM;
    float mech_rad = BSP_MT6816_GetMechRad();  /* 仅取 DMA 缓存，VOFA 不直接访问 SPI3 */
    float elec_rad = Sguan.encoder.Real_Erad;
    float vbus_show = (Sguan.foc.Real_VBUS > 1.0f) ?
                      Sguan.foc.Real_VBUS : Sguan.motor.VBUS;
    float current0_live = ((float)ADC_InjectedValues[1] - (float)Sguan.current.Current_offset0) *
                          Sguan.current.Final_Gain * (float)Sguan.motor.Current_Dir0;
    float current1_live = ((float)ADC_InjectedValues[0] - (float)Sguan.current.Current_offset1) *
                          Sguan.current.Final_Gain * (float)Sguan.motor.Current_Dir1;
    uint8_t fault_frozen = g_foc_fault_snapshot.valid;

    Sguan.TXdata.fdata[0]  = (float)Sguan.status;       /* Ch0：电机状态机编号；4(IDLE)及以上表示已完成初始化。 */
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
}


#endif // __USERDATA_USERCONTROL_H
