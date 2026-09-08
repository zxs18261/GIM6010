#ifndef __USERDATA_USERCONTROL_H
#define __USERDATA_USERCONTROL_H
#include "SguanFOC.h"
/* 电机控制User用户设置·实时参数控制页面 */

/* 用户自己的CODE BEGIN Includes */
#include "BSP_MT6816.h"
#include "Timer.h"
#include "Sguan_IQmath.h"
/* 用户自己的CODE END Includes */

#ifndef USER_OPENLOOP_UQ_MAX_RATIO
#define USER_OPENLOOP_UQ_MAX_RATIO  (0.35f)   /* |Uq| 上限比例 */
#endif
#ifndef USER_OPENLOOP_DT
#define USER_OPENLOOP_DT            (TIM_T)   /* High_Loop 已与 ADC 注入完成的 20 kHz 同步 */
#endif
/* 保留旧 V/f 软限幅作为可选调试功能；默认关闭即为纯开环 FOC。 */
#ifndef USER_OPENLOOP_VF_ENABLE
#define USER_OPENLOOP_VF_ENABLE     (0U)      /* 0：VO 与 Uq 独立；1：用 Uq 限制电角速度 */
#endif
#ifndef USER_OPENLOOP_VF_K
#define USER_OPENLOOP_VF_K          (25.0f)   /* Uq=4 → ωe≈100；Uq=8 → ωe≈200 */
#endif

static float s_ol_eangle = 0.0f;
static uint8_t s_ol_angle_initialized = 0U;
static float s_ol_applied_shaft_speed = 0.0f;

/* VOFA 对外速度统一采用输出轴机械角速度 [rad/s]。 */
static inline float User_LimitShaftSpeed(float shaft_speed){
    if (shaft_speed > GIM6010_SHAFT_MAX_RAD_S) {
        return GIM6010_SHAFT_MAX_RAD_S;
    }
    if (shaft_speed < -GIM6010_SHAFT_MAX_RAD_S) {
        return -GIM6010_SHAFT_MAX_RAD_S;
    }
    return shaft_speed;
}

/**
 * 纯开环 FOC：θe 由 VO 独立积分，Ud 固定为 0，Uq 由 AO 独立给定。
 *  - AO → Uq [V]（开环转矩作用量，不是闭环校准的 N·m 转矩）
 *  - VO → 输出轴机械角速度 [rad/s]；Target_Speed 内部保存电机轴机械角速度
 * Ch13 = 开环 θe；Ch14 = 编码器机械角（实测）
 *
 * 注意：Calculate 会先计算编码器角度；模式 0 每个高频周期都必须用积分角覆盖，
 * 使用于 SVPWM 的角度不参与任何编码器速度闭环。
 */
static inline void User_UserControl(void){
    if (Sguan.mode != Velocity_OPEN_MODE) {
        s_ol_applied_shaft_speed = 0.0f;
        s_ol_angle_initialized = 0U; /* 再次进入开环时从当前转子角度平滑接管。 */
        return;
    }
    s_ol_applied_shaft_speed = 0.0f;
    if ((Sguan.status <= MOTOR_STATUS_CALIBRATING) ||
        (Sguan.status >= MOTOR_STATUS_OVERVOLTAGE)) {
        /* 14及以上均为硬件/安全故障，开环积分器不得继续推进旧速度。 */
        return;
    }

    /* Target_Speed 保持库内语义：电机轴机械角速度；开环时再换算为电角速度。 */
    float we = Sguan.foc.Target_Speed * (float)Sguan.motor.Poles;
    float awe = (we >= 0.0f) ? we : -we;
    float we_hard_max = GIM6010_SHAFT_MAX_RAD_S * GIM6010_GEAR_RATIO *
                        (float)Sguan.motor.Poles;

    /* 防御性硬限：即使 Target_Speed 被其他入口写入，也不得越过输出轴命令上限。 */
    if (awe > we_hard_max) {
        we = (we >= 0.0f) ? we_hard_max : -we_hard_max;
        awe = we_hard_max;
    }

    /* 首次进入开环时从当前编码器电角度接管，避免电压矢量突跳。 */
    if (s_ol_angle_initialized == 0U) {
        s_ol_eangle = Sguan.encoder.Real_Erad;
        s_ol_angle_initialized = 1U;
    }

    #if USER_OPENLOOP_VF_ENABLE
        /* 可选 V/f 软限：仅在显式开启时才让 Uq 限制电角速度。 */
        float uq = Sguan.foc.Uq_in;
        float auq = (uq >= 0.0f) ? uq : -uq;
        float we_vf = USER_OPENLOOP_VF_K * auq;
        if (awe > we_vf) {
            we = (we >= 0.0f) ? we_vf : -we_vf;
        }
    #endif

    /* VO 决定积分速度；默认不再受 Uq 大小影响。 */
    s_ol_applied_shaft_speed = we /
        ((float)Sguan.motor.Poles * GIM6010_GEAR_RATIO);
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

/* AO：开环=Uq；闭环按 mode 写 Target */
static inline void User_AO_Adjust(float AO){
    /* 预定位等待/执行期及故障期拒绝运动指令，不改写任何目标。 */
    if ((Sguan.status <= MOTOR_STATUS_CALIBRATING) ||
        (Sguan.status >= MOTOR_STATUS_OVERVOLTAGE)) {
        return;
    }
    g_vofa_last_AO = AO;
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
        /* AO 在速度模式同样按输出轴机械 rad/s 接收，库内换算为电机轴速度。 */
        Sguan.foc.Target_Speed = User_LimitShaftSpeed(AO) * GIM6010_GEAR_RATIO;
        break;
    case PosVelCur_THREE_MODE:
        Sguan.foc.Target_Pos = AO;
        break;
    default:
        break;
    }
}

/* VO：仅开环下设置输出轴机械角速度 [rad/s]。 */
static inline void User_VO_Adjust(float VO){
    /* 校准完成前的 VO 指令不缓存，避免刚进入 IDLE 就自动转动。 */
    if ((Sguan.status <= MOTOR_STATUS_CALIBRATING) ||
        (Sguan.status >= MOTOR_STATUS_OVERVOLTAGE) ||
        (Sguan.mode != Velocity_OPEN_MODE)) {
        return;
    }
    VO = User_LimitShaftSpeed(VO);
    Sguan.foc.Target_Speed = VO * GIM6010_GEAR_RATIO;
}

static inline void User_BO_Adjust(float BO){
    /* 校准期不预存模式切换，避免刚进入 IDLE 就启动闭环输出。 */
    if ((Sguan.status <= MOTOR_STATUS_CALIBRATING) ||
        (Sguan.status >= MOTOR_STATUS_OVERVOLTAGE)) {
        return;
    }
    g_vofa_last_BO = BO;
    if (BO < 1.0f){
        Sguan.mode = Velocity_OPEN_MODE;
        /* 仅切换模式，不自动填入转速；电机是否旋转完全由 VO/ AO 命令决定。 */
    } else if (BO < 2.0f){
        Sguan.mode = Current_SINGLE_MODE;
    } else if (BO < 3.0f){
        Sguan.mode = VelCur_DOUBLE_MODE;
    } else {
        Sguan.mode = PosVelCur_THREE_MODE;
    }
}

static inline void User_CO_Adjust(float CO){
    g_vofa_last_CO = CO;
    if (CO < 0.5f){
        Sguan.foc.Ud_in = 0.0f;
        Sguan.foc.Uq_in = 0.0f;
        Sguan.foc.Target_Id = 0.0f;
        Sguan.foc.Target_Iq = 0.0f;
        Sguan.foc.Target_Speed = 0.0f;
        Sguan.foc.Target_Pos = 0.0f;
        #if CONFIG_Q31
        /* 定点模式同步清零，防止下一个高频周期恢复旧目标。 */
        Sguan.foc.Ud_in_q31 = 0;
        Sguan.foc.Uq_in_q31 = 0;
        Sguan.foc.Target_Id_q31 = 0;
        Sguan.foc.Target_Iq_q31 = 0;
        Sguan.foc.Target_Speed_q31 = 0;
        Sguan.foc.Target_Pos_q31 = 0;
        #endif /* CONFIG_Q31 */
        g_vofa_last_AO = 0.0f;
        /*
         * CO=0 是显式停机：转入 STANDBY 后 High_Loop 不再运行控制器，
         * 避免位置环把清零后的 Target_Pos 当成新位置命令并再次开启 PWM。
         */
        Sguan.status = MOTOR_STATUS_STANDBY;
        User_PwmOutputDisable();
    } else {
        Sguan.status = MOTOR_STATUS_UNINITIALIZED;
    }
}

/**
 * 开环JustFloat固定14通道：状态/模式/母线/DQ电压/DQ电流/三种速度/双角度/MOE/帧率。
 * 极对数、方向、校准Flash和限幅等静态参数改由INFO?/CAL?查询，避免持续占用带宽。
 */
static inline void User_UserTX(void){
    float mech_rad = BSP_MT6816_GetMechRad();  /* 仅取 DMA 缓存，VOFA 不直接访问 SPI3 */
    float elec_rad = Sguan.encoder.Real_Erad;
    float vbus_measured = User_VBUS_DataGet();
    /* 包括 0V 在内的有效 ADC 值都应真实显示；仅 ADC 未就绪时回退到配置值。 */
    float vbus_show = (vbus_measured != -9999.0f) ?
                      vbus_measured : Sguan.motor.VBUS;
    float applied_speed_show = 0.0f;

    if ((Sguan.mode == Velocity_OPEN_MODE) &&
        (Sguan.status >= MOTOR_STATUS_IDLE) &&
        (Sguan.status <= MOTOR_STATUS_POSITION_HOLD)) {
        /* 故障状态不再显示旧的开环积分速度，避免与实际已关断输出矛盾。 */
        applied_speed_show = s_ol_applied_shaft_speed;
    }

    Sguan.TXdata.fdata[0]  = (float)Sguan.status;       /* Ch0：电机状态机编号。 */
    Sguan.TXdata.fdata[1]  = (float)Sguan.mode;         /* Ch1：0开环、1电流、2速度、3位置模式。 */
    Sguan.TXdata.fdata[2]  = vbus_show;                 /* Ch2：母线电压 [V]。 */
    Sguan.TXdata.fdata[3]  = Sguan.foc.Ud_in;           /* Ch3：实际使用的d轴电压Ud [V]。 */
    Sguan.TXdata.fdata[4]  = Sguan.foc.Uq_in;           /* Ch4：实际使用的q轴电压Uq [V]。 */
    Sguan.TXdata.fdata[5]  = Sguan.current.Real_Id;     /* Ch5：d轴电流Id [A]。 */
    Sguan.TXdata.fdata[6]  = Sguan.current.Real_Iq;     /* Ch6：q轴电流Iq [A]。 */
    Sguan.TXdata.fdata[7]  = Sguan.foc.Target_Speed / GIM6010_GEAR_RATIO; /* Ch7：限幅后输出轴目标速度 [rad/s]。 */
    Sguan.TXdata.fdata[8]  = applied_speed_show;        /* Ch8：开环积分实际使用的输出轴速度 [rad/s]。 */
    Sguan.TXdata.fdata[9]  = Sguan.encoder.Real_Speed / GIM6010_GEAR_RATIO; /* Ch9：编码器测得的输出轴速度 [rad/s]。 */
    Sguan.TXdata.fdata[10] = elec_rad;                  /* Ch10：SVPWM实际使用的电角度 [rad]。 */
    Sguan.TXdata.fdata[11] = mech_rad;                  /* Ch11：MT6816电机轴单圈机械角 [rad]。 */
    Sguan.TXdata.fdata[12] = (float)User_PwmOutputIsEnabled(); /* Ch12：TIM1硬件实际MOE。 */
    Sguan.TXdata.fdata[13] = 0.0f;                     /* Ch13：发送层在DMA启动前写入实际帧率 [Hz]。 */
    (void)g_vofa_last_BO;
    (void)g_vofa_last_CO;
}


#endif // USERDATA_USERCONTROL_H
