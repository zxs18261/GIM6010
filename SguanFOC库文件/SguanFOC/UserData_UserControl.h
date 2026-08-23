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
    if (Sguan.status <= 3 || Sguan.status >= 19) {
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
    if (Sguan.mode != Velocity_OPEN_MODE) {
        return;
    }
    VO = User_LimitShaftSpeed(VO);
    Sguan.foc.Target_Speed = VO * GIM6010_GEAR_RATIO;
}

static inline void User_BO_Adjust(float BO){
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
        g_vofa_last_AO = 0.0f;
    } else {
        Sguan.status = MOTOR_STATUS_UNINITIALIZED;
    }
}

/**
 * Ch1/Ch2 = 输出轴实测/目标机械角速度 [rad/s]
 * Ch7 = Uq；Ch13 = 开环电角度；Ch14 = 编码器机械角
 * Ch18 = 开环经过软件上限后真正用于积分的输出轴速度 [rad/s]
 * Ch19 = 输出轴最大峰值转矩 [N·m]；Ch20 = 输出轴软件命令速度上限 [rad/s]
 */
static inline void User_UserTX(void){
    const float RAD2DEG = 57.2957795f;
    float mech_rad = BSP_MT6816_GetMechRad();  /* 仅取 DMA 缓存，VOFA 不直接访问 SPI3 */
    float elec_rad = Sguan.encoder.Real_Erad;
    float vbus_show = (Sguan.foc.Real_VBUS > 1.0f) ?
                      Sguan.foc.Real_VBUS : Sguan.motor.VBUS;
    float applied_speed_show = 0.0f;

    if ((Sguan.mode == Velocity_OPEN_MODE) &&
        (Sguan.status > 3) && (Sguan.status < 19)) {
        applied_speed_show = s_ol_applied_shaft_speed;
    }

    Sguan.TXdata.fdata[0]  = (float)Sguan.status;       /* Ch0：电机状态机编号；4(IDLE)及以上表示已完成初始化。 */
    Sguan.TXdata.fdata[1]  = Sguan.encoder.Real_Speed / GIM6010_GEAR_RATIO; /* Ch1：输出轴实测机械角速度 [rad/s]。 */
    Sguan.TXdata.fdata[2]  = Sguan.foc.Target_Speed / GIM6010_GEAR_RATIO;   /* Ch2：输出轴目标机械角速度 [rad/s]。 */
    Sguan.TXdata.fdata[3]  = Sguan.current.Real_Id;     /* Ch3：d 轴电流 Id [A]，通常用于励磁/弱磁观测。 */
    Sguan.TXdata.fdata[4]  = Sguan.current.Real_Iq;     /* Ch4：q 轴电流 Iq [A]，与电磁转矩直接相关。 */
    Sguan.TXdata.fdata[5]  = (float)Sguan.foc.Duty_u;   /* Ch5：U 相逻辑 PWM 比较值 [count]，范围约 0~TIM1 ARR(4249)。 */
    /* Ch6 相邻帧增量应约为 40：20 kHz 高环 / 500 Hz VOFA。 */
    Sguan.TXdata.fdata[6]  = (float)(g_high_loop_cnt & 0xFFFFU);
    Sguan.TXdata.fdata[7]  = Sguan.foc.Uq_in;           /* Ch7：q 轴电压给定 Uq [V]；开环时是产生转矩的直接电压指令。 */
    Sguan.TXdata.fdata[8]  = Sguan.current.Real_Ia;     /* Ch8：A 相实际电流 Ia [A]。 */
    Sguan.TXdata.fdata[9]  = Sguan.encoder.Real_Pos;    /* Ch9：PLL 估算的相对机械位置 [rad]；位置模式外通常归一化到一圈。 */
    Sguan.TXdata.fdata[10] = Sguan.encoder.Pos_offset;  /* Ch10：上电对齐后得到的机械零位偏置 [rad]。 */
    Sguan.TXdata.fdata[11] = (float)Sguan.mode;         /* Ch11：控制模式：0开环、1电流环、2速度-电流双环、3位置-速度-电流三环。 */

    Sguan.TXdata.fdata[12] = vbus_show;                 /* Ch12：母线电压显示值 [V]；无实测时回退为配置值 VBUS。 */
    Sguan.TXdata.fdata[13] = elec_rad;                  /* Ch13：SVPWM 实际使用的电角度 θe [rad]；开环时为积分生成的角度。 */
    Sguan.TXdata.fdata[14] = mech_rad;                  /* Ch14：MT6816 读取的单圈机械角 [rad]，范围约 0~2π。 */
    Sguan.TXdata.fdata[15] = elec_rad * RAD2DEG;        /* Ch15：Ch13 的角度制显示值 [deg]。 */
    Sguan.TXdata.fdata[16] = mech_rad * RAD2DEG;        /* Ch16：Ch14 的角度制显示值 [deg]。 */
    Sguan.TXdata.fdata[17] = g_vofa_last_AO;            /* Ch17：最近收到的原始 AO，单位随模式为 V/A/(rad/s)/rad。 */
    Sguan.TXdata.fdata[18] = applied_speed_show;        /* Ch18：开环实际用于积分的输出轴速度 [rad/s]；默认与 Uq 无关。 */
    Sguan.TXdata.fdata[19] = GIM6010_OUTPUT_PEAK_TORQUE_NM; /* Ch19：输出轴瞬时峰值转矩 [N·m]，不可连续使用。 */
    Sguan.TXdata.fdata[20] = GIM6010_SHAFT_MAX_RAD_S;   /* Ch20：输出轴软件命令速度上限 [rad/s]（额定的1.2倍）。 */
    (void)g_vofa_last_BO;
    (void)g_vofa_last_CO;
}


#endif // USERDATA_USERCONTROL_H
