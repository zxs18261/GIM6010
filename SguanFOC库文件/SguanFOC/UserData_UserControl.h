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
#define USER_OPENLOOP_DT            (0.001f)  /* 与 TIM2 1kHz High_Loop 一致 */
#endif
#ifndef USER_OPENLOOP_UQ_START
#define USER_OPENLOOP_UQ_START      (0.3f)    /* |Uq| 低于此视为无力矩 */
#endif
#ifndef USER_OPENLOOP_WE_DEFAULT
#define USER_OPENLOOP_WE_DEFAULT    (50.0f)   /* 默认开环电角速度 [rad/s] */
#endif
#ifndef USER_OPENLOOP_WE_MAX
#define USER_OPENLOOP_WE_MAX        (1500.0f) /* VO 硬限幅 [rad/s 电]；过大易失步 */
#endif
/* V/f：|ωe|_use ≤ K·|Uq|，避免 VO 很大但 Uq 不够导致磁场空转、转子不动 */
#ifndef USER_OPENLOOP_VF_K
#define USER_OPENLOOP_VF_K          (25.0f)   /* Uq=4 → ωe≈100；Uq=8 → ωe≈200 */
#endif

static float s_ol_eangle = 0.0f;
static uint8_t s_ol_hold_synced = 0;

/**
 * 真开环：θe 积分旋转（与编码器无关）+ Uq 幅值
 *  - AO → Uq（电压/“转矩”感）
 *  - VO → Target_Speed = 电角速度 ωe [rad/s]（速度）
 * Ch13 = 开环 θe；Ch14 = 编码器机械角（实测）
 *
 * 注意：Calculate 已先用编码器算好 sine/cosine。开环有 Uq 时必须覆盖，
 * 否则 VO≈0 会退回「编码器角+Uq」= 持续 q 轴电压 → 空载飞转（感觉最快）。
 */
static inline void User_UserControl(void){
    if (Sguan.mode != Velocity_OPEN_MODE) {
        return;
    }
    if (Sguan.status <= 3 || Sguan.status >= 19) {
        return;
    }

    float uq = Sguan.foc.Uq_in;
    float auq = (uq >= 0.0f) ? uq : -uq;
    float we = Sguan.foc.Target_Speed; /* 开环：当作电角速度指令 */
    float awe = (we >= 0.0f) ? we : -we;

    /* 无力矩：不覆盖角度，可手拧 */
    if (auq < USER_OPENLOOP_UQ_START) {
        s_ol_hold_synced = 0;
        return;
    }

    if (awe < 1.0f) {
        /* VO≈0：固定开环角（定子磁场静止）→ 转子锁停，而不是跟着编码器飞转 */
        if (!s_ol_hold_synced) {
            s_ol_eangle = Sguan.encoder.Real_Erad;
            s_ol_hold_synced = 1;
        }
        we = 0.0f;
    } else {
        s_ol_hold_synced = 0;
        /* V/f 软限：实际积分用的 ωe，防止高 VO + 低 Uq 失步 */
        float we_vf = USER_OPENLOOP_VF_K * auq;
        if (awe > we_vf) {
            we = (we >= 0.0f) ? we_vf : -we_vf;
        }
        s_ol_eangle += we * USER_OPENLOOP_DT;
        if (s_ol_eangle >= Value_2PI) {
            s_ol_eangle -= Value_2PI;
        } else if (s_ol_eangle < 0.0f) {
            s_ol_eangle += Value_2PI;
        }
    }

    /* 只要有 Uq，开环角必须覆盖编码器角，供后续 SVPWM 使用 */
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
        Sguan.foc.Target_Iq = AO;
        break;
    case VelCur_DOUBLE_MODE:
        Sguan.foc.Target_Speed = AO;
        break;
    case PosVelCur_THREE_MODE:
        Sguan.foc.Target_Pos = AO;
        break;
    default:
        break;
    }
}

/* VO：仅开环下设置电角速度 ωe [rad/s]，写入 Target_Speed（Ch2 可看） */
static inline void User_VO_Adjust(float VO){
    if (VO > USER_OPENLOOP_WE_MAX) {
        VO = USER_OPENLOOP_WE_MAX;
    } else if (VO < -USER_OPENLOOP_WE_MAX) {
        VO = -USER_OPENLOOP_WE_MAX;
    }
    Sguan.foc.Target_Speed = VO;
    g_vofa_last_BO = VO; /* 借用回显槽：Ch 上看 Target_Speed 即可 */
}

static inline void User_BO_Adjust(float BO){
    g_vofa_last_BO = BO;
    if (BO < 1.0f){
        Sguan.mode = Velocity_OPEN_MODE;
        /* 进开环时若尚未设速度，给默认电角速度 */
        if (Sguan.foc.Target_Speed > -1.0f && Sguan.foc.Target_Speed < 1.0f) {
            Sguan.foc.Target_Speed = USER_OPENLOOP_WE_DEFAULT;
        }
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
 * Ch2 = Target_Speed → 开环时即 ωe [rad/s]
 * Ch7 = Uq；Ch13 = 开环电角度；Ch14 = 编码器机械角
 */
static inline void User_UserTX(void){
    const float RAD2DEG = 57.2957795f;
    float mech_rad = BSP_MT6816_GetMechRad();
    float elec_rad = Sguan.encoder.Real_Erad;
    float vbus_show = (Sguan.foc.Real_VBUS > 1.0f) ?
                      Sguan.foc.Real_VBUS : Sguan.motor.VBUS;

    Sguan.TXdata.fdata[0]  = (float)Sguan.status;
    Sguan.TXdata.fdata[1]  = Sguan.encoder.Real_Speed;
    Sguan.TXdata.fdata[2]  = Sguan.foc.Target_Speed;   /* 开环=ωe */
    Sguan.TXdata.fdata[3]  = Sguan.current.Real_Id;
    Sguan.TXdata.fdata[4]  = Sguan.current.Real_Iq;
    Sguan.TXdata.fdata[5]  = (float)Sguan.foc.Duty_u;
    Sguan.TXdata.fdata[6]  = (float)(g_high_loop_cnt & 0xFFFFU);
    Sguan.TXdata.fdata[7]  = Sguan.foc.Uq_in;
    Sguan.TXdata.fdata[8]  = Sguan.current.Real_Ia;
    Sguan.TXdata.fdata[9]  = Sguan.encoder.Real_Pos;
    Sguan.TXdata.fdata[10] = Sguan.encoder.Pos_offset;
    Sguan.TXdata.fdata[11] = (float)Sguan.mode;

    Sguan.TXdata.fdata[12] = vbus_show;
    Sguan.TXdata.fdata[13] = elec_rad;                  /* 开环 θe */
    Sguan.TXdata.fdata[14] = mech_rad;                  /* 编码器机械角 */
    Sguan.TXdata.fdata[15] = elec_rad * RAD2DEG;
    Sguan.TXdata.fdata[16] = mech_rad * RAD2DEG;
    Sguan.TXdata.fdata[17] = g_vofa_last_AO;
    (void)g_vofa_last_BO;
    (void)g_vofa_last_CO;
}


#endif // USERDATA_USERCONTROL_H
