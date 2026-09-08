#ifndef __USERDATA_MOTOR_H
#define __USERDATA_MOTOR_H
#include "SguanFOC.h"
/* 电机控制User用户设置·电机参数(SguanFOC用户核心代码)
 * 数据来源：资料/文档/GIM6010_电机参数规格书(24V).md（厂家 24V 版）
 * 安全阈值：过压50V / 欠压10V / 过温100℃（用户需求，DOC/GIM6010.md §5）
 */

// 电机实体参数设置(根据实际需要填写)
static inline void User_MotorSet(void){
    // 1.mode选择电机的运行模式
    // 注意：每次启动(UNINITIALIZED→INITIALIZING)时 User_MotorSet 都会执行，
    //       运行中切模式请用 VOFA "BO=x?" 命令，不要在此处设置。
    // 2.flag电机标志位
    Sguan.flag.PWM_watchdog_limit = 10;         // (uint8_t)PWM错误限幅

    // 3.identify — GIM6010 24V 厂家电气参数
    //    线电感 230.77 mH、线电阻 0.44 Ω、Kt=0.61 → ψ≈Kt/(1.5×P)
    //    若 LCR 实测为 µH 量级，将 Ld/Lq 改为 230.77e-6f 再试
    Sguan.identify.Ld   = 0.23077f;             // (float)D轴电感 [H] 线电感 230.77mH
    Sguan.identify.Lq   = 0.23077f;             // (float)Q轴电感 [H]
    Sguan.identify.Rs   = 0.44f;                // (float)线电阻 [Ω]
    Sguan.identify.Flux = 0.02905f;             // (float)磁链 [Wb] ≈0.61/(1.5×14)

    // 4.motor — 机械/驱动板参数
    Sguan.motor.Poles = 14;                     // (uint8_t)极对数
    Sguan.motor.VBUS  = 24.0f;                  // (float)额定母线 [V]


    //需要实验测试得出，主要是三相线有焊反的可能（校准流程见 DOC §8.2）
    Sguan.motor.Motor_Dir = 1;                  // (int8_t)电机方向1->正向，负1->负向
    Sguan.motor.PWM_Dir = -1;                   // (int8_t)PWM占空比高低对应1->正向，负1->负向
    Sguan.motor.Duty = 4250;                    // (uint16_t)PWM满占空比，对齐 TIM1 ARR+1

    //编码器方向（2026-08-30 台架实测标定，DOC §8.2）：
    //从控制器端看转子顺时针 = Ch14(MT6816机械角)增大 = 关节正方向；
    //库内控制角 = (BSP角-零位)×Encoder_Dir，故取 +1 使闭环"正"=顺时针=Ch14增大
    Sguan.motor.Encoder_Dir = 1;                // (int8_t)编码器方向1->正向，负1->负向


    //与PCB设计有关（下桥臂双电阻：IV=ADC1_IN12, IW=ADC1_IN15）
    /* 2026-08-31 VOFA 受限电流阶跃验证：Iq*=+0.05A 时 Iq 向负方向发散，
     * 说明两路采样极性均与控制坐标系相反。只翻转极性；V/W 映射保持不变。 */
    Sguan.motor.Current_Dir0 = -1;              // (int8_t)User_ReadADC_Raw(0) 的采样极性
    Sguan.motor.Current_Dir1 = -1;              // (int8_t)User_ReadADC_Raw(1) 的采样极性
    Sguan.motor.Current_Num = 2;                // (uint8_t)0=AB 1=AC 2=BC；本板 IV+IW=BC
    Sguan.motor.ADC_Precision = 4096;           // (uint32_t)ADC采样精度
    Sguan.motor.Amplifier = 20.0f;              // (float)INA181A2 增益 ×20
    Sguan.motor.MCU_Voltage = 3.3f;             // (float)ADC 参考电压
    Sguan.motor.Sampling_Rs = 0.005f;           // (float)采样电阻 5mΩ

    // 5.安全 — 用户要求：欠压10V / 过压50V / 过温100℃ / 过流30.47A(堵转)
    //    注意：VBUS 分压 1/11 硬件量程上限约 36.3V；BSP_ADC 将贴近满量程映射为
    //          明确过量程高值，因此即使配置阈值为 50V，也会在量程边界 fail-closed
    Sguan.safe.VBUS_MAX = 50.0f;                // (float)母线过压阈值 [V]（用户要求）
    Sguan.safe.VBUS_MIM = 10.0f;                // (float)母线欠压阈值 [V]（用户要求；库成员名即 VBUS_MIM）
    Sguan.safe.VBUS_watchdog_limit = 1000;

    /* 双路 NTC 独立保护（BSP_ADC）：板载(MOS旁)阈值100℃ / 电机内部(外接P1)阈值80℃。
     * 库只有单一 Temp_MAX 比较点 → BSP_ADC_GetTemp() 返回“严重度归一化”值
     * （各通道按自身阈值折算到 100℃ 比较点），故此处 Temp_MAX 固定为 100。 */
    Sguan.safe.Temp_MAX = 100.0f;               // (float)归一化过温比较点（板载100℃/电机80℃各自映射）
    Sguan.safe.Temp_MIN = -20.0f;               // (float)工作温度下限 [℃]
    Sguan.safe.Temp_watchdog_limit = 1000;

    Sguan.safe.Dcur_MAX = 30.47f;               // (float)堵转/峰值电流 [A]
    Sguan.safe.Qcur_MAX = 30.47f;               // (float)堵转/峰值电流 [A]
    Sguan.safe.DQcur_watchdog_limit = 1000;

    /* Current/Speed/Position_limit：状态机滞环带宽，不是额定值本身
     * 额定：6.6 A；输出轴额定 25.66 rad/s（245 rpm）；电机轴额定约 205 rad/s */
    Sguan.safe.Current_limit = 0.5f;
    Sguan.safe.Speed_limit = 5.0f;
    Sguan.safe.Position_limit = 1.0f;

    Sguan.safe.DISABLED_watchdog_limit = 1000;
}


#endif // __USERDATA_MOTOR_H
