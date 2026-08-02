#ifndef __USERDATA_CONFIG_H
#define __USERDATA_CONFIG_H
/* 电机控制User用户设置·数据计算 */

/**
 * @description: 宏定义0或1决定“闭环控制系统”是否使用PI控制(默认关闭)
 * @reminder: 0->电流环“PI控制”，转速环“STA二阶滑膜控制”，位置环“PD控制” 
 * @reminder: 1->电流环“PI控制”，转速环“PI控制”，位置环“PD控制”
 * @return {*}
 */
#define Open_PI_Control 0

/**
 * @description: 宏定义0或1决定“MTPA最大转矩控制控制”是否开启(默认关闭)
 * @reminder: 0->不开启IPMSM的MTPA最大转矩控制
 * @reminder: 1->开启最大转矩控制控制
 * @return {*}
 */
#define Open_MTPA_Calculate 0

/**
 * @description: 宏定义0或1决定“FW弱磁控制”是否开启(默认关闭)
 * @reminder: 0->不开启IPMSM的弱磁控制
 * @reminder: 1->开启弱磁控制
 * @return {*}
 */
#define Open_FW_Calculate 0

/**
 * @description: 宏定义0或1决定“Q31->float线程”是否开启(默认关闭)
 * @reminder: 0->不开启Q31->float线程
 * @reminder: 1->实时读取float的Real_Speed,Real_Pos,Real_Id,Real_Iq
 * @return {*}
 */
#define Open_float_Calculate 0

/**
 * @description: 宏定义0或1决定“Q31定点运算”是否开启(默认关闭)
 * @reminder: 0->不开启Q31浮点转定点运算
 * @reminder: 1->开启浮点转定点的运算(量纲，需要参数标幺化)
 * @return {*}
 */
#define Open_Q31_Calculate 0

/**
 * @description: Q31定点化运算的数据标幺(基值设计)
 * @reminder:BASE需要->电压，电流，电阻，磁链，电感，弧度，时间，频率
 * @return {*}
 */
#define Q_Time 0.0078125f                 	// 常量标幺化(单位为s)
#define Q_Rad 512.0f                        // 角度大小(单位为rad弧度)
#define Q_Current 64.0f                     // 电流数据设定(单位为A安培)
#define Q_Voltage 128.0f                    // 电压数据设定(单位为V伏特)
#define Q_Temperature 256.0f                // 温度标幺设计，只用于温控状态机(单位为℃)  
#define Q_Speed (Q_Rad/Q_Time)              // 角速度大小(单位rad/s)
#define Q_Hz (1.0f/Q_Time)                  // 频率Hz的大小(单位1/s)
#define Q_Inductor ((Q_Voltage*Q_Time)/Q_Current) // 电感数据设定(单位为H亨利)
#define Q_Flux (Q_Voltage*Q_Time)           // 磁链大小(单位为Wb韦伯)
#define Q_Resistor (Q_Voltage/Q_Current)    // 电阻数据设定(单位为Ω欧姆)

/**
 * @description: 宏定义决定UART或者CAN发送数据的模式
 * @reminder: (Open_Printf_Debug)0->发送正常数据
 * @reminder: 1->仅发送Debug数据，不发送正常数据
 * @return {*}
 */
#define Open_Printf_Debug 0


// 定时器中断参数设计
#define TIM_T 1e-4                          // 最大频率的控制周期

/* ================= GIM6010 24V 规格常量（见 资料/文档/GIM6010_电机参数规格书(24V).md） ================= */
#define GIM6010_GEAR_RATIO          8.0f        // 减速比 8:1
#define GIM6010_ENCODER_BITS        14          // 本板 MT6816 位数（模块内置为 16bit）
#define GIM6010_RATED_CURRENT_A     6.6f        // 额定电流 [A]
#define GIM6010_PEAK_CURRENT_A      30.47f      // 堵转/峰值电流 [A]
#define GIM6010_SHAFT_RATED_RAD_S   25.66f      // 输出轴额定角速度 [rad/s]（245 rpm）
#define GIM6010_MOTOR_RATED_RAD_S   205.3f      // 电机轴额定角速度 [rad/s]（1960 rpm）
#define GIM6010_KT_NM_A             0.61f       // 扭矩常数 [N·m/A]

/**
 * @description: 上电校准“转子回零”的 d 轴电压比例 Ud = 比例 × VBUS
 * @reminder: 原库默认 0.3（24V→7.2V）。Rs=0.44Ω 时理论电流仍可达十余安，
 *            2A 电源会 OCP。开环调试建议 0.03~0.06；电源≥5A 可略加大。
 */
#define USER_ALIGN_UD_RATIO     0.04f
#define USER_ALIGN_HOLD_MS      800U        // 回零保持时间 [ms]
#define USER_ALIGN_RELEASE_MS   400U        // 撤电压后再等 [ms]


#endif // USERDATA_CONFIG_H
