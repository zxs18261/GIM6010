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


/*
 * 控制时基：TIM1=170 MHz、中心对齐、ARR=4249 -> 20 kHz PWM。
 * High_Loop 由 ADC1 注入转换完成中断驱动；只有实测该回调为 20 kHz 时才能使用此值。
 * 若 CubeMX 修改 TIM1 ARR 或 ADC 触发次数，必须同步更新这里的宏。
 */
#define USER_FOC_CONTROL_HZ       (20000U)
#define USER_TIM1_PWM_ARR         (4249U)
#define TIM_T                     (1.0f / (float)USER_FOC_CONTROL_HZ)

/* MT6816 每 5 个 20 kHz FOC 周期请求一次 SPI DMA 采样，即 4 kHz。 */
#define USER_ENCODER_SAMPLE_HZ    (4000U)
#define USER_ENCODER_SAMPLE_DIV   (USER_FOC_CONTROL_HZ / USER_ENCODER_SAMPLE_HZ)

/* JustFloat：14通道=60 B，921600-8N1下500 Hz约占32.6%链路带宽。 */
#define USER_VOFA_TX_HZ           (500U)
#define USER_VOFA_TX_PERIOD_MS    (1000U / USER_VOFA_TX_HZ)

/* 调试事件仅保存在本次上电RAM中；满队列覆盖最旧记录并统计溢出。 */
#define USER_DEBUG_EVENT_CAPACITY   (32U)
#define USER_DEBUG_TEXT_BUFFER_SIZE (512U)
#define USER_DEBUG_RX_COMMAND_MAX   (32U)

/* 20 kHz 下沿用库的 Response / Response² 分频：速度环 1 kHz、位置环 50 Hz。
 * 这是首次闭环上电的保守起点；位置环参数整定完成后再单独提高。 */
#define USER_CONTROL_RESPONSE     (20U)

/* ================= GIM6010 24V 规格常量（见 资料/文档/GIM6010_电机参数规格书(24V).md） ================= */
#define GIM6010_GEAR_RATIO              8.0f        // 减速比 8:1
#define GIM6010_ENCODER_BITS            14          // 本板 MT6816 位数（模块内置为 16bit）
#define GIM6010_RATED_CURRENT_A         6.6f        // 额定连续电流 [A]
#define GIM6010_PEAK_CURRENT_A          30.47f      // 堵转/瞬时峰值电流 [A]
#define GIM6010_OUTPUT_RATED_TORQUE_NM  4.85f       // 输出轴额定连续转矩 [N·m]
#define GIM6010_OUTPUT_PEAK_TORQUE_NM   12.39f      // 输出轴堵转/瞬时峰值转矩 [N·m]
#define GIM6010_SHAFT_RATED_RPM         245.0f      // 输出轴额定转速 [rpm]
#define GIM6010_SHAFT_RATED_RAD_S       25.656340f  // 输出轴额定角速度 [rad/s]
#define GIM6010_SPEED_MARGIN_RATIO      1.2f        // 用户要求：速度命令预留 1.2 倍额定裕量
#define GIM6010_SHAFT_MAX_RAD_S         (GIM6010_SHAFT_RATED_RAD_S * GIM6010_SPEED_MARGIN_RATIO)
#define GIM6010_MOTOR_RATED_RAD_S       (GIM6010_SHAFT_RATED_RAD_S * GIM6010_GEAR_RATIO)
#define GIM6010_MOTOR_MAX_RAD_S         (GIM6010_SHAFT_MAX_RAD_S * GIM6010_GEAR_RATIO)
#define GIM6010_KT_NM_A                 0.61f       // 厂家扭矩常数 [N·m/A]，轴侧定义待台架确认

/**
 * @description: 上电校准“转子回零”的 d 轴电压比例 Ud = 比例 × VBUS
 * @reminder: 原库默认 0.3（24V→7.2V）。Rs=0.44Ω 时理论电流仍可达十余安，
 *            2A 电源会 OCP。开环调试建议 0.03~0.06；电源≥5A 可略加大。
 */
#define USER_ALIGN_UD_RATIO     0.04f
#define USER_ALIGN_HOLD_MS      800U        // 回零保持时间 [ms]
#define USER_ALIGN_RELEASE_MS   400U        // 撤电压后再等 [ms]
/* 更换电机或需强制重做预定位时递增此值并重新编译/烧录。 */
#define USER_CALIBRATION_EPOCH          1U

/* 首次预定位的母线门限：弱电供电但 VBUS<10V 时只等待，不会使能三相桥。 */
#define USER_ALIGN_VBUS_READY_V       10.0f
#define USER_ALIGN_VBUS_STABLE_MS     200U

/* 相序自检：电角度正向走一圈再逆向返回，以编码器方向为基准校正 Motor_Dir。 */
#define USER_PHASE_CHECK_ENABLE        1U
#define USER_PHASE_CHECK_RAMP_MS       800U
#define USER_PHASE_CHECK_SETTLE_MS     100U
/* 最小可信机械位移；0.01 rad 兼容电机端和减速后编码器。 */
#define USER_PHASE_CHECK_MIN_RAD       0.01f

/* 普通运行时 DQ 电压均小于此值就关闭 MOE，让电机真正进入高阻空闲。 */
#define USER_PWM_ENABLE_MIN_V         0.02f

/* 欠压连续20 ms才锁存，过滤ADC启动首样本；过压仅确认1 ms，避免延迟危险关断。 */
#define USER_VBUS_UNDERVOLT_CONFIRM_MS 20U
#define USER_VBUS_OVERVOLT_CONFIRM_MS   1U
/* 欠压/过压恢复各保留1 V滞环，避免阈值附近反复进入和退出故障。 */
#define USER_VBUS_RECOVERY_HYST_V       1.0f


#endif // USERDATA_CONFIG_H
