#ifndef __USERDATA_CONFIG_H
#define __USERDATA_CONFIG_H
/* 电机控制User用户设置·数据计算
 * GIM6010 关节控制器：功能开关 + 控制时基 + 机械常量（DOC/GIM6010.md §11.2）
 */

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
/* 当前 GIM6010 上位机/遥测集成只完成并审查了 float 路径；库内遗留 Q31
 * 分支并不具备等价的命令缓存和最终电压限幅，禁止误开后生成不安全固件。 */
#if Open_Q31_Calculate
#error "GIM6010 integration supports float FOC only; keep Open_Q31_Calculate at 0"
#endif

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
/* 四模式上位机交付依赖 JustFloat 24 通道；旧逐字符 printf 调试路径不具备
 * 等价的 DMA 缓冲与带宽约束，禁止在本工程中误开。 */
#if Open_Printf_Debug
#error "GIM6010 four-mode integration requires Open_Printf_Debug at 0"
#endif


/*
 * 控制时基：TIM1=170 MHz、中心对齐、ARR=4249 -> 20 kHz PWM。
 * High_Loop 由 ADC1 注入转换完成中断驱动（TIM1_CC4 硬件触发，每 PWM 周期 1 次）。
 * 若 CubeMX 修改 TIM1 ARR 或 ADC 触发次数，必须同步更新这里的宏。
 */
#define USER_FOC_CONTROL_HZ       (20000U)
#define USER_TIM1_PWM_ARR         (4249U)
#define TIM_T                     (1.0f / (float)USER_FOC_CONTROL_HZ)
/* TIM2 每 1 ms 检查一次 ADC 注入完成计数；连续无更新达到该值即锁 SENSOR_ERROR。
 * 这是软件失效安全超时，不替代驱动器硬件 OCP/定时器 Break。 */
#define USER_ADC_HIGH_LOOP_TIMEOUT_MS   (5U)
/* 单次偶发抖动不立即锁死；连续三个 50us 控制周期达到/超过预算时关桥。 */
#define USER_FOC_DEADLINE_MISS_LIMIT    (3U)

/* MT6816 当前采用主循环内、HAL_GetTick 门控的最高 1 kHz 阻塞采样；
 * 20 kHz FOC ISR 和 VOFA 都只读取已发布缓存。BSP 内保留的 SPI DMA
 * request/service 路径尚未恢复，不能把它描述为当前 4 kHz 采样链。 */

/* JustFloat：24 通道 = 100 B，921600-8N1 下 500 Hz 约占 54% 链路带宽。
 * 原版库 Printf_Normal_Loop 不限速，500Hz 限速在 User_CorrespondSet 内实现。 */
#define USER_VOFA_TX_HZ           (500U)
#define USER_VOFA_TX_PERIOD_MS    (1000U / USER_VOFA_TX_HZ)

/* 20 kHz 下沿用库的 Response / Response² 分频：速度环 1 kHz、位置环 50 Hz。
 * 这是首次闭环上电的保守起点；位置环参数整定完成后再单独提高。 */
#define USER_CONTROL_RESPONSE     (20U)

/* 闭环台架验证安全开关：先验证反馈极性/相序，再恢复各闭环模式的电机参数前馈。
 * 所有闭环模式在进入 SVPWM 前还会经过最终 DQ 电压限制；0.30V 足以观察
 * AO=0.01A 的反馈方向，同时把映射/角度错误时的稳态相电流限制在约 0.7A。 */
#define USER_CURRENT_FEEDFORWARD_ENABLE       0
#define USER_CLOSED_LOOP_DQ_VOLTAGE_LIMIT_V   0.30f

/* ================= GIM6010 24V 规格常量（见 资料/文档/GIM6010_电机参数规格书(24V).md） ================= */
#define GIM6010_GEAR_RATIO              8.0f        // 减速比 8:1
#define GIM6010_ENCODER_BITS            14          // 本板 MT6816 位数（模块内置为 16bit）
#define GIM6010_RATED_CURRENT_A         6.6f        // 额定连续电流 [A]
#define GIM6010_PEAK_CURRENT_A          30.47f      // 堵转/瞬时峰值电流 [A]
#define GIM6010_OUTPUT_RATED_TORQUE_NM  4.85f       // 输出轴额定连续转矩 [N·m]
#define GIM6010_OUTPUT_PEAK_TORQUE_NM   12.39f      // 输出轴堵转/瞬时峰值转矩 [N·m]
#define GIM6010_SHAFT_RATED_RPM         245.0f      // 输出轴额定转速 [rpm]
#define GIM6010_SHAFT_RATED_RAD_S       25.656340f  // 输出轴额定角速度 [rad/s]
#define GIM6010_SPEED_MARGIN_RATIO      1.2f        // 速度命令预留 1.2 倍额定裕量
#define GIM6010_SHAFT_MAX_RAD_S         (GIM6010_SHAFT_RATED_RAD_S * GIM6010_SPEED_MARGIN_RATIO)
#define GIM6010_MOTOR_RATED_RAD_S       (GIM6010_SHAFT_RATED_RAD_S * GIM6010_GEAR_RATIO)
#define GIM6010_MOTOR_MAX_RAD_S         (GIM6010_SHAFT_MAX_RAD_S * GIM6010_GEAR_RATIO)
#define GIM6010_KT_NM_A                 0.61f       // 厂家扭矩常数 [N·m/A]

/**
 * @description: 上电校准“转子回零”的 d 轴电压比例 Ud = 比例 × VBUS
 * @reminder: 原库 Sguan_Start_Tick 内部固定 0.3×VBUS（24V→7.2V，Rs=0.44Ω 理论 16A，
 *            小电源必 OCP）。不改算法文件，在 User_PwmDuty_Set 内对 CALIBRATING 态
 *            占空比按本比例缩放：0.3×24V→0.04×24V≈0.96V；按 0.44Ω 静态估算约
 *            2.2A，实际电流及台架电源限流必须实测确认。
 */
#define USER_ALIGN_UD_RATIO     0.04f
#define USER_ALIGN_UD_LIB_RATIO 0.30f       /* 库内固定值，缩放系数 = USER_ALIGN_UD_RATIO/此值 */
/* 对齐带电阶段的独立三相瞬时限流。3.0A 是首次低压台架的保守起点，未经过
 * 当前电机/采样链实测；烧录前必须结合电源限流、偏置噪声和实际对齐电流确认。 */
#define USER_ALIGN_CURRENT_LIMIT_A 3.0f

/* 单位换算常量：当前 JustFloat 角度通道保持 rad，仅输出轴速度换算为 rpm；
 * USER_RAD2DEG 保留给可选上位机显示换算，活动遥测代码不使用。 */
#define USER_RAD2DEG             57.2957795f
#define USER_RADS2RPM            9.5492966f   /* rpm = rad/s × 60/2π */


#endif // __USERDATA_CONFIG_H
