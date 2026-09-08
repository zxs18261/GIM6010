/**
  ******************************************************************************
  * @file    BSP_MT6816.h
  * @brief   GIM6010 板载 MT6816 磁编码器 BSP（SPI3 + PA15 CSN）
  * @note    - 硬件：CSN=PA15 / SCK=PB3 / MISO=PB4 / MOSI=PB5（SPI3）
  *          - CubeMX SPI3 可能为占位配置，Init() 内重配为 Mode3 + 16bit
  *          - 编码器安装在电机转子侧：机械角 = 电机轴角；输出轴 = 电机轴 / 减速比
  *          - 采样由 SPI3 DMA 状态机完成；FOC 只能读取缓存快照，不能在 ISR 内轮询 SPI
  *
  * 使用示例：
  *   BSP_MT6816_Data_t enc;
  *   BSP_MT6816_Init();
  *   BSP_MT6816_Update(&enc);   // 周期调用
  *   // enc.elec_rad / enc.mech_rad / enc.shaft_omega_rpm ...
  ******************************************************************************
  */

#ifndef __BSP_MT6816_H
#define __BSP_MT6816_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ============================ 电机 / 编码器参数 ============================ */
#define BSP_MT6816_CPR              (16384U)    /*!< 14bit 单圈计数 */
#define BSP_MT6816_CPR_MASK         (BSP_MT6816_CPR - 1U)
#define BSP_MT6816_POLE_PAIRS       (14U)       /*!< GIM6010 24V 极对数 */
#define BSP_MT6816_GEAR_RATIO       (8.0f)      /*!< 减速比 8:1 */
#define BSP_MT6816_CACHE_HZ         (4000U)     /*!< 编码器目标采样频率 [Hz] */
#define BSP_MT6816_STALE_TIMEOUT_US (2000U)     /*!< 缓存超过此年龄视为失效 */

/* ============================ 状态 ======================================== */
typedef enum
{
    BSP_MT6816_OK = 0U,
    BSP_MT6816_ERR_SPI,
    BSP_MT6816_ERR_PARITY,
    BSP_MT6816_ERR_NO_MAGNET
} BSP_MT6816_Status_t;

/**
  * @brief 编码器测量结果
  * @note  mech_*  : 电机轴（编码器直接侧）
  *        shaft_* : 输出轴（mech / GEAR_RATIO）
  *        elec_*  : 电角度 = mech_single * POLE_PAIRS，归一化到 [0, 2π)
  */
typedef struct
{
    uint16_t raw14;                 /*!< 单圈原始计数 0~16383 */
    int32_t  turns;                 /*!< 电机轴多圈计数 */

    float elec_rad;                 /*!< 电角度 [rad]  0~2π */
    float elec_deg;                 /*!< 电角度 [deg]  0~360 */
    float mech_rad;                 /*!< 电机轴单圈机械角 [rad] 0~2π */
    float mech_deg;                 /*!< 电机轴单圈机械角 [deg] 0~360 */
    float mech_cont_rad;            /*!< 电机轴连续机械角 [rad] */
    float mech_cont_deg;            /*!< 电机轴连续机械角 [deg] */
    float shaft_rad;                /*!< 输出轴单圈角 [rad] 0~2π */
    float shaft_deg;                /*!< 输出轴单圈角 [deg] 0~360 */
    float shaft_cont_rad;           /*!< 输出轴连续角 [rad] */
    float shaft_cont_deg;           /*!< 输出轴连续角 [deg] */

    float mech_omega_rad_s;         /*!< 电机轴角速度 [rad/s] */
    float mech_omega_rpm;           /*!< 电机轴转速 [rpm] */
    float shaft_omega_rad_s;        /*!< 输出轴角速度 [rad/s] */
    float shaft_omega_rpm;          /*!< 输出轴转速 [rpm] */

    BSP_MT6816_Status_t status;     /*!< 本次更新状态 */
    uint32_t parity_err_cnt;        /*!< 校验错误累计 */
    uint32_t magnet_err_cnt;        /*!< 无磁/弱磁累计 */
    uint32_t sequence;              /*!< 成功采样序号 */
    uint32_t timestamp_cycles;      /*!< 该快照对应的 DWT 周期计数 */
} BSP_MT6816_Data_t;

/** @brief 异步采样链路诊断数据，用于 VOFA 或调试器观测。 */
typedef struct
{
    uint32_t request_cnt;            /*!< ADC ISR 请求次数 */
    uint32_t started_cnt;            /*!< 已启动 DMA 采样次数 */
    uint32_t completed_cnt;          /*!< 成功发布缓存次数 */
    uint32_t dropped_request_cnt;    /*!< DMA/主循环来不及服务而合并的请求数 */
    uint32_t dma_error_cnt;          /*!< DMA/SPI 传输错误次数 */
    uint32_t sample_age_us;          /*!< 当前缓存年龄 [us] */
} BSP_MT6816_Diagnostics_t;

/* ============================ API ========================================= */
void BSP_MT6816_Init(void);
void BSP_MT6816_SetInvert(uint8_t invert);
uint8_t BSP_MT6816_GetInvert(void);

/**
  * @brief 由 20 kHz ADC ISR 调用；仅递增请求序号，不访问 SPI、不等待 DMA。
  */
void BSP_MT6816_RequestSampleFromISR(void);

/**
  * @brief 主循环服务函数；仅在有待处理请求时启动 SPI3 DMA，不进行阻塞式 SPI 轮询。
  */
void BSP_MT6816_Service(void);

/** @brief 复制最近一次编码器快照；不触发任何 SPI 访问。 */
uint8_t BSP_MT6816_GetSnapshot(BSP_MT6816_Data_t *data);

/** @brief 当前缓存是否在 BSP_MT6816_STALE_TIMEOUT_US 内且状态有效。 */
uint8_t BSP_MT6816_IsSampleFresh(void);

/** @brief 当前缓存年龄 [us]；无有效样本时返回 UINT32_MAX。 */
uint32_t BSP_MT6816_GetSampleAgeUs(void);

/** @brief 读取异步采样链路统计；不触发任何 SPI 访问。 */
void BSP_MT6816_GetDiagnostics(BSP_MT6816_Diagnostics_t *diagnostics);

/**
 * @brief  兼容接口：复制缓存角度/速度，不再发起 SPI 轮询
  * @param  data 输出结构体，不可为 NULL
  * @retval 本次读取状态
  */
BSP_MT6816_Status_t BSP_MT6816_Update(BSP_MT6816_Data_t *data);

/**
 * @brief  兼容接口：复制缓存角度/速度，不再发起 SPI 轮询
  * @param  data 输出结构体
  * @param  dt_s 距上次更新的时间间隔 [s]，建议 > 0
  */
BSP_MT6816_Status_t BSP_MT6816_UpdateDt(BSP_MT6816_Data_t *data, float dt_s);

/** @brief 从缓存读取 14bit 原始角；无有效缓存时返回 0xFFFF */
uint16_t BSP_MT6816_ReadRaw14(void);

/**
 * @brief  从缓存读取电机轴单圈机械角 [rad]
 * @note   不访问 SPI；失败时返回上次有效值
 */
float BSP_MT6816_GetMechRad(void);

/** @brief FOC 专用机械角；按缓存速度预测至当前时刻，降低 4 kHz 缓存的相位滞后。 */
float BSP_MT6816_GetMechRadForControl(void);

/** @brief 快速取电角度 [rad] 0~2π */
float BSP_MT6816_GetElecRad(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_MT6816_H */
