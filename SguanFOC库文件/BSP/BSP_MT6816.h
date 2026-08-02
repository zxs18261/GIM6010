/**
  ******************************************************************************
  * @file    BSP_MT6816.h
  * @brief   GIM6010 板载 MT6816 磁编码器 BSP（SPI3 + PA15 CSN）
  * @note    - 硬件：CSN=PA15 / SCK=PB3 / MISO=PB4 / MOSI=PB5（SPI3）
  *          - CubeMX SPI3 可能为占位配置，Init() 内重配为 Mode3 + 16bit
  *          - 编码器安装在电机转子侧：机械角 = 电机轴角；输出轴 = 电机轴 / 减速比
  *          - 调用周期建议与控制环一致（如 1kHz / 10kHz），由 Update() 估算速度
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
} BSP_MT6816_Data_t;

/* ============================ API ========================================= */
void BSP_MT6816_Init(void);
void BSP_MT6816_SetInvert(uint8_t invert);
uint8_t BSP_MT6816_GetInvert(void);

/**
  * @brief  读取编码器并更新角度/速度（用内部 tick 估算 dt）
  * @param  data 输出结构体，不可为 NULL
  * @retval 本次读取状态
  */
BSP_MT6816_Status_t BSP_MT6816_Update(BSP_MT6816_Data_t *data);

/**
  * @brief  读取编码器并更新角度/速度（外部给定 dt）
  * @param  data 输出结构体
  * @param  dt_s 距上次更新的时间间隔 [s]，建议 > 0
  */
BSP_MT6816_Status_t BSP_MT6816_UpdateDt(BSP_MT6816_Data_t *data, float dt_s);

/** @brief 仅读 14bit 原始角；失败返回 0xFFFF */
uint16_t BSP_MT6816_ReadRaw14(void);

/**
  * @brief  快速取电机轴单圈机械角 [rad]，供 SguanFOC User_Encoder_ReadRad 使用
  * @note   失败时返回上次有效值
  */
float BSP_MT6816_GetMechRad(void);

/** @brief 快速取电角度 [rad] 0~2π */
float BSP_MT6816_GetElecRad(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_MT6816_H */
