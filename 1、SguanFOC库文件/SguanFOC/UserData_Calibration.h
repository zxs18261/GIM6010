#ifndef __USERDATA_CALIBRATION_H
#define __USERDATA_CALIBRATION_H

#include "SguanFOC.h"
#include <stdint.h>

/* 保存到 Flash 的电机/编码器预定位结果。 */
typedef struct {
    float position_offset_rad;
    int8_t encoder_dir;
    int8_t motor_dir;
    int8_t pwm_dir;
    int8_t current_dir0;
    int8_t current_dir1;
    uint8_t current_num;
    uint8_t pole_pairs;
} USER_CALIBRATION_DATA;

/* CAL?使用的只读Flash诊断快照；读取不会擦除、写入或应用任何电机参数。 */
typedef struct {
    USER_CALIBRATION_DATA data;
    uint32_t flash_address;
    uint32_t expected_magic;
    uint32_t stored_magic;
    uint32_t expected_format_version;
    uint32_t stored_format_version;
    uint32_t calculated_firmware_signature;
    uint32_t stored_firmware_signature;
    uint32_t calculated_checksum;
    uint32_t stored_checksum;
    uint32_t expected_commit_marker;
    uint32_t stored_commit_marker;
    uint8_t magic_valid;
    uint8_t format_valid;
    uint8_t firmware_valid;
    uint8_t checksum_valid;
    uint8_t commit_valid;
    uint8_t data_valid;
    uint8_t record_valid;
} USER_CALIBRATION_DIAGNOSTIC;

/* 返回 1 表示记录完整且与当前固件一致，否则必须重新预定位。 */
uint8_t User_Calibration_Load(USER_CALIBRATION_DATA *data);

/* 擦除并写入预留的最后一页 Flash，返回 1 表示回读校验通过。 */
uint8_t User_Calibration_Save(const USER_CALIBRATION_DATA *data);

/* 返回记录总有效性，同时无副作用地填充各项校验结果供CAL?显示。 */
uint8_t User_Calibration_ReadDiagnostic(USER_CALIBRATION_DIAGNOSTIC *diagnostic);

/* 便于调试和文档对照的固定存储地址。 */
uint32_t User_Calibration_GetFlashAddress(void);

#endif /* __USERDATA_CALIBRATION_H */
