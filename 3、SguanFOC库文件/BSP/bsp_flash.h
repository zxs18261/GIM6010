#ifndef __BSP_FLASH_H
#define __BSP_FLASH_H
/* GIM6010 内部 Flash 配置存储: 末2页双备份(见 02_设计文档\04_) */
#include <stdint.h>

/* STM32G431CBT6: 128KB, 2KB/页; 用第62/63页 */
#define FLASH_PAGE0_ADDR   0x0801F000u
#define FLASH_PAGE1_ADDR   0x0801F800u
#define FLASH_PAGE_SIZE    0x800u

#define CONFIG_MAGIC       0x47494D36u   /* "GIM6" */

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint32_t seq;            /* 写入序号(双页比较用) */
    uint32_t crc;            /* 以下字段的校验 */

    /* 节点配置(需求④) */
    uint8_t  node_id;        /* 0xFF=未分配 */
    uint8_t  node_valid;     /* 0xAA=有效 */
    uint16_t session_token;  /* CAN 会话 token */

    /* 电机标定(需求①) */
    uint8_t  calib_done;     /* 0xAA=标定完成 */
    float    pos_offset;     /* 编码器零位偏移 */
    int8_t   motor_dir;
    int8_t   pwm_dir;
    int8_t   encoder_dir;
    int8_t   current_dir0;
    int8_t   current_dir1;
    uint8_t  phase_remap[3]; /* 相序重映射 */
    int32_t  current_offset0;/* 0电流ADC偏置 IV */
    int32_t  current_offset1;/* 0电流ADC偏置 IW */

    uint8_t  reserved[128];
} config_t;

/* 返回 1=读到有效配置, 0=无(未配置/损坏) */
int  FLASH_ConfigLoad(config_t *cfg);
void FLASH_ConfigSave(config_t *cfg);
void FLASH_ConfigErase(void);     /* 恢复出厂: 双页全擦 */

#endif
