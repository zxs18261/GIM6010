#ifndef __BSP_MT6816_H
#define __BSP_MT6816_H
/* MT6816 14位磁编码器 SPI 驱动 (SPI3, Mode3, 16bit, CS=PA15) */
#include "main.h"
#include "spi.h"

/* 寄存器读命令: (0x80|addr)<<8, 返回寄存器内容在接收值的低字节 */
#define MT6816_CMD_ANGLE_HI   0x8300u   /* 读 0x03: Angle[13:6] */
#define MT6816_CMD_ANGLE_LO   0x8400u   /* 读 0x04: Angle[5:0] + bit1=磁场缺失 + bit0=偶校验 */

#define MT6816_CS_LOW()   HAL_GPIO_WritePin(MT6816_CSN_GPIO_Port, MT6816_CSN_Pin, GPIO_PIN_RESET)
#define MT6816_CS_HIGH()  HAL_GPIO_WritePin(MT6816_CSN_GPIO_Port, MT6816_CSN_Pin, GPIO_PIN_SET)

/* 返回 0~16383 的 14 位原始角度; 读失败(磁场缺失/校验错)返回上一次有效值并置 *err */
uint16_t MT6816_GetAngle(uint8_t *err);
float    MT6816_GetRad(void);     /* 0~2π 弧度(忽略错误, 供快速路径使用) */
void     MT6816_Init(void);

#endif // __BSP_MT6816_H
