/*
 * BSP_MT6816.c - MT6816 14bit 磁编码器驱动 (SPI3, Mode3, 16bit frame, CS=PA15)
 *
 * 数据手册要点(已与多来源核对):
 *  - 读角度需两次 16bit 寄存器读: 发 0x8300(读0x03:Angle[13:6])、0x8400(读0x04:Angle[5:0]+状态)
 *  - SPI 返回帧内容在"接收值低字节"; 组合 angle = (hi<<6) | (lo>>2)
 *  - 0x04 bit1 = 磁通不足(磁场缺失)置1; bit0 = 偶校验(0x03全8bit + 0x04 bit7:1 的1个数为偶)
 *  - 本驱动在 High_Loop(10kHz) 内被调用, 使用阻塞收发, 单次约3us @5.3MHz 可接受
 */
#include "BSP_MT6816.h"

static uint16_t last_angle = 0;   /* 上次有效角度, 读失败时保持 */

void MT6816_Init(void){
    MT6816_CS_HIGH();             /* 片选初始无效电平 */
    last_angle = 0;
}

/* 单次16位帧交换: 返回接收值(寄存器内容位于低字节) */
static uint16_t MT6816_ReadReg(uint16_t cmd){
    uint16_t rx = 0;
    MT6816_CS_LOW();
    HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&cmd, (uint8_t*)&rx, 1, HAL_MAX_DELAY);
    MT6816_CS_HIGH();
    return rx;
}

/* 偶校验: 若 data 中 1 的个数为偶返回 1 */
static uint8_t MT6816_EvenParity(uint16_t data){
    uint8_t cnt = 0;
    while (data){ cnt ^= 1u; data &= (uint16_t)(data - 1u); } /* 每次清除最低1位, 统计奇偶 */
    return (uint8_t)(cnt ^ 1u);   /* 偶数个1 -> 1 */
}

uint16_t MT6816_GetAngle(uint8_t *err){
    uint8_t  e = 1;               /* 默认失败 */
    uint16_t hi = MT6816_ReadReg(MT6816_CMD_ANGLE_HI) & 0x00FFu;
    uint16_t lo = MT6816_ReadReg(MT6816_CMD_ANGLE_LO) & 0x00FFu;

    if (!(lo & 0x02u)){           /* 磁场存在(bit1=0) */
        /* 组合14位角度 */
        uint16_t comb = (uint16_t)((hi << 8) | lo);
        if (MT6816_EvenParity(comb)){       /* 偶校验通过 */
            last_angle = (uint16_t)((hi << 6) | (lo >> 2));
            e = 0;
        }
    }
    if (err) *err = e;
    return last_angle;
}

float MT6816_GetRad(void){
    /* 快速路径: 忽略偶校验/磁场位(磁场缺失时仍返回旧值), 供10kHz FOC路径使用 */
    uint16_t hi = MT6816_ReadReg(MT6816_CMD_ANGLE_HI) & 0x00FFu;
    uint16_t lo = MT6816_ReadReg(MT6816_CMD_ANGLE_LO) & 0x00FFu;
    if (!(lo & 0x02u)){
        last_angle = (uint16_t)((hi << 6) | (lo >> 2));
    }
    return (float)last_angle * 0.0003834952f;  /* 2π/16384 */
}
