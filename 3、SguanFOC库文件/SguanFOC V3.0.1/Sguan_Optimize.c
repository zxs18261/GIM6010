/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-02-26 22:37:25
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-04-01 03:45:27
 * @FilePath: \SguanFOC_Debug\SguanFOC\Sguan_Optimize.c
 * @Description: SguanFOC库的“MTPA和FW弱磁控制，三次谐波注入”实现
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_Optimize.h"

// 经典最大转矩电流比 id 计算公式
// [修复] 标准MTPA闭式解(令 Δ=Ld-Lq): id* = [sqrt(λ²+8Δ²·iq²) - λ] / (4Δ)
//   - iq=0 时 id* -> 0（无转矩则无d轴电流）
//   - 凸极 Lq>Ld (Δ<0) 时给出负的 id（弱磁/磁阻利用）
//   原代码 -(λ+sqrt)/(4Δ) 符号/量级错误；且未防 Ld==Lq 除零(表贴机)
void MTPA_Loop(float *Target_id,float flux,float Ld,float Lq,float iq){
    float delta = Ld - Lq;
    if (Value_fabsf(delta) < 1e-9f){   // 表贴电机 Ld≈Lq：MTPA不适用，id保持0
        *Target_id = 0.0f;
        return;
    }
    *Target_id = (Value_sqrtf(flux*flux + 8.0f*delta*delta*iq*iq) - flux) /
                 (4.0f*delta);
}

void MTPA_Loop_q31(Q31_t *Target_id, Q31_t flux, Q31_t Ld, Q31_t Lq, Q31_t iq){
    Q31_t delta;
    Q31_t temp;
    Q31_t sqrt_input;
    int64_t wide;

    delta = IQmath_Q31_sub(Ld, Lq);
    if (delta == 0){            // 表贴电机保护：Ld==Lq 不适用 MTPA
        *Target_id = 0;
        return;
    }

    /* temp = 8*delta^2*iq^2 （Q31域平方后再乘整数8，乘整数必须左移、不能再用Q31乘法） */
    temp = IQmath_Q31_mul(delta, delta);        // delta^2
    temp = IQmath_Q31_mul(temp, iq);            // delta^2*iq
    temp = IQmath_Q31_mul(temp, iq);            // delta^2*iq^2
    wide = (int64_t)temp << 3;                  // *8 (带饱和)
    if (wide > Q31_MAX) wide = Q31_MAX;
    else if (wide < Q31_MIN) wide = Q31_MIN;
    temp = (Q31_t)wide;

    sqrt_input = IQmath_Q31_add(IQmath_Q31_mul(flux, flux), temp);

    /* id* = (sqrt(...) - flux) / (4*delta)   [修复：符号/量级同float版] */
    temp = Value_sqrt_q31(sqrt_input);
    temp = IQmath_Q31_sub(temp, flux);

    wide = (int64_t)delta << 2;                 // 4*delta (带饱和)
    if (wide > Q31_MAX) wide = Q31_MAX;
    else if (wide < Q31_MIN) wide = Q31_MIN;
    delta = (Q31_t)wide;

    *Target_id = IQmath_Q31_div(temp, delta);
}
