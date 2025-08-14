/*
 * @Author: 
 * @Date: 2025-08-06 15:58:55
 * @LastEditors: 抖音@翼之道男
 */

#ifndef __LPF_H__
#define __LPF_H__
#include "yzdn_math.h"
typedef struct
{
    f32 dt;  // sampling period s
    f32 T; // time constant
    f32 alpha; // 
    f32 input; // input signal
    f32 output; // output signal
    f32 output_pre;  // previous output signal
}ST_LPF;

void lpf_init(ST_LPF* self, f32 T);
void lpf_init_2(ST_LPF *self, f32 T, f32 y0);

f32 lpf_step(ST_LPF* self, f32 input, f32 dt);

#endif

