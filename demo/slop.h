/*
 * @Author: 
 * @Date: 2025-08-06 15:58:55
 * @LastEditors: 抖音@翼之道男
 */

#ifndef __SLOP_H__
#define __SLOP_H__

#include "yzdn_math.h"

typedef struct
{
    float dt; // 控制周期 s
    float accel; // 加速度 1/s
    float ref; // 目标值
    float out; // 输出值
    float err;
    float err_th; // 误差阈值
}SLOP_t, *SLOP_h;

void SLOP_init(SLOP_h h, float err_th);
float SLOP_step(SLOP_h h, float dt, float accel, float ref);
void SLOP_reset(SLOP_h h);


#endif