/*
 * @Author: 
 * @Date: 2025-08-06 15:58:55
 * @LastEditors: 抖音@翼之道男
 */
#include "integrator.h"


// u_0 积分的初始值
void INTEGRATOR_init(INTEGRATOR_h h, float gain, float u_0)
{
    h->u = u_0;
    h->gain = gain;
}

// 运行周期dt 秒， 输入 r
float INTEGRATOR_step(INTEGRATOR_h h, float dt, float r)
{
    h->dt = dt;
    h->r = r;
    h->u += h->r * h->dt * h->gain;
    return h->u;
}

