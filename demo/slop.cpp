/*
 * @Author: 
 * @Date: 2025-08-06 15:58:55
 * @LastEditors: 抖音@翼之道男
 */

#include "slop.h"

void SLOP_init(SLOP_h h, float err_th)
{
    h->err_th = err_th;
}

float SLOP_step(SLOP_h h, float dt, float accel, float ref)
{
    h->dt = dt;
    h->accel = accel;
    h->ref = ref;
    h->err = h->ref - h->out;
    float du = h->dt * h->accel;
    if(YZDN_MATH_absf(h->err) < du) // 步长大于误差就不用积分了
    {
        h->out =  h->ref;
    }
    else
    {
        float u = 0.0f;
        if(h->err > h->err_th)
        {
            u = h->accel;
        }
        else if(h->err < -1.0f * h->err_th)
        {
            u = -1.0f * h->accel;
        }
        h->out += u * h->dt;
    }
    return h->out;
}

void SLOP_reset(SLOP_h h)
{
    h->out = 0.0f;
    h->err = 0.0f;
    h->ref = 0.0f;
}