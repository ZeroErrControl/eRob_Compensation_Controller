/*
 * @Author: 
 * @Date: 2025-08-06 15:58:55
 * @LastEditors: 抖音@翼之道男
 */
#ifndef __TF_2RD_H__
#define __TF_2RD_H__
#include "yzdn_math.h"

typedef struct 
{
    float dt; // sampling period s
    uint8_t en_input;

    // continuous form
    float k; // transfer function gain
    float a; // transfer function denominator coefficient 1
    float b; // transfer function denominator coefficient 2
    float dy[2]; // dy = [ddy, dy]
    float y[2]; // y=[dy, y] 

    // discrete form
    float num[3]; // G(z) numerator coefficients
    float den[3]; // G(z) denominator coefficients
    float r; // input signal r(k)
    float u; // output signal u(k)
    float r_pre; // r(k-1)
    float r_pre2; // r(k-2)
    float u_pre;  // u(k-1)
    float u_pre2; // u(k-2)
}TF_2RD_t, *TF_2RD_h;

// discrete form
void TF_2RD_discrete_init(TF_2RD_h h, float num_0, float num_1, float num_2, float den_0, float den_1, float den_2);
float TF_2RD_discrete_step(TF_2RD_h h, float dt, float r);

// continuous form
void TF_2RD_continue_init(TF_2RD_h h, float k, float a, float b);
float TF_2RD_continue_step(TF_2RD_h h, float dt, float r);

// common
float TF_2RD_get_out(TF_2RD_h h);

#endif



