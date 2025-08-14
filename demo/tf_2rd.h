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
    float dt; // 采样周期 s
    uint8_t en_input;

    // 连续形式
    float k; // 传递函数增益
    float a; // 传递函数分母系数1
    float b; // 传递函数分母系数2
    float dy[2]; // dy = [ddy, dy]
    float y[2]; // y=[dy, y] 

    // 离散化形式
    float num[3]; // G(z)分子系数
    float den[3]; // G(z)分母系数
    float r; // 输入信号 r(k)
    float u; // 输出信号 u(k)
    float r_pre; // r(k-1)
    float r_pre2; // r(k-2)
    float u_pre;  // u(k-1)
    float u_pre2; // u(k-2)
}TF_2RD_t, *TF_2RD_h;

// 离散化形式
void TF_2RD_discrete_init(TF_2RD_h h, float num_0, float num_1, float num_2, float den_0, float den_1, float den_2);
float TF_2RD_discrete_step(TF_2RD_h h, float dt, float r);

// 连续形式
void TF_2RD_continue_init(TF_2RD_h h, float k, float a, float b);
float TF_2RD_continue_step(TF_2RD_h h, float dt, float r);

// 公用
float TF_2RD_get_out(TF_2RD_h h);

#endif



