#include "lpf.h"
/*
 * @Author: 
 * @Date: 2025-08-06 15:58:55
 * @LastEditors: 抖音@翼之道男
 */

/**
 * @brief 一阶低通滤波-初始化滤波时间常数
 * G(s)=\frac{1}{T*s+1}
 * @param self 
 * @param T 滤波时间常数，数字越大，截止频率越低
 * @return ** void 
 */
void lpf_init(ST_LPF* self, f32 T)
{
    self->T = T;
}

// 带有初始值的初始化
void lpf_init_2(ST_LPF *self, f32 T, f32 y0)
{
    self->T = T;
    self->input = y0;
    self->output = y0;
    self->output_pre = y0;
}

/**
 * @brief 一阶低通滤波函数-主运算，根据输入信号，计算输出信号
 * 
 * @param self 
 * @param input 输入信号
 * @param output 输出信号
 * @return ** void 
 */
f32 lpf_step(ST_LPF* self, f32 input, f32 dt)
{
    self->dt = dt;
    self->input = input;
    self->alpha = self->dt/(self->dt + self->T);
    self->output = self->alpha * self->input +  (1 - self->alpha) * self->output_pre;
    self->output_pre = self->output;
    return self->output;
}

