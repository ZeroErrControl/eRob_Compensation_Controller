#include "lpf.h"
/*
 * @Author: 
 * @Date: 2025-08-06 15:58:55
 * @LastEditors: 抖音@翼之道男
 */

/**
 * @brief first order low pass filter - initialize filter time constant
 * G(s)=\frac{1}{T*s+1}
 * @param self 
 * @param T filter time constant, the larger the number, the lower the cutoff frequency
 * @return ** void 
 */
void lpf_init(ST_LPF* self, f32 T)
{
    self->T = T;
}

// initialize with initial value
void lpf_init_2(ST_LPF *self, f32 T, f32 y0)
{
    self->T = T;
    self->input = y0;
    self->output = y0;
    self->output_pre = y0;
}

/**
 * @brief first order low pass filter function - main operation, calculate the output signal according to the input signal
 * 
 * @param self 
 * @param input input signal
 * @param output output signal
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

