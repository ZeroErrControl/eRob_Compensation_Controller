/*
 * @Author: 
 * @Date: 2025-08-06 15:58:55
 * @LastEditors: 抖音@翼之道男
 */

#ifndef __INTEGRATOR_H__
#define __INTEGRATOR_H__

typedef struct
{
    float dt;
    float gain; // gain
    float r; // input
    float u; // output
}INTEGRATOR_t, *INTEGRATOR_h;

void INTEGRATOR_init(INTEGRATOR_h h, float gain, float u_0);
float INTEGRATOR_step(INTEGRATOR_h h, float dt, float r);

#endif



