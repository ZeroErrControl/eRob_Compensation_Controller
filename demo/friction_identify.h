/*
 * @Author: 
 * @Date: 2025-08-06 15:58:55
 * @LastEditors: 抖音@翼之道男
 */
#ifndef __FRICTION_IDENTIFY_H__
#define __FRICTION_IDENTIFY_H__

#include "yzdn_math.h"

void Friction_Identify_init(void);
void Friction_Identify_step(float dt, float spd_fbk, float current_fbk, float angle_det);
float Friction_Identify_compensate_step(float spd_fbk);
void Friction_Identify_set_gain(float gain);
float Gravity_compensate_step(float angle);

float Friction_Identify_get_spd_ref(void);
float Friction_Identify_get_current_max(void);
float Friction_Identify_get_current_min(void);
uint8_t Friction_Identify_get_finish(void);
float Friction_Identify_get_gain(void);

float Friction_Identify_get_gravity_gain(void);
void Friction_Identify_set_gravity_gain(float gain);

#endif

