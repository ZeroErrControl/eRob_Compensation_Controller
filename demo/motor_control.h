/*
 * @Author: 
 * @Date: 2025-08-06 15:58:55
 * @LastEditors: 抖音@翼之道男
 */

#ifndef __MOTOR_CONTROL_H__
#define __MOTOR_CONTROL_H__

#include "eRob.h"
#include <cstdio>

void MOTOR_CTRL_init(void);
void MOTOR_CTRL_step(float dt);
void MOTOR_CTRL_exit(void);
void MOTOR_CTRL_set_fbk_raw(txpdo_t fbk);

rxpdo_t MOTOR_CTRL_get_cmd(void);

extern FILE *p_file_log; // log file

#endif