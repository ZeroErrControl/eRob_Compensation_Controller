/*
 * @Author: 
 * @Date: 2025-08-06 15:58:55
 * @LastEditors: 抖音@翼之道男
 */
#ifndef __LOG_H__
#define __LOG_H__
#include "stdio.h"
#include "motor_control.h"

// #define MAIN_LOG(format, ...) printf("[MAIN]" format"\n", ##__VA_ARGS__)
// #define FRIC_LOG(format, ...) printf("[FRIC]" format"\n", ##__VA_ARGS__)
// #define ECAT_LOG(format, ...) printf("[ECAT]" format, ##__VA_ARGS__)
// #define KEY_LOG(format, ...)  printf("[KEY]" format, ##__VA_ARGS__)

#define MAIN_LOG(format, ...) fprintf(p_file_log, "[MAIN]" format"\n", ##__VA_ARGS__);printf("[MAIN]" format"\n", ##__VA_ARGS__);
#define FRIC_LOG(format, ...) fprintf(p_file_log, "[FRIC]" format"\n", ##__VA_ARGS__);printf("[FRIC]" format"\n", ##__VA_ARGS__)
#define ECAT_LOG(format, ...) fprintf(p_file_log, "[ECAT]" format, ##__VA_ARGS__);    printf("[ECAT]" format, ##__VA_ARGS__);
#define KEY_LOG(format, ...)  fprintf(p_file_log, "[KEY]" format, ##__VA_ARGS__);     printf("[KEY]"  format, ##__VA_ARGS__);
#define FREQ_LOG(format, ...) fprintf(p_file_log, "[FREQ]" format, ##__VA_ARGS__);     printf("[FREQ]"  format, ##__VA_ARGS__);
#endif

