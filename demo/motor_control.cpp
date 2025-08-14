/*
 * @Author: 
 * @Date: 2025-08-06 15:58:55
 * @LastEditors: 抖音@翼之道男
 */

#include "motor_control.h"
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <cstdio>
#include <termios.h>
#include <time.h>
#include "friction_identify.h"
#include "yzdn_math.h"
#include "lpf.h"
#include "log.h"
#include "tf_2rd.h"
#include "freq_scan.h"
#include "slop.h"

// #define MOTOR_EN_FRIC_IDEN // 启动摩檫力辨识

// 模组的参数
#define MOTOR_CURRENT_BASE (5.4f) // 电机的电流基值=额定电流A
#define MOTOR_ENCODER_SIM  (524288.0f) // 编码为线数
#define MOTOR_ENCODER_SIM_DIV (1.0f/MOTOR_ENCODER_SIM) // 编码器线数的倒数
#define MOTOR_K_RADPS2SIMPS   (MOTOR_ENCODER_SIM * 1.0F / YZDN_MATH_2PI) // 弧度每秒换算为脉冲每秒

#define MOTOR_UNLIMITED_ACCEL (1000.0F)  // 无限加速度参数 rpm/s
#define MOTOR_LIMITED_ACCEL (10.0f)  // 受限的加速度参数 rpm/s

typedef enum
{
  MC_MODE_OFF = 0X00,    // 空闲
  MC_MODE_COMP_FRIC_PT,      // 摩擦力补偿后的电流环模式
  MC_MODE_COMP_GRAVITY_PT,      // 重力补偿后的电流环模式
  MC_MODE_COMP_PT,      // 重力和摩擦力补偿后的电流环模式
  MC_MODE_COMP_PV,      // 重力和摩擦力补偿后的速度环模式
  MC_MODE_FRIC_IDEN,    // 摩檫力辨识
  MC_MODE_COMP_PV_IDEN_SIN, // 重力和摩擦力补偿后的速度环辨识 正弦模式
  MC_MODE_COMP_PV_IDEN_SQUARE, // 重力和摩擦力补偿后的速度环辨识 方波模式
  MC_MODE_COMP_PV_IDEN_SIN_2, // 重力和摩擦力补偿后的速度环辨识 正弦模式 加速度限幅
  MC_MODE_COMP_PV_IDEN_SQUARE_2, // 重力和摩擦力补偿后的速度环辨识 方波模式 加速度限幅
  MC_MODE_NUM,          // 模式的数量
}MOTOR_CTRL_mode_e;

char mc_mode_name[MC_MODE_NUM][45] = {
    "MC_MODE_OFF","MC_MODE_COMP_FRIC_PT",
    "MC_MODE_COMP_GRAVITY_PT", "MC_MODE_COMP_PT","MC_MODE_COMP_PV",
    "MC_MODE_FRIC_IDEN","MC_MODE_COMP_PV_IDEN_SIN","MC_MODE_COMP_PV_IDEN_SQUARE",
    "MC_MODE_COMP_PV_IDEN_SIN_2","MC_MODE_COMP_PV_IDEN_SQUARE_2"};

// 电机控制结构体
typedef struct
{
    float dt;         // 固定控制周期 s
    float dt_2;       // 实际测量的控制周期 s
    float time;       // 时间 s
    struct timespec time_now;
    struct timespec time_pre;
    MOTOR_CTRL_mode_e mode; // 控制模式
    uint16_t  tick;   // 计数
    // 速度控制参数
    float speed_ref;  // 速度目标值 rpm
    float speed_max;  // 最大速度限制 rpm
    // 电机控制参数
    float current_ref; // 电流目标值 mA
    float current_offset; // 电流前馈 mA
    float current_max; // 最大电流限制 mA
    // 加速度限制
    float accel_limit; // 加速限制 rad/s^2 R
    float accel_limit_rpm; // 加速度限制 rpm/s R/W
    // 反馈参数
    float current_fbk; // 电流反馈 mA
    float speed_fbk;   // 速度反馈 rad/s
    float speed_fbk_rpm; // 速度反馈 rpm
    float angle_fbk;   // 角度反馈 rad
    float angle_fbk_deg; // 角度反馈 deg
    // 摩擦和重力补偿
    uint8_t en_fric_comp; // 打开摩檫力补偿 0x01=开启
    uint8_t en_gravity_comp; // 使能重力补偿
    uint8_t en_fric_iden;     // 开启摩檫力补偿 0x01=开启
    float current_fric; // 摩擦力补偿电流 mA
    float current_gravity; // 重力补偿电流 mA
    // 电机参数
    float current_base;    // 电流基值 A
    ST_LPF lpf_current; // 电流低通滤波
    // 电机反馈参数
    txpdo_t fbk_raw;  // 电机反馈参数原始值
    rxpdo_t cmd_raw;  // 电机指令原始值
    // 延时
    float delay;
    // 速度环模型
    TF_2RD_t tf_spd_d; // 离散化速度模型
    TF_2RD_t tf_spd_c; // 连续速度模型
    SLOP_t   slop_spd; // 斜坡模型
}MOTOR_CTRL_t, *MOTOR_CTRL_h;

void MOTOR_CTRL_key(void);


MOTOR_CTRL_t g_motor_ctrl;
FILE *p_file_log = NULL; // 日志文件
FILE *p_data =NULL; // 数据文件

// 电机控制参数初始化
void MOTOR_CTRL_init(void)
{
    MOTOR_CTRL_t *h =&g_motor_ctrl;
    h->current_ref = 0.0f;
    h->current_base = MOTOR_CURRENT_BASE;
    h->speed_ref = 0.0f;
    h->current_max = 5400.0f;
    lpf_init(&h->lpf_current, 0.2f);
    Friction_Identify_init();
    FREQ_SCAN_init();
    SLOP_init(&h->slop_spd, 0.0001f);

    // 控制模式
    h->mode = MC_MODE_OFF;

    // 电机控制模式配置
    h->cmd_raw.controlword = 0x000F;
    h->cmd_raw.torque_slope = 0x00;
    h->cmd_raw.max_torque = h->current_max / h->current_base;
    h->cmd_raw.padding = 0x00;
    h->cmd_raw.speed_limit = MOTOR_ENCODER_SIM; // 最大转速
    h->cmd_raw.speed_limit_2 = MOTOR_ENCODER_SIM;
    h->cmd_raw.accelerate_up = 2000000; // 加速度限制
    h->cmd_raw.accelerate_down = 2000000;

    // 速度环模型
    TF_2RD_discrete_init(&h->tf_spd_d, 0.0121667f,0.0243333f,0.0121667f, 1.0f, -1.7175803f,0.7660727f);

    // 日志文件
    p_file_log = fopen("log.txt", "w");
    if (p_file_log == NULL) 
    {
        ECAT_LOG("can not open log.txt\n");
        return;
    }
    p_data = fopen("data.txt", "w");
    if (p_data == NULL) 
    {
        ECAT_LOG("can not open data.txt\n");
        return;
    }

    uint32_t accel_limit = MOTOR_LIMITED_ACCEL * YZDN_MATH_K_RPM2RADPS * MOTOR_K_RADPS2SIMPS;
    uint32_t accel_unlimit = MOTOR_UNLIMITED_ACCEL * YZDN_MATH_K_RPM2RADPS * MOTOR_K_RADPS2SIMPS;
    ECAT_LOG("accel_limit:%.1f rpm/s, %d cnt/s\n", MOTOR_LIMITED_ACCEL, accel_limit);
    ECAT_LOG("accel_unlimit:%.1f rpm/s, %d cnt/s\n", MOTOR_UNLIMITED_ACCEL, accel_unlimit);

    clock_gettime(CLOCK_MONOTONIC, &h->time_now);
    h->time_pre = h->time_now;
}


// 电机控制函数 dt 控制周期s
void MOTOR_CTRL_step(float dt)
{
    MOTOR_CTRL_h h = &g_motor_ctrl;
    h->dt = dt;
    txpdo_t *h_tx = &h->fbk_raw;
    rxpdo_t *h_rx = &h->cmd_raw;
    h->tick ++;

    switch(h->mode)
    {
        // 空闲
        case MC_MODE_OFF:
        {
            h->cmd_raw.mode_of_operation = 0x04; // 控制模式 0X03=PV, 0X04=PT
            h->speed_ref = 0.0f;
            h->current_ref = 0.0f;
            h->current_offset = 0.0f;
            h->en_fric_iden = 0x00;
            h->en_fric_comp = 0x00;
            h->en_gravity_comp = 0x00;
            break;
        }


        // 摩檫力辨识
        case MC_MODE_FRIC_IDEN: // 摩檫力辨识
        {
            h->cmd_raw.mode_of_operation = 0x03; // 控制模式 0X03=PV, 0X04=PT
            h->en_fric_iden = 0x01;
            h->en_fric_comp = 0x00;
            h->en_gravity_comp = 0x00;
            Friction_Identify_step(h->dt, h->speed_fbk_rpm, h->lpf_current.output, 0.0f);
            h->speed_ref = Friction_Identify_get_spd_ref();
            h->accel_limit_rpm = MOTOR_UNLIMITED_ACCEL;
            break;
        }
        // 带有补偿的速度模型辨识
        case MC_MODE_COMP_PV_IDEN_SQUARE:
        {
            h->cmd_raw.mode_of_operation = 0x03; // 控制模式 0X03=PV, 0X04=PT
            h->en_fric_iden = 0x00;
            h->en_fric_comp = 0x01;
            h->en_gravity_comp = 0x01;
            h->accel_limit_rpm = MOTOR_UNLIMITED_ACCEL;
            // 方波速度目标值 用于辨识
            h->delay += h->dt;
            float period = 6.0f;
            if(h->delay > period)
            {
                h->delay = 0.0f;
            }
            if(h->delay > 0.5f * period)
            {
                h->speed_ref = 2.0f;
            }
            else
            {
                h->speed_ref = -2.0f;
            }
            break;
        }
        // 带有补偿的速度模型辨识
        case MC_MODE_COMP_PV_IDEN_SIN:
        {
            h->cmd_raw.mode_of_operation = 0x03; // 控制模式 0X03=PV, 0X04=PT
            h->en_fric_iden = 0x00;
            h->en_fric_comp = 0x01;
            h->en_gravity_comp = 0x01;
            h->accel_limit_rpm = MOTOR_UNLIMITED_ACCEL;
            // 正弦扫频辨识
            FREQ_SCAN_step(dt);
            h->speed_ref = FREQ_SCAN_get_input();
            break;
        }

        // 带有补偿的速度模型辨识
        case MC_MODE_COMP_PV_IDEN_SQUARE_2:
        {
            h->cmd_raw.mode_of_operation = 0x03; // 控制模式 0X03=PV, 0X04=PT
            h->en_fric_iden = 0x00;
            h->en_fric_comp = 0x01;
            h->en_gravity_comp = 0x01;
            h->accel_limit_rpm = MOTOR_LIMITED_ACCEL; // 加速限制 rpm/s
            // 方波速度目标值 用于辨识
            h->delay += h->dt;
            float period = 6.0f;
            if(h->delay > period)
            {
                h->delay = 0.0f;
            }
            if(h->delay > 0.5f * period)
            {
                h->speed_ref = 2.0f;
            }
            else
            {
                h->speed_ref = -2.0f;
            }
            break;
        }
        // 带有补偿的速度模型辨识
        case MC_MODE_COMP_PV_IDEN_SIN_2:
        {
            h->cmd_raw.mode_of_operation = 0x03; // 控制模式 0X03=PV, 0X04=PT
            h->en_fric_iden = 0x00;
            h->en_fric_comp = 0x01;
            h->en_gravity_comp = 0x01;
            h->accel_limit_rpm = MOTOR_LIMITED_ACCEL; // 加速限制 rpm/s
            // 正弦扫频辨识
            FREQ_SCAN_step(dt);
            h->speed_ref = FREQ_SCAN_get_input();
            break;
        }

        // 带有补偿的力矩模式
        case MC_MODE_COMP_PT: 
        {
            h->cmd_raw.mode_of_operation = 0x04; // 控制模式 0X03=PV, 0X04=PT
            h->en_fric_iden = 0x00;
            h->en_fric_comp = 0x01;
            h->en_gravity_comp = 0x01;
            h->accel_limit_rpm = MOTOR_UNLIMITED_ACCEL;
            break;
        }

        // 带有补偿的力矩模式
        case MC_MODE_COMP_FRIC_PT: 
        {
            h->cmd_raw.mode_of_operation = 0x04; // 控制模式 0X03=PV, 0X04=PT
            h->en_fric_iden = 0x00;
            h->en_fric_comp = 0x01;
            h->en_gravity_comp = 0x00;
            h->accel_limit_rpm = MOTOR_UNLIMITED_ACCEL;
            break;
        }

        // 带有补偿的力矩模式
        case MC_MODE_COMP_GRAVITY_PT: 
        {
            h->cmd_raw.mode_of_operation = 0x04; // 控制模式 0X03=PV, 0X04=PT
            h->en_fric_iden = 0x00;
            h->en_fric_comp = 0x00;
            h->en_gravity_comp = 0x01;
            h->accel_limit_rpm = MOTOR_UNLIMITED_ACCEL;
            break;
        }
        // 带有补偿的速度模式
        case MC_MODE_COMP_PV:
        {
            h->cmd_raw.mode_of_operation = 0x03; // 控制模式 0X03=PV, 0X04=PT
            h->en_fric_iden = 0x00;
            h->en_fric_comp = 0x01;
            h->en_gravity_comp = 0x01;
            h->accel_limit_rpm = MOTOR_UNLIMITED_ACCEL;
            break;
        }
        default:
            break;
    }

    // 计算程序运行周期
    clock_gettime(CLOCK_MONOTONIC, &h->time_now);
    h->dt_2 = h->time_now.tv_sec - h->time_pre.tv_sec + (h->time_now.tv_nsec - h->time_pre.tv_nsec) * 1e-9;
    h->time_pre = h->time_now;
    // 速度环模型
    float ref_slop = SLOP_step(&h->slop_spd, h->dt_2, h->accel_limit_rpm, h->speed_ref);
    float tf_out_d = TF_2RD_discrete_step(&h->tf_spd_d, h->dt_2, ref_slop);

    // 键盘控制
    MOTOR_CTRL_key();

    // 反馈数据
    h->time           += h->dt;
    h->angle_fbk      = h_tx->actual_position * MOTOR_ENCODER_SIM_DIV * YZDN_MATH_2PI;
    h->angle_fbk_deg  = h->angle_fbk * YZDN_MATH_K_RAD2DEG;
    h->speed_fbk      = h_tx->actual_velocity * MOTOR_ENCODER_SIM_DIV * YZDN_MATH_2PI;
    h->speed_fbk_rpm  = h->speed_fbk * YZDN_MATH_K_RADPS2RPM;
    h->current_fbk    = h_tx->actual_torque * h->current_base;
    lpf_step(&h->lpf_current, h->current_fbk, h->dt);

    // 设置电机控制参数
    h->accel_limit = h->accel_limit_rpm * YZDN_MATH_K_RPM2RADPS;
    float accel_raw = h->accel_limit_rpm * YZDN_MATH_K_RPM2RADPS * MOTOR_K_RADPS2SIMPS;
    if(accel_raw > 2000000.0f)
    {
        accel_raw = 2000000.0f;
    }
    h_rx->accelerate_up =  accel_raw;
    h_rx->accelerate_down =  accel_raw;
    h_rx->target_torque = h->current_ref / h->current_base; // 电机的电流指令 标幺值 千分比
    h_rx->torque_offset = h->current_offset / h->current_base;
    h_rx->speed_target  = h->speed_ref * YZDN_MATH_K_RPM2RADPS * MOTOR_K_RADPS2SIMPS; // 速度目标值 puls/s

    // 摩檫力补偿
    h->current_fric = Friction_Identify_compensate_step(h->speed_fbk_rpm);
    h->current_gravity = Gravity_compensate_step(h->angle_fbk_deg);
    h->current_offset = h->en_fric_comp * h->current_fric + h->en_gravity_comp * h->current_gravity;
   

    // 打印电机控制和运行参数
    if (h->tick % 500 == 0) // 1 Hz
    {
        ECAT_LOG("mode:%d, spd_ref:%.1f, i_ref:%.1f, i_offset:%.1f, angle:%.1f, spd:%.1f, i:%.1f, i_fric:%.1f, i_gravity:%.1f\n", 
            h->mode,
            h->speed_ref, h->current_ref, h->current_offset, 
            h->angle_fbk_deg, h->speed_fbk_rpm, h->current_fbk, 
            h->current_fric, h->current_gravity);
    }
    // 记录日志 500Hz
    if(0x00 == Friction_Identify_get_finish())
    {
        fprintf(p_data, "%.3f, %.3f, %.3f, %.3f, %.1f, %.1f\n", 
            h->time, h->speed_ref, h->speed_fbk_rpm, tf_out_d, h->current_fbk, h->lpf_current.output);
    }
}

// 退出处理
void MOTOR_CTRL_exit(void)
{
    ECAT_LOG("exit, save file");
    sleep(1);
    fclose(p_data);
    fclose(p_file_log);
}

// 用于非阻塞读取键盘输入的函数
int kbhit(void) 
{
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF) {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
}

// 键盘控制函数
void MOTOR_CTRL_key(void)
{
    if (kbhit()) 
    {
        char ch = getchar();
        MOTOR_CTRL_h h = &g_motor_ctrl;
        KEY_LOG("input cmd:%c\n",ch);
        switch(ch)
        {
            case 'p':
            {
                if(h->mode < (MC_MODE_NUM - 0x01))
                {
                    h->mode = MOTOR_CTRL_mode_e((uint8_t)h->mode + 0x01);
                }
                else
                {
                    h->mode = MC_MODE_OFF;
                }
                KEY_LOG("mc mode:%s\n", mc_mode_name[h->mode])
                break;
            }
            case 'w':
            {
                h->current_ref += 10.0f;
                h->speed_ref += 1.0f;
                break;
            }
            case 's':
            {
                h->current_ref -= 10.0f;
                h->speed_ref -= 1.0f;
                break;
            }
            case 'q':
            {
                exit_app = 0x01;
                break;
            }
            case 'e':
            {
                h->current_offset += 10.0f;
                break;
            }
            case 'd':
            {
                h->current_offset -= 10.0f;
                break;
            }
            case 'b':
            {
                h->current_ref = 0.0f;
                h->current_offset = 0.0f;
                h->speed_ref = 0.0f;
                break;
            }
            // 打开摩檫力补偿
            case 'f':
            {
                h->en_fric_comp = (0x01 - h->en_fric_comp);
                KEY_LOG("en_fric_comp:%d\n", h->en_fric_comp);
                break;
            }
            case 'c':
            {
                h->en_fric_iden = 0x01 - h->en_fric_iden;
                KEY_LOG("en_fric_iden:%d\n", h->en_fric_iden);
                break;
            }
            case 'j':
            {
                float gain = Friction_Identify_get_gain() + 0.01f;
                Friction_Identify_set_gain(gain);
                KEY_LOG("fric gain:%.3f\n", gain);
                break;
            }
            case 'k':
            {
                float gain = Friction_Identify_get_gain() - 0.01f;
                Friction_Identify_set_gain(gain);
                KEY_LOG("fric gain:%.3f\n", gain);
                break;
            }

            case 'n':
            {
                float gain = Friction_Identify_get_gravity_gain() + 0.01f;
                Friction_Identify_set_gravity_gain(gain);
                KEY_LOG("gravity gain:%.3f\n", gain);
                break;
            }
            case 'm':
            {
                float gain = Friction_Identify_get_gravity_gain() - 0.01f;
                Friction_Identify_set_gravity_gain(gain);
                KEY_LOG("gravity gain:%.3f\n", gain);
                break;
            }
            default:
            {
                KEY_LOG("-------------------------------\n");
                KEY_LOG("press key to control motor:\n");
                KEY_LOG("w: ref     + 10, spd_ref   +1,\n");
                KEY_LOG("s: ref     - 10, spd_ref   -1,\n");
                KEY_LOG("e: offset  + 10\n");
                KEY_LOG("d: offset  - 10\n");
                KEY_LOG("j: fric_gain    + 0.01\n");
                KEY_LOG("k: fric_gain    - 0.01\n");
                KEY_LOG("n: gravity_gain + 0.01\n");
                KEY_LOG("m: gravity_gain - 0.01\n");
                KEY_LOG("b: stop\n");
                KEY_LOG("q: exit\n");
                KEY_LOG("f: en_fric_comp\n");
                KEY_LOG("c: en_fric_c\n");
                KEY_LOG("-------------------------------\n");
                break;
            }
        }
    }
}

// 更新电机反馈参数 fbk
void MOTOR_CTRL_set_fbk_raw(txpdo_t fbk)
{
    MOTOR_CTRL_h h = &g_motor_ctrl;
    h->fbk_raw = fbk;
}

// 读取发送给电机的控制参数
rxpdo_t MOTOR_CTRL_get_cmd(void)
{
    MOTOR_CTRL_h h = &g_motor_ctrl;
    return h->cmd_raw;
}