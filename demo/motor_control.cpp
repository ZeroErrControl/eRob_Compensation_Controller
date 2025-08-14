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

// #define MOTOR_EN_FRIC_IDEN // start friction identification

// module parameters
#define MOTOR_CURRENT_BASE (5.4f) // motor current base = rated current A
#define MOTOR_ENCODER_SIM  (524288.0f) // encoder lines
#define MOTOR_ENCODER_SIM_DIV (1.0f/MOTOR_ENCODER_SIM) // encoder lines reciprocal
#define MOTOR_K_RADPS2SIMPS   (MOTOR_ENCODER_SIM * 1.0F / YZDN_MATH_2PI) // rad/s to pulse/s

#define MOTOR_UNLIMITED_ACCEL (1000.0F)  // unlimited acceleration parameter rpm/s
#define MOTOR_LIMITED_ACCEL (10.0f)  // limited acceleration parameter rpm/s

typedef enum
{
  MC_MODE_OFF = 0X00,    // idle
  MC_MODE_COMP_FRIC_PT,      // current loop mode after friction compensation
  MC_MODE_COMP_GRAVITY_PT,      // current loop mode after gravity compensation
  MC_MODE_COMP_PT,      // current loop mode after gravity and friction compensation
  MC_MODE_COMP_PV,      // speed loop mode after gravity and friction compensation
  MC_MODE_FRIC_IDEN,    // friction identification
  MC_MODE_COMP_PV_IDEN_SIN, // speed loop identification after gravity and friction compensation sine mode
  MC_MODE_COMP_PV_IDEN_SQUARE, // speed loop identification after gravity and friction compensation square mode
  MC_MODE_COMP_PV_IDEN_SIN_2, // speed loop identification after gravity and friction compensation sine mode with acceleration limit
  MC_MODE_COMP_PV_IDEN_SQUARE_2, // speed loop identification after gravity and friction compensation square mode with acceleration limit
  MC_MODE_NUM,          // number of modes
}MOTOR_CTRL_mode_e;

char mc_mode_name[MC_MODE_NUM][45] = {
    "MC_MODE_OFF","MC_MODE_COMP_FRIC_PT",
    "MC_MODE_COMP_GRAVITY_PT", "MC_MODE_COMP_PT","MC_MODE_COMP_PV",
    "MC_MODE_FRIC_IDEN","MC_MODE_COMP_PV_IDEN_SIN","MC_MODE_COMP_PV_IDEN_SQUARE",
    "MC_MODE_COMP_PV_IDEN_SIN_2","MC_MODE_COMP_PV_IDEN_SQUARE_2"};

    // motor control structure
typedef struct
{
    float dt;         // fixed control period s
    float dt_2;       // actual measured control period s
    float time;       // time s
    struct timespec time_now;
    struct timespec time_pre;
    MOTOR_CTRL_mode_e mode; // control mode
    uint16_t  tick;   // count
    // speed control parameters
    float speed_ref;  // speed target value rpm
    float speed_max;  // maximum speed limit rpm
    // motor control parameters
    float current_ref; // current target value mA
    float current_offset; // current feedforward mA
    float current_max; // maximum current limit mA
    // acceleration limit
    float accel_limit; // acceleration limit rad/s^2 R
    float accel_limit_rpm; // acceleration limit rpm/s R/W
    // 反馈参数
    float current_fbk; // current feedback mA
    float speed_fbk;   // speed feedback rad/s
    float speed_fbk_rpm; // speed feedback rpm
    float angle_fbk;   // angle feedback rad
    float angle_fbk_deg; // angle feedback deg
    // friction and gravity compensation
    uint8_t en_fric_comp; // enable friction compensation 0x01=enable
    uint8_t en_gravity_comp; // enable gravity compensation
    uint8_t en_fric_iden;     // enable friction compensation 0x01=enable
    float current_fric; // friction compensation current mA
    float current_gravity; // gravity compensation current mA
    // 电机参数
    float current_base;    // current base A
    ST_LPF lpf_current; // current low pass filter
    // motor feedback parameters
    txpdo_t fbk_raw;  // motor feedback parameters raw value
    rxpdo_t cmd_raw;  // motor command parameters raw value
    // delay
    float delay;
    // speed loop model
    TF_2RD_t tf_spd_d; // discrete speed model
    TF_2RD_t tf_spd_c; // continuous speed model
    SLOP_t   slop_spd; // slope model
}MOTOR_CTRL_t, *MOTOR_CTRL_h;

void MOTOR_CTRL_key(void);


MOTOR_CTRL_t g_motor_ctrl;
FILE *p_file_log = NULL; // log file
FILE *p_data =NULL; // data file

// motor control parameters initialization
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

    // control mode
    h->mode = MC_MODE_OFF;

    // motor control mode configuration
    h->cmd_raw.controlword = 0x000F;
    h->cmd_raw.torque_slope = 0x00;
    h->cmd_raw.max_torque = h->current_max / h->current_base;
    h->cmd_raw.padding = 0x00;
    h->cmd_raw.speed_limit = MOTOR_ENCODER_SIM; // maximum speed
    h->cmd_raw.speed_limit_2 = MOTOR_ENCODER_SIM;
    h->cmd_raw.accelerate_up = 2000000; // acceleration limit
    h->cmd_raw.accelerate_down = 2000000;

    // speed loop model
    TF_2RD_discrete_init(&h->tf_spd_d, 0.0121667f,0.0243333f,0.0121667f, 1.0f, -1.7175803f,0.7660727f);

    // log file
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


// motor control function dt control period s
void MOTOR_CTRL_step(float dt)
{
    MOTOR_CTRL_h h = &g_motor_ctrl;
    h->dt = dt;
    txpdo_t *h_tx = &h->fbk_raw;
    rxpdo_t *h_rx = &h->cmd_raw;
    h->tick ++;

    switch(h->mode)
    {
        // idle
        case MC_MODE_OFF:
        {
            h->cmd_raw.mode_of_operation = 0x04; // control mode 0X03=PV, 0X04=PT
            h->speed_ref = 0.0f;
            h->current_ref = 0.0f;
            h->current_offset = 0.0f;
            h->en_fric_iden = 0x00;
            h->en_fric_comp = 0x00;
            h->en_gravity_comp = 0x00;
            break;
        }


        // friction identification
        case MC_MODE_FRIC_IDEN: // friction identification
        {
            h->cmd_raw.mode_of_operation = 0x03; // control mode 0X03=PV, 0X04=PT
            h->en_fric_iden = 0x01;
            h->en_fric_comp = 0x00;
            h->en_gravity_comp = 0x00;
            Friction_Identify_step(h->dt, h->speed_fbk_rpm, h->lpf_current.output, 0.0f);
            h->speed_ref = Friction_Identify_get_spd_ref();
            h->accel_limit_rpm = MOTOR_UNLIMITED_ACCEL;
            break;
        }
        // speed loop identification with compensation
        case MC_MODE_COMP_PV_IDEN_SQUARE:
        {
            h->cmd_raw.mode_of_operation = 0x03; // control mode 0X03=PV, 0X04=PT
            h->en_fric_iden = 0x00;
            h->en_fric_comp = 0x01;
            h->en_gravity_comp = 0x01;
            h->accel_limit_rpm = MOTOR_UNLIMITED_ACCEL;
            // square wave speed target value for identification
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
            // speed loop identification with compensation
        case MC_MODE_COMP_PV_IDEN_SIN:
        {
            h->cmd_raw.mode_of_operation = 0x03; // control mode 0X03=PV, 0X04=PT
            h->en_fric_iden = 0x00;
            h->en_fric_comp = 0x01;
            h->en_gravity_comp = 0x01;
            h->accel_limit_rpm = MOTOR_UNLIMITED_ACCEL;
            // sine frequency scan identification
            FREQ_SCAN_step(dt);
            h->speed_ref = FREQ_SCAN_get_input();
            break;
        }

        // speed loop identification with compensation
        case MC_MODE_COMP_PV_IDEN_SQUARE_2:
        {
            h->cmd_raw.mode_of_operation = 0x03; // control mode 0X03=PV, 0X04=PT
            h->en_fric_iden = 0x00;
            h->en_fric_comp = 0x01;
            h->en_gravity_comp = 0x01;
            h->accel_limit_rpm = MOTOR_LIMITED_ACCEL; // acceleration limit rpm/s
            // square wave speed target value for identification
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
        // speed loop identification with compensation
        case MC_MODE_COMP_PV_IDEN_SIN_2:
        {
            h->cmd_raw.mode_of_operation = 0x03; // control mode 0X03=PV, 0X04=PT
            h->en_fric_iden = 0x00;
            h->en_fric_comp = 0x01;
            h->en_gravity_comp = 0x01;
            h->accel_limit_rpm = MOTOR_LIMITED_ACCEL; // acceleration limit rpm/s
            // sine frequency scan identification
            FREQ_SCAN_step(dt);
            h->speed_ref = FREQ_SCAN_get_input();
            break;
        }

        // torque loop with compensation
        case MC_MODE_COMP_PT: 
        {
            h->cmd_raw.mode_of_operation = 0x04; // control mode 0X03=PV, 0X04=PT
            h->en_fric_iden = 0x00;
            h->en_fric_comp = 0x01;
            h->en_gravity_comp = 0x01;
            h->accel_limit_rpm = MOTOR_UNLIMITED_ACCEL;
            break;
        }

        // torque loop with friction compensation
        case MC_MODE_COMP_FRIC_PT: 
        {
            h->cmd_raw.mode_of_operation = 0x04; // control mode 0X03=PV, 0X04=PT
            h->en_fric_iden = 0x00;
            h->en_fric_comp = 0x01;
            h->en_gravity_comp = 0x00;
            h->accel_limit_rpm = MOTOR_UNLIMITED_ACCEL;
            break;
        }

            // torque loop with gravity compensation
        case MC_MODE_COMP_GRAVITY_PT: 
        {
            h->cmd_raw.mode_of_operation = 0x04; // control mode 0X03=PV, 0X04=PT
            h->en_fric_iden = 0x00;
            h->en_fric_comp = 0x00;
            h->en_gravity_comp = 0x01;
            h->accel_limit_rpm = MOTOR_UNLIMITED_ACCEL;
            break;
        }
        // speed loop with compensation
        case MC_MODE_COMP_PV:
        {
            h->cmd_raw.mode_of_operation = 0x03; // control mode 0X03=PV, 0X04=PT
            h->en_fric_iden = 0x00;
            h->en_fric_comp = 0x01;
            h->en_gravity_comp = 0x01;
            h->accel_limit_rpm = MOTOR_UNLIMITED_ACCEL;
            break;
        }
        default:
            break;
    }

    // calculate program running period
    clock_gettime(CLOCK_MONOTONIC, &h->time_now);
    h->dt_2 = h->time_now.tv_sec - h->time_pre.tv_sec + (h->time_now.tv_nsec - h->time_pre.tv_nsec) * 1e-9;
    h->time_pre = h->time_now;
    // speed loop model
    float ref_slop = SLOP_step(&h->slop_spd, h->dt_2, h->accel_limit_rpm, h->speed_ref);
    float tf_out_d = TF_2RD_discrete_step(&h->tf_spd_d, h->dt_2, ref_slop);

    // keyboard control
    MOTOR_CTRL_key();

    // feedback data
    h->time           += h->dt;
    h->angle_fbk      = h_tx->actual_position * MOTOR_ENCODER_SIM_DIV * YZDN_MATH_2PI;
    h->angle_fbk_deg  = h->angle_fbk * YZDN_MATH_K_RAD2DEG;
    h->speed_fbk      = h_tx->actual_velocity * MOTOR_ENCODER_SIM_DIV * YZDN_MATH_2PI;
    h->speed_fbk_rpm  = h->speed_fbk * YZDN_MATH_K_RADPS2RPM;
    h->current_fbk    = h_tx->actual_torque * h->current_base;
    lpf_step(&h->lpf_current, h->current_fbk, h->dt);

    // set motor control parameters
    h->accel_limit = h->accel_limit_rpm * YZDN_MATH_K_RPM2RADPS;
    float accel_raw = h->accel_limit_rpm * YZDN_MATH_K_RPM2RADPS * MOTOR_K_RADPS2SIMPS;
    if(accel_raw > 2000000.0f)
    {
        accel_raw = 2000000.0f;
    }
    h_rx->accelerate_up =  accel_raw;
    h_rx->accelerate_down =  accel_raw;
    h_rx->target_torque = h->current_ref / h->current_base; // motor current command normalized value
    h_rx->torque_offset = h->current_offset / h->current_base;
    h_rx->speed_target  = h->speed_ref * YZDN_MATH_K_RPM2RADPS * MOTOR_K_RADPS2SIMPS; // speed target value puls/s

    // friction compensation
    h->current_fric = Friction_Identify_compensate_step(h->speed_fbk_rpm);
    h->current_gravity = Gravity_compensate_step(h->angle_fbk_deg);
    h->current_offset = h->en_fric_comp * h->current_fric + h->en_gravity_comp * h->current_gravity;
   

    // print motor control and running parameters
    if (h->tick % 500 == 0) // 1 Hz
    {
        ECAT_LOG("mode:%d, spd_ref:%.1f, i_ref:%.1f, i_offset:%.1f, angle:%.1f, spd:%.1f, i:%.1f, i_fric:%.1f, i_gravity:%.1f\n", 
            h->mode,
            h->speed_ref, h->current_ref, h->current_offset, 
            h->angle_fbk_deg, h->speed_fbk_rpm, h->current_fbk, 
            h->current_fric, h->current_gravity);
    }
    // record log 500Hz
    if(0x00 == Friction_Identify_get_finish())
    {
        fprintf(p_data, "%.3f, %.3f, %.3f, %.3f, %.1f, %.1f\n", 
            h->time, h->speed_ref, h->speed_fbk_rpm, tf_out_d, h->current_fbk, h->lpf_current.output);
    }
}

// exit processing
void MOTOR_CTRL_exit(void)
{
    ECAT_LOG("exit, save file");
    sleep(1);
    fclose(p_data);
    fclose(p_file_log);
}

// function to read keyboard input non-blocking
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

    // keyboard control function
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