/*
 * @Author: 
 * @Date: 2025-08-06 15:58:55
 * @LastEditors: 抖音@翼之道男
 */
#include "freq_scan.h"
#include "math.h"
#include "stdio.h"
#include "log.h"

#define FREQ_SCAN_POINT_T (5.0f)  // 每个点采集多少周期

typedef enum
{
    FREQ_SCAN_STATE_OFF = 0x00, // 空闲
    FREQ_SCAN_STATE_SCAN,   // 对当前点进行扫频 
    FREQ_SCAN_STATE_NEXT,   // 下一个扫频点
    FREQ_SCAN_STATE_FINISH, // 扫频结束
    FREQ_SCAN_STATE_NUM,
}FREQ_SCAN_state_e;

typedef enum
{
    FREQ_SCAN_MODE_OFF = 0x00,
    FREQ_SCAN_MODE_SINGLE_POINT, // 单点模式
    FREQ_SCAN_MODE_CONTINUE,  // 连续模式
}FREQ_SCAN_MODE_e;

typedef struct
{
    float dt; // 控制周期 s
    FREQ_SCAN_MODE_e mode; // 扫频模式
    FREQ_SCAN_state_e state; // 扫频状态机 
    FREQ_SCAN_state_e state_pre;
    uint8_t trigger; // 0x01 = 触发扫频
    uint8_t trigger_pre;
    uint8_t finish; // 0x01 = 扫频结束
    float input;  // 扫频的输入信号
    float output; // 扫频输出信号
    float out_am; // 目标响应信号的幅值
    float out_max; // 目标响应信号的最大值
    float out_min; // 目标响应信号的最小值
    float am; // 幅值
    float freq; // 频率 Hz
    float angle; // 角度 rad
    float det_angle; // 角度增量 rad

    // 单点扫频参数
    float x; // 扫频点 10^x Hz
    float x_0; // 开始扫频点 10^x_0 Hz
    float x_1; // 结束扫频点 10^x_0 Hz
    float dx;  // 扫频步长 10^dx
    float x_time; // 点x扫频时长 s
    
    // 连续扫频参数
    float time; // 扫频时间 s
    float time_span; // 多久扫频结束
    float freq_start; // 开始频率
    float freq_stop;  // 结束频率
    float delay; // 延时 s
    uint8_t x_finish; // 该点扫频结束 0x01=结束
}FREQ_SCAN_t, *FREQ_SCAN_h;

uint8_t FREQ_SCAN_single_mdoe_step(FREQ_SCAN_h h);
uint8_t FREQ_SCAN_continue_mdoe_step(FREQ_SCAN_h h);

FREQ_SCAN_t g_freq_can;

void FREQ_SCAN_init(void)
{
    FREQ_SCAN_h h = &g_freq_can;
    h->freq_start = powf(10.0f, h->x_0);
    h->freq_stop = powf(10.0f, h->x_1);
    h->out_max = -100.0f;
    h->out_min = 100.0f;
    h->am = 3.0f;
    h->x_0 = -2.0f;
    h->x_1 = 2.0f;
    h->dx = 0.1f;
    h->trigger = 0x01;
    h->time_span = 50.0f;
    h->mode = FREQ_SCAN_MODE_CONTINUE;
}

// 设置状态机模式
void FREQ_SCAN_set_state(FREQ_SCAN_h h, FREQ_SCAN_state_e state)
{
    const char* name[FREQ_SCAN_STATE_NUM] = {"off", "scan", "next", "finish"};
    h->state_pre = h->state;
    h->state = state;
    FREQ_LOG("state:%s -> %s, freq:%.2f, am:%.2f, time:%.2f", 
                name[h->state_pre], name[h->state], h->freq, h->out_am, h->x_time);
}


void FREQ_SCAN_step(float dt)
{
    FREQ_SCAN_h h = &g_freq_can;
    h->dt = dt;
    switch(h->mode)
    {
        case FREQ_SCAN_MODE_OFF:
        {
            break;
        }
        
        case FREQ_SCAN_MODE_SINGLE_POINT:
        {
            if(FREQ_SCAN_single_mdoe_step(h))
            {
                h->mode = FREQ_SCAN_MODE_OFF;
            }
            break;
        }

        case FREQ_SCAN_MODE_CONTINUE:
        {
            if(FREQ_SCAN_continue_mdoe_step(h))
            {
                h->mode = FREQ_SCAN_MODE_OFF;
            }
            break;
        }

        default:
        break;
    }
}

uint8_t FREQ_SCAN_continue_mdoe_step(FREQ_SCAN_h h)
{
    uint8_t ret = 0x00;
    h->x = YZDN_MATH_linear_interpolation(0.0f, h->time_span, h->x_0, h->x_1, h->time);
    h->freq = powf(10.0f, h->x);
    h->det_angle = YZDN_MATH_2PI * h->freq * h->dt;
    h->angle += h->det_angle;
    if(h->angle > YZDN_MATH_2PI)
    {
        h->angle = 0.0f;
    }
    else if(h->angle < 0.0f)
    {
        h->angle = YZDN_MATH_2PI;
    }
    h->input = h->am * sinf(h->angle);
    if(h->time < h->time_span)
    {
        h->time += h->dt;
    }
    else
    {
        ret = 0x01;
        h->freq = 0.0f;
        h->angle = 0.0f;
        h->input = 0.0f;
        h->time = 0.0f;
    }
    return ret;
}


// 对数扫频方法
uint8_t FREQ_SCAN_single_mdoe_step(FREQ_SCAN_h h)
{
    uint8_t ret = 0x0;
    switch(h->state)
    {
        // 等待触发扫频
        case FREQ_SCAN_STATE_OFF:
        {
            if((0x01==h->trigger)&&(0x00 == h->trigger_pre))
            {
                h->x = h->x_0;
                FREQ_SCAN_set_state(h, FREQ_SCAN_STATE_NEXT);
            }
            h->trigger_pre = h->trigger;
            break;
        }

        // 单点扫频
        case FREQ_SCAN_STATE_SCAN:
        {
            // 生成扫频正弦输入信号
            h->angle += h->det_angle;
            if(h->angle > YZDN_MATH_2PI)
            {
                h->angle = 0.0f;
                h->x_finish = 0x01;
            }
            else if(h->angle < 0.0f)
            {
                h->angle = YZDN_MATH_2PI;
                h->x_finish = 0x01;
            }
            h->input = h->am * sinf(h->angle);

            // 计算输出响应信号的幅值
            if(h->out_max < h->output)
            {
                h->out_max = h->output;
            }
            else if(h->out_min > h->output)
            {
                h->out_min = h->output;
            }
            h->out_am = 0.5f * (h->out_max - h->out_min);


            h->delay += h->dt;
            if((h->delay > h->x_time)&&h->x_finish)
            {
                h->delay = 0.0f;
                h->freq = 0.0f;
                h->angle = 0.0f;
                h->input = 0.0f;
                h->out_max = -10000.0f;
                h->out_min = 10000.0f;
                FREQ_SCAN_set_state(h, FREQ_SCAN_STATE_NEXT);
                h->x_finish = 0x00;
                FREQ_LOG("freq:%.2f,am:%.2f", h->freq, h->out_am);
            }
            break;
        }

        // 下一个扫频点
        case FREQ_SCAN_STATE_NEXT:
        {
            if(h->x < h->x_1)
            {
                h->x += h->dx;
                h->freq = powf(10.0f, h->x);
                h->det_angle = YZDN_MATH_2PI * h->freq * h->dt;
                h->x_time = (1.0f / h->freq) * FREQ_SCAN_POINT_T; // 每个频点采集若干周期
                FREQ_SCAN_set_state(h, FREQ_SCAN_STATE_SCAN);
            }
            else
            {
                h->x = h->x_0;
                FREQ_SCAN_set_state(h, FREQ_SCAN_STATE_FINISH);
            }
            break;
        }
        case FREQ_SCAN_STATE_FINISH:
        {
            FREQ_SCAN_set_state(h, FREQ_SCAN_STATE_OFF);
            ret = 0x01;
            break;
        }
        default:
            break;
    }
    return ret;
}


// 设置扫频时候的输出响应信号
void FREQ_SCAN_set_output(float out)
{
    FREQ_SCAN_h h = &g_freq_can;
    h->output = out;
}

// 读取扫频时候的输入激励信号
float FREQ_SCAN_get_input(void)
{
    FREQ_SCAN_h h = &g_freq_can;
    return h->input;
}

