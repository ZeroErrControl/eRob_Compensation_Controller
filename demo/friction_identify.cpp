/*
 * @Author: 
 * @Date: 2025-08-06 15:58:55
 * @LastEditors: 抖音@翼之道男
 */
#include "friction_identify.h"
#include "log.h"
#include "math.h"

// #define FRIC_MODE_LOAD

// // 摩擦力辨识数据
// #define FRIC_TABLE_LEN (37)

// float spd_x[FRIC_TABLE_LEN] = {0.1f,0.72f,1.45f,2.18f,2.90f,3.63f,4.35f,5.08f,5.80f,6.53f,
//     7.25f,7.98f,8.70f,9.43f,10.15f,10.87f,11.59f,12.33f,13.05f,13.78f,
//     14.50f,15.23f,15.95f,16.68f,17.40f,18.13f,18.85f,19.58f,20.30f,21.03f,
//     21.75f,22.48f,23.23f,23.93f,24.66f,25.38f,26.11f};
// float current_y[FRIC_TABLE_LEN] = {50.0f,160.50f,179.24f,197.73f,207.33f,215.97f,221.32f,226.32f,233.56f,240.11f,
//     244.70f,249.98f,254.18f,258.07f,260.24f,265.76f,265.76f,272.05f,274.25f,278.42f,
//     280.92f,283.45f,286.11f,290.84f,292.40f,295.50f,301.21f,301.63f,300.50f,304.15f,
//     305.28f,303.67f,306.44f,308.27f,313.76f,317.24f,316.09f};

// #define FRIC_TABLE_LEN (41)
// float spd_x[FRIC_TABLE_LEN] = {0.1f, 
//                     0.4f,1.1f,1.8f,2.5f,3.2f,3.9f,4.7f,5.4f,6.1f,6.8f,
//                     7.5f,8.2f,9.0f,9.7f,10.4f,11.1f,11.8f,12.5f,13.2f,14.0f,
//                     14.7f,15.4f,16.1f,16.8f,17.5f,18.2f,19.0f,19.7f,20.4f,21.1f,
//                     21.8f,22.5f,23.2f,24.0f,24.7f,25.4f,26.1f,26.8f,27.5f,28.2f};
// float current_y[FRIC_TABLE_LEN] = {90.0f,
//                     179.5f,208.4f,229.4f,240.5f,249.6f,260.4f,275.0f,279.6f,285.5f,289.1f,
//                     295.9f,298.6f,302.4f,309.4f,315.4f,320.7f,327.5f,329.1f,336.3f,338.9f,
//                     344.1f,348.7f,354.8f,358.4f,361.9f,363.9f,368.0f,371.9f,376.4f,379.4f,
//                     380.5f,382.4f,382.5f,385.1f,384.0f,383.3f,383.8f,384.9f,388.2f,390.6f};

// #define FRIC_TABLE_LEN (40)
// float spd_x[FRIC_TABLE_LEN] = {0.4f,1.1f,1.8f,2.5f,3.2f,3.9f,4.7f,5.4f,6.1f,6.8f,
//                         7.5f,8.2f,9.0f,9.7f,10.4f,11.1f,11.8f,12.5f,13.2f,14.0f,
//                         14.7f,15.4f,16.1f,16.8f,17.5f,18.2f,19.0f,19.7f,20.4f,21.1f,
//                         21.8f,22.5f,23.2f,24.0f,24.7f,25.4f,26.1f,26.8f,27.5f,28.2f};
// float current_y[FRIC_TABLE_LEN] = {188.9f,215.6f,226.1f,239.6f,247.9f,259.7f,270.1f,279.4f,288.6f,296.7f,
//                         302.8f,310.0f,316.0f,321.6f,327.7f,335.1f,339.5f,344.1f,350.0f,355.1f,
//                         360.2f,361.3f,360.6f,364.5f,366.8f,371.1f,373.8f,376.4f,379.0f,378.8f,
//                         380.1f,380.9f,381.0f,382.0f,378.9f,378.9f,374.0f,380.9f,385.5f,389.7f};

#define FRIC_TABLE_LEN (40)
float spd_x[FRIC_TABLE_LEN] = {0.1f,0.8f,1.5f,2.2f,3.0f,3.7f,4.4f,5.1f,5.9f,6.6f,
                            7.3f,8.0f,8.7f,9.5f,10.2f,10.9f,11.6f,12.4f,13.1f,13.8f,
                            14.5f,15.3f,16.0f,16.7f,17.4f,18.1f,18.9f,19.6f,20.3f,21.0f,
                            21.8f,22.5f,23.2f,23.9f,24.6f,25.4f,26.1f,26.8f,27.5f,28.3f};
float current_y[FRIC_TABLE_LEN] = {128.4f,160.4f,174.7f,186.1f,192.4f,202.9f,207.8f,216.8f,223.0f,230.1f,
                            235.3f,239.3f,243.6f,249.8f,254.8f,258.6f,263.0f,268.3f,273.4f,277.3f,
                            281.1f,285.7f,290.6f,294.8f,299.6f,302.6f,305.8f,307.2f,310.9f,313.4f,
                            317.9f,319.6f,322.5f,326.0f,329.6f,330.3f,333.4f,336.2f,337.3f,344.9f};

#define GRAVITY_CURRENT_AM (-520.0F) // 重力补偿的电流幅值 mA m*g


typedef struct{
    float dt; // 运行周期
    uint8_t state; // 状态
    uint8_t start; // 开始测量 0x01
    uint8_t finish; // 测量完成 0x01
    float ref; // 转速目标值 rpm
    float ref_dir; // 方向
    uint8_t ref_idx; // 当前采集第几个点的数据
    float ref_min; // 最小目标值
    float ref_max; // 最大目标值
    uint8_t data_num; // 采集的数据点个数
    float spd; // 模组转速 rpm
    float current; // 电流 mA
    float angle_det; // 双编码器插值 deg
    // 采集平均值和幅值
    uint32_t cnt;
    float spd_avr; // 模组转速 rpm
    float current_avr; // 电流 mA
    float angle_det_avr; // 双编码器插值 deg
    float delay; // 延时 s
    float stable_time; // 稳定时间 s
    float sample_time; // 采样时间 s
    float stop_time; // 从最大转速停止的时间 s
    float current_max; // 最大电流 mA
    float current_min; // 最小电流 mA
    float current_offset; // 电流的偏置 mA
    float current_am; // 电流幅值 mA
    // 摩檫力补偿
    float fric_gain; // 摩檫力补偿增益 
    float fric_current; // 摩檫力补偿电流 mA
    // 重力补偿
    float gravity_gain; // 重力补偿增益
    float gravity_am; // 重力补偿电流幅值 mA
    float gravity_current; // 重力补偿电流 mA
    float comp_current; // 摩檫力和重力的总的补偿电流 mA
}ST_FRIC_IDEN_t, *ST_FRIC_IDEN_h;


ST_FRIC_IDEN_t g_fric_ident;

void Friction_Identify_init(void)
{
    ST_FRIC_IDEN_h h = &g_fric_ident;
    h->data_num = 40u;
    h->start = 0x01;
    h->ref_min = 0.1f;
    h->ref_max = 29.0f;
    h->stop_time = 20.0f;
    h->stable_time = 5.0f;
    h->ref_dir = 1.0f;
    h->fric_gain = 1.03f;
    h->gravity_am = GRAVITY_CURRENT_AM;
    h->gravity_gain = 1.0f;
}

/**
 * 辨识摩擦力模型-采集数据
 * spd_fbk 速度反馈 rpm
 * current_fbk 电流反馈 mA
 * angle_det 双编码器反馈 deg
 * **/
void Friction_Identify_step(float dt, float spd_fbk, float current_fbk, float angle_det)
{
    ST_FRIC_IDEN_h h = &g_fric_ident;
    h->dt = dt;
    h->spd = spd_fbk;
    h->current = current_fbk;
    h->angle_det = angle_det;
    switch(h->state)
    {
        case 0x00: // 空闲
        {
            if(h->start)
            {
                h->state = 10u;
                h->finish = 0x00;
                h->start = 0x00;
            }
            break;
        }

        case 10: // 设置目标转速 等待转速稳定
        {
            h->ref = h->ref_dir * (h->ref_min + h->ref_idx * (h->ref_max - h->ref_min) / (float)h->data_num);

            h->current_max = -1000.0f;
            h->current_min =  1000.0f;

            // 计算转几圈的时间
            float ref_abs = YZDN_MATH_absf(h->ref);
            #ifdef FRIC_MODE_LOAD
            if(ref_abs > 0.1f)
            {
                float spd_radps = ref_abs * YZDN_MATH_K_RPM2RADPS;
                h->sample_time = 3.0f * YZDN_MATH_2PI / spd_radps;
            }
            #else
            h->sample_time = 10.0f;
            #endif
            
            h->delay += h->dt;
            if(h->delay > h->stable_time)
            {
                h->delay = 0.0f;
                if(h->ref_idx < h->data_num)
                {
                    h->ref_idx ++;
                    h->state = 20u;
                }
                else
                {
                    h->ref_idx = 0x00;
                    h->ref = 0.0f;
                    if(h->ref_dir > 0.0f)
                    {
                        h->state = 30u;
                        h->ref_dir = -1.0f;
                    }
                    else
                    {
                        h->state = 0x00;
                        h->finish = 0x01;
                    }
                }
            }
            break;
        }

        case 20: // 采集数据
        {
            
            h->spd_avr += h->spd;
            h->angle_det_avr += h->angle_det;
            h->current_avr +=h->current;
            // 查找电流的最大最小值
            if(h->current > h->current_max)
            {
                h->current_max = h->current;
            }
            if(h->current < h->current_min)
            {
                h->current_min = h->current;
            }
            h->cnt ++;
            h->delay += h->dt;
           
            if(h->delay > h->sample_time)
            {
                h->spd_avr /= h->cnt;
                h->angle_det_avr /= h->cnt;
                h->current_avr /= h->cnt;
                h->cnt = 0x00;
                h->state = 10u;
                float ic_firc = Friction_Identify_compensate_step(h->spd_avr);
                float i_err = ic_firc - h->current_avr;
                h->current_am = 0.5f * (h->current_max - h->current_min);
                h->current_offset = 0.5f * (h->current_max + h->current_min);
                FRIC_LOG("%d, spd:%.3f, i_avr:%.1f, i_max:%.1f, i_min:%.1f, i_offset:%.1f, i_am:%.1f, angel_det:%.1f, ic:%.1f, ic_err:%.1f", 
                            h->ref_idx, h->spd_avr, 
                            h->current_avr, h->current_max, h->current_min, h->current_offset, h->current_am, 
                            h->angle_det_avr, ic_firc, i_err);
                h->spd_avr = 0.0f;
                h->angle_det_avr  = 0.0f;
                h->current_avr = 0.0f;
            }
            break;
        }

        case 30:// 减速到0
        {
            h->delay += h->dt;
            if(h->delay > h->stop_time)
            {
                h->delay = 0.0f;
                h->state = 10u;
            }
        }

        default:
            break;
    }
}

// 摩擦力补偿 spd_fbk 速度反馈 rpm
float Friction_Identify_compensate_step(float spd_fbk)
{
    ST_FRIC_IDEN_h h = &g_fric_ident;
    float dir = 0.0f;
    if(spd_fbk > 0.1f)
    {
        dir = 1.0f;
    }
    else if(spd_fbk < -0.1f)
    {
        dir = -1.0f;
    }
    else
    {
        dir = 0.0f;
    }
    h->fric_current = h->fric_gain * dir * MATH_linear_table_interpolation(spd_x, current_y, FRIC_TABLE_LEN, MATH_absf(spd_fbk));
    return h->fric_current;
}

// 重力补偿 angle deg
float Gravity_compensate_step(float angle)
{
    ST_FRIC_IDEN_h h = &g_fric_ident;
    float angle_rad = angle * YZDN_MATH_K_DEG2RAD;
    h->gravity_current = h->gravity_am * h->gravity_gain * sin(angle_rad);
    return h->gravity_current;
}


// 读取速度目标值
float Friction_Identify_get_spd_ref(void)
{
    ST_FRIC_IDEN_h h = &g_fric_ident;
    return h->ref;
}

float Friction_Identify_get_current_max(void)
{
    ST_FRIC_IDEN_h h = &g_fric_ident;
    return h->current_max;
}

float Friction_Identify_get_current_min(void)
{
    ST_FRIC_IDEN_h h = &g_fric_ident;
    return h->current_min;
}

// 读取辨识完成标志位
uint8_t Friction_Identify_get_finish(void)
{
    ST_FRIC_IDEN_h h = &g_fric_ident;
    return h->finish;
}

// 读取摩檫力增益
float Friction_Identify_get_gain(void)
{
    ST_FRIC_IDEN_h h = &g_fric_ident;
    return h->fric_gain;
}

// 读取摩檫力增益
void Friction_Identify_set_gain(float gain)
{
    ST_FRIC_IDEN_h h = &g_fric_ident;
    h->fric_gain = gain;
}

// 读取重力增益
float Friction_Identify_get_gravity_gain(void)
{
    ST_FRIC_IDEN_h h = &g_fric_ident;
    return h->gravity_gain;
}

void Friction_Identify_set_gravity_gain(float gain)
{
    ST_FRIC_IDEN_h h = &g_fric_ident;
    h->gravity_gain = gain;
}

