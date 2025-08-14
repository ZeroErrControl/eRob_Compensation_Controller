/*
 * @Author: 
 * @Date: 2025-08-06 15:58:55
 * @LastEditors: 抖音@翼之道男
 */
/*
 * @Description: 有两个延迟的脉冲传递函数实现
 */
#include "tf_2rd.h"

// 离散化传递函数 初始化
// G(z) = (num[0]*z^2 + num[1]*z + num[2]) / (den[0]*z^2 + den[1]*z + den[2]) 
void TF_2RD_discrete_init(TF_2RD_h h, float num_0, float num_1, float num_2, float den_0, float den_1, float den_2)
{
    h->num[0] = num_0;
    h->num[1] = num_1;
    h->num[2] = num_2;
    h->den[0] = den_0;
    h->den[1] = den_1;
    h->den[2] = den_2;
    h->en_input = 0x01;
}

// 离散化传递函数计算
// G(z) = (num[0]*z^2 + num[1]*z + num[2]) / (den[0]*z^2 + den[1]*z + den[2]) 
float TF_2RD_discrete_step(TF_2RD_h h, float dt, float r)
{
    if(h->en_input)
    {
        h->r = r;
    }
    h->u = h->num[0] * h->r + h->num[1]*h->r_pre + h->num[2]*h->r_pre2
    - h->den[1] * h->u_pre - h->den[2] * h->u_pre2;
    h->r_pre2 = h->r_pre;
    h->r_pre = h->r;
    h->u_pre2 = h->u_pre;
    h->u_pre = h->u;
    return h->u;
}

// 连续传递函数初始化
// G(s) = k / (s^2+a*s+b)
void TF_2RD_continue_init(TF_2RD_h h, float k, float a, float b)
{
    h->k = k;
    h->a = a;
    h->b = b;
    h->en_input = 0x01;
}

// 连续传递函数计算
// G(s) = k / (s^2+a*s+b)
float TF_2RD_continue_step(TF_2RD_h h, float dt, float r)
{
    // 计算导数
    float y[2] = {h->y[0], h->y[1]};
    float k[2] = {0x00};
    float k2[2] = {0x00};
    float k3[2] = {0x00};
    float k4[2] = {0x00};
    h->dt = dt;
    const float div = 1.0f / 6.0f;
    if(h->en_input)
    {
        h->r = r;
    }
    // 四阶龙格库塔积分
    k[0] = h->k * h->r - h->a * y[0] - h->b * y[1];
    k[1] = y[0];
    // 2
    y[0] += 0.5f * k[0] * dt;
    y[1] += 0.5f * k[1] * dt;
    k2[0] = h->k * h->r - h->a * y[0] - h->b * y[1];
    k2[1] = y[0];
    // 3
    y[0] += 0.5f * k2[0] * dt;
    y[1] += 0.5f * k2[1] * dt;
    k3[0] = h->k * h->r - h->a * y[0] - h->b * y[1];
    k3[1] = y[0];
    // 4
    y[0] += k3[0] * dt;
    y[1] += k3[1] * dt;
    k4[0] = h->k * h->r - h->a * y[0] - h->b * y[1];
    k4[1] = y[0];
    // 积分
    h->dy[0] = div * (k[0] + 2.0f*k2[0] + 2.0f*k3[0] + k4[0]);
    h->dy[1] = div * (k[1] + 2.0f*k2[1] + 2.0f*k3[1] + k4[1]);
    h->y[0] += h->dy[0] * h->dt;
    h->y[1] += h->dy[1] * h->dt;
    // 最终输出
    h->u = h->y[1];
    return h->u;
}

// 读取输出
float TF_2RD_get_out(TF_2RD_h h)
{
    return h->u;
}


