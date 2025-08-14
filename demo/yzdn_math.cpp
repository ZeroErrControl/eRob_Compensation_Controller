/*
 * @Author: 
 * @Date: 2025-08-06 15:58:55
 * @LastEditors: 抖音@翼之道男
 */

#include "yzdn_math.h"

// 浮点绝对值
f32 YZDN_MATH_absf(f32 input)
{
    if(input <0.0f)
    {
        return -input;
    }
    else
    {
        return input;
    }
}

f32 MATH_absf(f32 input)
{
    if(input <0.0f)
    {
        return -input;
    }
    else
    {
        return input;
    }
}

// 自然对数计算
fp64 YZDN_MATH_ln(fp64 a)
{
   int N = 15u;
   int k,nk;
   fp64 x,xx,y;
   x = (a-1)/(a+1);
   xx = x*x;
   nk = 2*N+1;
   y = 1.0/nk;
   for(k=N;k>0;k--)
   {
     nk = nk - 2;
     y = 1.0/nk+xx*y;
     
   }
   return 2.0*x*y;
}

/**
 * 两点线性插值
 * 从(x_0,y_0)插值到(x_1,y_1)， x_0必须小于x_1
 * x 插值的点
 * return 插值结果
 * **/
f32 YZDN_MATH_linear_interpolation(f32 x_0, f32 x_1, f32 y_0, f32 y_1, f32 x)
{
    if(x_0 >= x_1)
    {
        return 0.0f;
    }
    f32 y = 0.0f;
    if(x < x_0)
    {
        y = y_0;
    }
    else if(x > x_1)
    {
        y = y_1;
    }
    else
    {
        f32 k = (y_1 - y_0)/(x_1 - x_0);
        y = k * (x - x_0) + y_0;
    }
    return y;
}

/**
 * @brief 分段线性插值函数。给定散点，根据输入的x输出散点的线性插值结果。
 * 
 * @param[in] pxtable 散点的x列表，x必须是从小到大排列
 * @param[in] pytable 散点的y列表
 * @param[in] len 散点的个数
 * @param[in] x 代插值的x坐标
 * @return ** f32 插值的结果
 */
f32 MATH_linear_table_interpolation(f32 *pxtable, f32 *pytable, int16_t len, f32 x)
{
    f32 x_0 = pxtable[0];
    f32 y_0 = pytable[0];
    f32 x_1 = pxtable[len-1];
    f32 y_1 = pytable[len-1];
    f32 y = 0;
  
    // 线性内插
    for(int i=0; i<(len-2); i++)
    {
        if((x >= pxtable[i]) && (x <= pxtable[i+1]))
        {
            x_0 = pxtable[i];
            y_0 = pytable[i];
            x_1 = pxtable[i+1];
            y_1 = pytable[i+1];
        }
    }
    f32 k = (y_1 - y_0) / (x_1 - x_0);
    y = k * (x - x_0) + y_0;
	
	// 越界处理
    if(x < pxtable[0])
    {
        y = pytable[0];
    }
    if(x > pxtable[len-1])
    {
        y = pytable[len-1];
    }
    return y;
}


f32 YZDN_MATH_linear_table_interpolation(f32 *pxtable, f32 *pytable, int16_t len, f32 x)
{
    f32 x_0 = pxtable[0];
    f32 y_0 = pytable[0];
    f32 x_1 = pxtable[len-1];
    f32 y_1 = pytable[len-1];
    f32 y = 0;
  
    // 线性内插
    for(int i=0; i<(len-2); i++)
    {
        if((x >= pxtable[i]) && (x <= pxtable[i+1]))
        {
            x_0 = pxtable[i];
            y_0 = pytable[i];
            x_1 = pxtable[i+1];
            y_1 = pytable[i+1];
        }
    }
    f32 k = (y_1 - y_0) / (x_1 - x_0);
    y = k * (x - x_0) + y_0;
	
	// 越界处理
    if(x < pxtable[0])
    {
        y = pytable[0];
    }
    if(x > pxtable[len-1])
    {
        y = pytable[len-1];
    }
    return y;
}


/**
 * @brief 限幅函数。将输入的x限制在[-max, +max]之间
 * 
 * @param[in] max 限制值
 * @param[in] x 输入
 * @return ** f32 输出
 */
f32 YZDN_MATH_limit(f32 max, f32 x)
{
    f32 y = 0;
    y = x;
    if(y > max)
    {
        y = max;
    }
    if(y < -max)
    {
        y = -max;
    }
    return y;
}

f32 YZDN_MATH_limit_between(f32 x, f32 min, f32 max)
{
    f32 val = x;
    val = val > max ? max : val;
    val = val < min ? min : val;
    return val;
}

uint32_t YZDN_MATH_limit_between_u32(uint32_t x, uint32_t min, uint32_t max)
{
    uint32_t val = x;
    val = val > max ? max : val;
    val = val < min ? min : val;
    return val;
}

/**
 * @brief 符号函数
 * 
 * @param[in] x 
 * @return ** f32 
 */
f32 YZDN_MATH_sign(f32 x)
{
    f32 sign = 0;
    if(x>0)
    {
        sign = 1;
    }
    if(x<0)
    {
        sign = -1;
    }
    return sign;
}

/**
 * @brief 死区映射
 * 连续没有突变的映射
 * 当 |x| < dead_zone时, y=0
 *
 * @param input
 * @param dead_zone 死区大小 必须大于0
 * @return ** f32
 */
f32 YZDN_MATH_dead_zone(f32 input, f32 dead_zone)
{
    f32 output = input;
    if (input < dead_zone && input > -dead_zone)
    {
        output = 0.0f;
    }
    else if (input > dead_zone)
    {
        output = input - dead_zone;
    }
    else
    {
        output = input + dead_zone;
    }
    return output;
}

/**
 * @brief 带有死区的线性映射函数
 * C0连续
 * @param x 输入
 * @param d 死区 d > 0
 * @param k 死区外的斜率
 * @return ** f32
 */
f32 YZDN_MATH_dead_zone_linear(f32 x, f32 d, f32 k)
{
    f32 y = x;
    if (x > d)
    {
        y = k * (x - d);
    }
    else if (x < -d)
    {
        y = k * (x + d);
    }
    else
    {
        y = 0.0f;
    }
    return y;
}



/**
 * @brief 两点插值函数
 * 插值 (x_0,y_0)到(x_1,y_1)，输出x处的y值
 * @param y_0
 * @param y_1
 * @param x_0
 * @param x_1
 * @param x
 * @return ** f32
 */
f32 YZDN_MATH_2point_interpolation(f32 y_0, f32 y_1, f32 x_0, f32 x_1, f32 x)
{

    f32 y = 0;
    f32 max = y_1 > y_0 ? y_1 : y_0;

    if (x > x_1)
    {
        y = y_1;
    }
    else if (x < x_0)
    {
        y = y_0;
    }
    else
    {
        f32 tmp = x_1 - x_0;
        if (tmp != 0)
        {
            f32 k = (y_1 - y_0) / tmp;
            y = k * (x - x_0) + y_0;
        }
    }
    return y;
}

/**
 * @brief 抛物线插值
 * 顶点坐标 (x_0,y_0), 抛物线上右侧另外一点坐标 (x_1,y_1)
 * @param x_0
 * @param y_0
 * @param x_1
 * @param y_1
 * @param x
 * @return ** f32
 */
f32 YZDN_MATH_parabola_interpolation(f32 x_0, f32 y_0, f32 x_1, f32 y_1, f32 x)
{
    f32 k_1 = 0;

    f32 tmp = (x_1 - x_0) * (x_1 - x_0);
    if (tmp != 0)
    {
        k_1 = (y_1 - y_0) / tmp;
    }

    f32 y = k_1 * (x - x_0) * (x - x_0) + y_0;
    if (x > x_1) // 不允许外插值
    {
        y = y_1;
    }
    if (x < x_0)
    {
        y = y_0;
    }
    return y;
}
