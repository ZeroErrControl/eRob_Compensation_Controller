/*
 * @Author: 
 * @Date: 2025-08-06 15:58:55
 * @LastEditors: 抖音@翼之道男
 */

#include "yzdn_math.h"

// float absolute value
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

// natural logarithm calculation
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
 * two-point linear interpolation
 * interpolate from (x_0,y_0) to (x_1,y_1), x_0 must be less than x_1
 * x interpolation point
 * return interpolation result
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
 * @brief piecewise linear interpolation function. Given scattered points, output the linear interpolation result of the scattered points according to the input x.
 * 
 * @param[in] pxtable scattered points x list, x must be arranged in ascending order
 * @param[in] pytable scattered points y list
 * @param[in] len number of scattered points
 * @param[in] x x coordinate to be interpolated
 * @return ** f32 interpolation result
 */
f32 MATH_linear_table_interpolation(f32 *pxtable, f32 *pytable, int16_t len, f32 x)
{
    f32 x_0 = pxtable[0];
    f32 y_0 = pytable[0];
    f32 x_1 = pxtable[len-1];
    f32 y_1 = pytable[len-1];
    f32 y = 0;
  
    // linear interpolation
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
	
	// boundary handling
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
  
    // linear interpolation
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
	
	// boundary handling
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
 * @brief limit function. Limit the input x between [-max, +max]
 * 
 * @param[in] max limit value
 * @param[in] x input
 * @return ** f32 output
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
 * @brief sign function
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
 * @brief dead zone mapping
 * continuous mapping without sudden changes
 * when |x| < dead_zone, y=0
 * @param input
 * @param dead_zone dead zone size, must be greater than 0
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
 * @brief linear mapping function with dead zone
 * C0 continuous
 * @param x input
 * @param d dead zone, d > 0
 * @param k slope outside the dead zone
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
 * @brief two-point interpolation function
 * interpolate (x_0,y_0) to (x_1,y_1), output y value at x
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
 * @brief parabolic interpolation
 * vertex coordinates (x_0,y_0), another point on the right side of the parabola (x_1,y_1)
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
    if (x > x_1) // not allowed extrapolation
    {
        y = y_1;
    }
    if (x < x_0)
    {
        y = y_0;
    }
    return y;
}
