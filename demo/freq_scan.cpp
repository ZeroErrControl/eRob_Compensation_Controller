/*
 * @Author: 
 * @Date: 2025-08-06 15:58:55
 * @LastEditors: 抖音@翼之道男
 */
#include "freq_scan.h"
#include "math.h"
#include "stdio.h"
#include "log.h"

#define FREQ_SCAN_POINT_T (5.0f)  // how many cycles to collect for each point

typedef enum
{
    FREQ_SCAN_STATE_OFF = 0x00, // idle
    FREQ_SCAN_STATE_SCAN,   // scan the current point
    FREQ_SCAN_STATE_NEXT,   // next scan point
    FREQ_SCAN_STATE_FINISH, // scan finished
    FREQ_SCAN_STATE_NUM,
}FREQ_SCAN_state_e;

typedef enum
{
    FREQ_SCAN_MODE_OFF = 0x00,
    FREQ_SCAN_MODE_SINGLE_POINT, // single point mode
    FREQ_SCAN_MODE_CONTINUE,  // continuous mode
}FREQ_SCAN_MODE_e;

typedef struct
{
    float dt; // control period s
    FREQ_SCAN_MODE_e mode; // scan mode
    FREQ_SCAN_state_e state; // scan state machine
    FREQ_SCAN_state_e state_pre;
    uint8_t trigger; // 0x01 = trigger scan
    uint8_t trigger_pre;
    uint8_t finish; // 0x01 = scan finished
    float input;  // scan input signal
    float output; // scan output signal
    float out_am; // target response signal amplitude
    float out_max; // target response signal maximum value
    float out_min; // target response signal minimum value
    float am; // amplitude
    float freq; // frequency Hz
    float angle; // angle rad
    float det_angle; // angle increment rad

    // single point scan parameters
    float x; // scan point 10^x Hz
    float x_0; // start scan point 10^x_0 Hz
    float x_1; // end scan point 10^x_0 Hz
    float dx;  // scan step 10^dx
    float x_time; // scan time for point x s
    
    // continuous scan parameters
    float time; // scan time s
    float time_span; // how long to scan
    float freq_start; // start frequency
    float freq_stop;  // end frequency
    float delay; // delay s
    uint8_t x_finish; // scan finished for this point 0x01=finished
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

// set state machine mode
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


// logarithmic scan method
uint8_t FREQ_SCAN_single_mdoe_step(FREQ_SCAN_h h)
{
    uint8_t ret = 0x0;
    switch(h->state)
    {
        // wait for trigger scan
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

        // single point scan
        case FREQ_SCAN_STATE_SCAN:
        {
            // generate scan sine input signal
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

            // calculate the amplitude of the output response signal
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

        // next scan point
        case FREQ_SCAN_STATE_NEXT:
        {
            if(h->x < h->x_1)
            {
                h->x += h->dx;
                h->freq = powf(10.0f, h->x);
                h->det_angle = YZDN_MATH_2PI * h->freq * h->dt;
                h->x_time = (1.0f / h->freq) * FREQ_SCAN_POINT_T; // collect several cycles for each frequency point
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


    // set the output response signal when scanning
void FREQ_SCAN_set_output(float out)
{
    FREQ_SCAN_h h = &g_freq_can;
    h->output = out;
}

// read the input excitation signal when scanning
float FREQ_SCAN_get_input(void)
{
    FREQ_SCAN_h h = &g_freq_can;
    return h->input;
}

