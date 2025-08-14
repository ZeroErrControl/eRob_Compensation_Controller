
#ifndef __ERBO_H__
#define __ERBO_H__

#include <cstdint>


// Structure for RXPDO (Control data sent to slave)
typedef struct {
    uint16_t controlword;     // 0x6040:0, 16 bits control word
    int16_t target_torque;    // 0x6071:0, 16 bits target torque
    int32_t torque_slope;     // 0x6087:0, 32 bits torque slope, write 0 to disable slope
    uint16_t max_torque;      // 0x6072:0, 16 bits maximum torque
    uint8_t mode_of_operation;// 0x6060:0, 8 bits control mode
    int16_t torque_offset;    // 0x60B2:0, 16 bits torque offset
    int32_t speed_target;     // 0x60FF:0, int32_t speed target value plus/s
    uint32_t speed_limit;     // 0x60FF:0, uint32_t maximum allowed speed plus/s
    uint32_t speed_limit_2;   // 0x6080:0, uint32_t maximum allowed speed 2 plus/s
    uint32_t accelerate_up;   // 0x6083:0, uint32_t contour acceleration plus/s
    uint32_t accelerate_down; // 0x6084:0, uint32_t contour deceleration plus/s
    uint8_t padding;          // 8 bits padding    16bit alignment
} __attribute__((__packed__)) rxpdo_t;

// Structure for TXPDO (Status data received from slave)
typedef struct {
    uint16_t statusword;      // 0x6041:0, 16 bits
    int32_t actual_position;  // 0x6064:0, 32 bits 
    int32_t actual_velocity;  // 0x606C:0, 32 bits
    int16_t actual_torque;    // 0x6077:0, 16 bits
} __attribute__((__packed__)) txpdo_t;

extern uint8_t exit_app;

#endif


