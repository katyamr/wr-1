#pragma once

#include <stdint.h>

struct MPU9250 {
    bool setup();
    bool read();

    static bool ready();

    float acc[3];
    float gyro[3];
    uint16_t temp;
    int16_t mag[3];
};
