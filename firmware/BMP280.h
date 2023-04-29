#pragma once

#include <stdint.h>

struct BMP280 {
    bool setup();
    bool read();

    void reset_slp(float altitude);

    static int16_t *dig();
    static bool ready();

    float temp;
    float pres;
    float alti;
};
