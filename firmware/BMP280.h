#pragma once

#include <stdint.h>

struct BMP280 {
    uint8_t setup();
    uint8_t read();

    void reset_slp(float altitude);

    float temp;
    float pres;
    float alti;
};
