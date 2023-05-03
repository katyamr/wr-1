#pragma once

#include <stdint.h>

struct spi {
    using callback_t = void();

    static void init();
    static inline void wait_spif();
    static uint8_t recv();
    static void send(uint8_t data);
    static uint8_t transfer(uint8_t data);
    static void transfer(void *buf, uint16_t count);
    static void skip(uint16_t count);
    static uint8_t last_data();

    static void cs_low();
    static void cs_high();
};
