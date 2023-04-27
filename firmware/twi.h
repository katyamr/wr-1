#pragma once

#include <inttypes.h>

enum {
    TWI_OK,
    TWI_ERR_NOT_READY,
    TWI_ERR_SLA_WC,
    TWI_ERR_SLA_NACK,
    TWI_ERR_DATA_NACK,
    TWI_ERR_TIMEOUT,
    TWI_ERR_BUS_ERROR,
    TWI_ERR_ARB_LOST,
    TWI_ERR,
};

struct twi {
    using callback_t = void();

    static void init();
    static void disable();
    static uint8_t *buffer();

    static uint8_t read(uint8_t sla, void *data, uint8_t len, uint8_t stop, callback_t);
    static uint8_t read(uint8_t sla, void *data, uint8_t len, uint8_t stop);
    static uint8_t reg_read(uint8_t sla, uint8_t reg, void *data, uint8_t len);
    static uint8_t reg_read(uint8_t sla, uint8_t reg);

    static uint8_t write(uint8_t sla, void *data, uint8_t len, uint8_t stop, callback_t);
    static uint8_t write(uint8_t sla, void *data, uint8_t len, uint8_t stop);
    static uint8_t reg_write(uint8_t sla, uint8_t reg, uint8_t val);

    static void update();
};
