#include "MPU9250.h"
#include "twi.h"
#include "bits/MPU9250.h"

#define MPU9250_ADDRESS         MPU9250_ADDRESS_AD0_HIGH
#define MPU9250_GYRO_CONFIG     (MPU9250_GYRO_FS_1000 << 3)
#define MPU9250_ACCEL_CONFIG    (MPU9250_ACCEL_FS_8 << 3)

namespace {
    const float gyro_k = 1000.0 / (1 << 15);
    const float accel_k = 9.81 * 8 / (1 << 15);

    bool ready = false;
}

bool MPU9250::setup() {
    twi::reg_write(MPU9250_ADDRESS, MPU9250_RA_CONFIG, 0x03);

    twi::reg_write(MPU9250_ADDRESS, MPU9250_RA_GYRO_CONFIG, MPU9250_GYRO_CONFIG);
    twi::reg_write(MPU9250_ADDRESS, MPU9250_RA_ACCEL_CONFIG, MPU9250_ACCEL_CONFIG);
    twi::reg_write(MPU9250_ADDRESS, MPU9250_RA_FF_THR, 0x06);

    twi::reg_write(MPU9250_ADDRESS, MPU9250_RA_INT_PIN_CFG, 0x02);

    twi::reg_write(MPU9150_MAG_ADDRESS, 0x0A, 0x16);

    ::ready = true;

    return true;
}

bool MPU9250::read() {
    uint8_t *b = twi::buffer();

    twi::reg_read(MPU9250_ADDRESS, MPU9250_RA_ACCEL_XOUT_H, b, 14);
    int16_t v[7];

    for (uint8_t i = 0; i < 7; ++i) {
        v[i] = (b[2 * i] << 8) | b[2 * i + 1];
    }

    for (uint8_t i = 0; i < 3; ++i) {
        acc[i] = (float) v[i] * accel_k;
    }

    temp = v[3];

    for (uint8_t i = 0; i < 3; ++i) {
        gyro[i] = (float) v[i + 4] * gyro_k;
    }

    twi::reg_read(MPU9150_MAG_ADDRESS, MPU9150_RA_MAG_XOUT_L, b, 6);

    for (uint8_t i = 0; i < 3; ++i) {
        mag[i] = (b[2 * i + 1] << 8) | b[2 * i + 1];
    }

    return true;
}

bool MPU9250::ready() { return ::ready; }
