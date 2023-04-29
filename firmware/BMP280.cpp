#include "BMP280.h"
#include "twi.h"
#include "bits/BMP280.h"
#include <math.h>

#define BMP280_ADDRESS  BMP280_ALTERNATIVE_ADDRESS
#define BMP280_CONFIG   (BMP280_TSB_0_5 | BMP280_FILTER_OFF | BMP280_SPI_OFF)
#define BMP280_CONTROL  (BMP280_OVERSAMPLING_T1 | BMP280_OVERSAMPLING_P2 | BMP280_MODE_NORMAL)

namespace {

int32_t compensate_T_int32(int32_t adc_T);
// uint32_t compensate_P_int32(int32_t adc_P);
float compensate_P_float(int32_t adc_P);
bool read_dig();
int16_t *get_dig();
uint8_t chip_id();
bool wait_ready();

int32_t  t_fine;
float    slp;
bool     ready = false;

}

bool BMP280::setup() {
    slp = 1.0;

    if (chip_id() == 0x58 &&
        read_dig() &&
        (TWI_OK == twi::reg_write(BMP280_ADDRESS, BMP280_RA_CONFIG, BMP280_CONFIG)) &&
        (TWI_OK == twi::reg_write(BMP280_ADDRESS, BMP280_RA_CONTROL, BMP280_CONTROL)) &&
        wait_ready() &&
        read())
    {
        reset_slp(0);
        return true;
    }

    return false;
}

bool BMP280::read() {
    uint8_t *b = twi::buffer();

    uint8_t res = twi::reg_read(BMP280_ADDRESS, BMP280_RA_PRESSUREDATA, b, 6);
    if (res != TWI_OK) {
        return false;
    }

    int32_t adc_P =
            ((int32_t) b[0] << 12)
          | ((int32_t) b[1] << 4)
          | (b[2] >> 4);

    int32_t adc_T =
            ((int32_t) b[3] << 12)
          | ((int32_t) b[4] << 4)
          | (b[5] >> 4);

    temp = compensate_T_int32(adc_T) / 100.0;
    pres = compensate_P_float(adc_P);

    alti = 44330 * (1 - pow(pres / slp, 1 / 5.255));

    return true;
}

void BMP280::reset_slp(float altitude) {
    slp = pres / pow(1 - (altitude / 44330), 5.255);
    alti = altitude;
}

int16_t *BMP280::dig() { return get_dig(); }
bool BMP280::ready() { return ::ready; }

namespace {

struct {
    uint16_t T1;
    int16_t  T2;
    int16_t  T3;
    uint16_t P1;
    int16_t  P2;
    int16_t  P3;
    int16_t  P4;
    int16_t  P5;
    int16_t  P6;
    int16_t  P7;
    int16_t  P8;
    int16_t  P9;
} dig;

bool read_dig() {
    uint8_t *b = twi::buffer();

    uint8_t res = twi::reg_read(BMP280_ADDRESS, BMP280_RA_DIG_T1, b, 12 * 2);
    if (res != TWI_OK) {
        return false;
    }

    uint16_t *d = (uint16_t *) &dig;
    for (uint8_t i = 0; i < 12; ++i) {
        d[i] = ((uint16_t) b[2 * i + 1] << 8) | b[2 * i];
    }

    return true;
}

int16_t *get_dig() {
    return (int16_t *) &dig;
}

uint8_t chip_id() {
    return twi::reg_read(BMP280_ADDRESS, BMP280_RA_CHIPID);
}

bool wait_ready() {
    ready = false;

    for (uint8_t i = 0; i < 50; ++i) {
        uint8_t status = twi::reg_read(BMP280_ADDRESS, BMP280_RA_STATUS);
        if ((status & (1 << 3)) == 0) {
            ready = true;
            return true;
        }
    }

    return false;
}

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC. // t_fine carries fine temperature as global value
int32_t compensate_T_int32(int32_t adc_T) {
    const int32_t T1 = dig.T1;
    const int32_t T2 = dig.T2;
    const int32_t T3 = dig.T3;

    int32_t var1, var2;

    var1 = (adc_T >> 3) - (T1 << 1);
    var1 = (var1 * T2) >> 11;

    var2 = (adc_T >> 4) - T1;
    var2 = (var2 * var2) >> 12;
    var2 = (var2 * T3) >> 14;

    t_fine = var1 + var2;

    return (t_fine * 5 + 128) >> 8;
}

#if 0
// Returns pressure in Pa as unsigned 32 bit integer. Output value of “96386” equals 96386 Pa = 963.86 hPa
uint32_t compensate_P_int32(int32_t adc_P) {
    int32_t var1, var2, var3, var4, var5;
    uint32_t p;

    const int32_t P1 = dig.P1;
    const int32_t P2 = dig.P2;
    const int32_t P4 = dig.P4;
    const int32_t P5 = dig.P5;
    const int32_t P6 = dig.P6;
    const int32_t P7 = dig.P7;
    const int32_t P8 = dig.P8;
    const int32_t P9 = dig.P9;

    var1 = (t_fine >> 1) - 64000;

    var2 = var1 >> 2;
    var2 = (var2 * var2) >> 11;
    var2 = var2 * P6 + ((var1 * P5) << 1);
    var2 = (var2 >> 2) + (P4 << 16);

    var3 = var1 >> 2;
    var3 = (var3 * var3) >> 13;
    var3 = (dig.P3 * var3) >> 3;
    var3 = var3 + ((P2 * var1) >> 1);
    var3 = (var3 >> 18) + 32768;
    var3 = (var3 * P1) >> 15;
    if (var3 == 0) {
        return 0;
    }

    p = 1048576 - adc_P - (var2 >> 12);
    p = p * 3125;

    if (p < 0x80000000U) {
        p = (p << 1) / ((uint32_t) var3);
    } else {
        p = (p / (uint32_t) var3) * 2;
    }

    var4 = (int32_t) (p >> 3);
    var4 = (var4 * var4) >> 13;
    var4 = (P9 * var4) >> 12;

    var5 = (P8 * (int32_t) (p >> 2)) >> 13;

    return p + ((var4 + var5 + P7) >> 4);
}
#endif

float compensate_P_float(int32_t adc_P) {
    float var1, var2, p;

    var1 = ((float) t_fine / 2.0) - 64000.0;
    var2 = var1 * var1 * ((float) dig.P6) / 32768.0;
    var2 = var2 + var1 * ((float) dig.P5) * 2.0;
    var2 = (var2 / 4.0) + (((float) dig.P4) * 65536.0);
    var1 = (((float) dig.P3) * var1 * var1 / 524288.0 + ((float) dig.P2) * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * ((float) dig.P1);
    if (var1 == 0.0) {
        return 0; // avoid exception caused by division by zero
    }
    p = 1048576.0 - (float) adc_P;
    p = (p - (var2 / 4096.0)) * 6250.0 / var1;
    var1 = ((float) dig.P9) * (p) * (p) / 2147483648.0;
    var2 = p * ((float) dig.P8) / 32768.0;
    p = p + (var1 + var2 + ((float) dig.P7)) / 16.0;

    return p;
}

}

