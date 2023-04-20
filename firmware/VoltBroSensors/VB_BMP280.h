#ifndef _VB_BMP280_H_
#define _VB_BMP280_H_

#if defined(ARDUINO) && (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif
#include "Wire.h"

#define BMP280_DEFAULT_ADDRESS 0x76//Нога SDO на GY91 притянута на 0
#define BMP280_ALTERNATIVE_ADDRESS 0x77 //Если ногу SDO притянуть к 1, по умолчанию на GY91 она притянута на GND

#define BMP280_REGISTER_DIG_T1 0x88
#define BMP280_REGISTER_DIG_T2 0x8A
#define BMP280_REGISTER_DIG_T3 0x8C

#define BMP280_REGISTER_DIG_P1 0x8E
#define BMP280_REGISTER_DIG_P2 0x90
#define BMP280_REGISTER_DIG_P3 0x92
#define BMP280_REGISTER_DIG_P4 0x94
#define BMP280_REGISTER_DIG_P5 0x96
#define BMP280_REGISTER_DIG_P6 0x98
#define BMP280_REGISTER_DIG_P7 0x9A
#define BMP280_REGISTER_DIG_P8 0x9C
#define BMP280_REGISTER_DIG_P9 0x9E

#define BMP280_REGISTER_CHIPID  0xD0
#define BMP280_REGISTER_VERSION 0xD1
#define BMP280_REGISTER_SOFTRESET 0xE0

#define BMP280_REGISTER_STATUS 0xF3
#define BMP280_REGISTER_CONTROL 0xF4
#define BMP280_REGISTER_CONFIG  0xF5
#define BMP280_REGISTER_PRESSUREDATA 0xF7
#define BMP280_REGISTER_TEMPDATA     0xFA

#define BMP280_OVERSAMPLING_T1        0b1
#define BMP280_OVERSAMPLING_T2        0b10
#define BMP280_OVERSAMPLING_T4        0b11
#define BMP280_OVERSAMPLING_T8        0b100
#define BMP280_OVERSAMPLING_T16        0b101

#define BMP280_OVERSAMPLING_P1        0b1
#define BMP280_OVERSAMPLING_P2        0b10
#define BMP280_OVERSAMPLING_P4        0b11
#define BMP280_OVERSAMPLING_P8        0b100
#define BMP280_OVERSAMPLING_P16        0b101

#define BMP280_MODE_SLEEP            0b0
#define BMP280_MODE_FORCED            0b01
#define BMP280_MODE_NORMAL            0b11

#define BMP280_TSB_0_5                0b00      //3.6.3 datasheet
#define BMP280_TSB_62_5                0b1
#define BMP280_TSB_125                0b10
#define BMP280_TSB_250                0b11
#define BMP280_TSB_500                0b100
#define BMP280_TSB_1000                0b101
#define BMP280_TSB_2000                0b110
#define BMP280_TSB_4000                0b111

#define BMP280_FILTER_OFF            0b00      //3.3.3 datasheet
#define BMP280_FILTER_COEFFICIENT2    0b1
#define BMP280_FILTER_COEFFICIENT4    0b10
#define BMP280_FILTER_COEFFICIENT8    0b11
#define BMP280_FILTER_COEFFICIENT16  0b100

#define BMP280_SPI_OFF  0b00
#define BMP280_SPI_ON   0b01

#define BMP280_MEAS   (BMP280_OVERSAMPLING_T2 << 5) | (BMP280_OVERSAMPLING_P16 << 2) | (BMP280_MODE_FORCED) // Настройки для запуска измерений
#define BMP280_CONFIG (BMP280_TSB_250 << 5) | (BMP280_FILTER_COEFFICIENT4 << 2) | (BMP280_SPI_OFF)

using BMP280_S32_t = int32_t;
using BMP280_U32_t = uint32_t;

class VB_BMP280
{
  public:
    bool begin(uint8_t address = BMP280_DEFAULT_ADDRESS,
        uint8_t config = BMP280_CONFIG,
        uint8_t control = BMP280_MEAS);

    bool read();
    void reset_SLP();

    // Output Data
    float temp; // Значение температуры в °C
    float pres; // Значение давления в мм.рт.ст.
    float alti; // Значение высоты над уровнем моря

    BMP280_S32_t temp_i;
    BMP280_U32_t pres_i;

    // Init vars
    float start_altitude = 0;

  private:
    bool test();
    void initialize(uint8_t config); // Инициализация сенсора
    void wait_measuring(); // Задержка по состоянию флага преобразований (3-й бит в регистре 0xF3)    
    int32_t read_temperature(int32_t adc_T);
    uint16_t read16_LE(uint8_t reg);

    BMP280_S32_t compensate_T_int32(BMP280_S32_t adc_T);
    BMP280_U32_t compensate_P_int32(BMP280_S32_t adc_P) const;

    float compensate_P_float(BMP280_S32_t adc_P) const;

    uint8_t dev_addr;
    uint8_t dev_control;

    float SLP; // Расчётное давление на уровне моря в мм.рт.ст.
    uint16_t dig_T1;
    int16_t dig_T2, dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
    int32_t t_fine;

    bool ErrData = false;   // Флаг ошибки чтения данных
};

#endif /* _VB_BMP280_H_ */
