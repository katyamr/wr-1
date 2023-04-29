#pragma once

#define BMP280_DEFAULT_ADDRESS  0x76
#define BMP280_ALTERNATIVE_ADDRESS 0x77

#define BMP280_RA_DIG_T1        0x88
#define BMP280_RA_DIG_T2        0x8A
#define BMP280_RA_DIG_T3        0x8C

#define BMP280_RA_DIG_P1        0x8E
#define BMP280_RA_DIG_P2        0x90
#define BMP280_RA_DIG_P3        0x92
#define BMP280_RA_DIG_P4        0x94
#define BMP280_RA_DIG_P5        0x96
#define BMP280_RA_DIG_P6        0x98
#define BMP280_RA_DIG_P7        0x9A
#define BMP280_RA_DIG_P8        0x9C
#define BMP280_RA_DIG_P9        0x9E

#define BMP280_RA_CHIPID        0xD0
#define BMP280_RA_VERSION       0xD1
#define BMP280_RA_SOFTRESET     0xE0

#define BMP280_RA_STATUS        0xF3
#define BMP280_RA_CONTROL       0xF4
#define BMP280_RA_CONFIG        0xF5
#define BMP280_RA_PRESSUREDATA  0xF7
#define BMP280_RA_TEMPDATA      0xFA

#define BMP280_OVERSAMPLING_T1  (0x01 << 5)
#define BMP280_OVERSAMPLING_T2  (0x02 << 5)
#define BMP280_OVERSAMPLING_T4  (0x03 << 5)
#define BMP280_OVERSAMPLING_T8  (0x04 << 5)
#define BMP280_OVERSAMPLING_T16 (0x05 << 5)

#define BMP280_OVERSAMPLING_P1  (0x01 << 2)
#define BMP280_OVERSAMPLING_P2  (0x02 << 2)
#define BMP280_OVERSAMPLING_P4  (0x03 << 2)
#define BMP280_OVERSAMPLING_P8  (0x04 << 2)
#define BMP280_OVERSAMPLING_P16 (0x05 << 2)

#define BMP280_MODE_SLEEP       0x00
#define BMP280_MODE_FORCED      0x01
#define BMP280_MODE_NORMAL      0x03

#define BMP280_TSB_0_5          (0x00 << 5)      // 3.6.3 datasheet
#define BMP280_TSB_62_5         (0x01 << 5)
#define BMP280_TSB_125          (0x02 << 5)
#define BMP280_TSB_250          (0x03 << 5)
#define BMP280_TSB_500          (0x04 << 5)
#define BMP280_TSB_1000         (0x05 << 5)
#define BMP280_TSB_2000         (0x06 << 5)
#define BMP280_TSB_4000         (0x07 << 5)

#define BMP280_FILTER_OFF       (0x00 << 2)      // 3.3.3 datasheet
#define BMP280_FILTER_COEFF2    (0x01 << 2)
#define BMP280_FILTER_COEFF4    (0x02 << 2)
#define BMP280_FILTER_COEFF8    (0x03 << 2)
#define BMP280_FILTER_COEFF16   (0x04 << 2)

#define BMP280_SPI_OFF          0x00
#define BMP280_SPI_ON           0x01
