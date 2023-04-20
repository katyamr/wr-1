#include "VB_BMP280.h"
//#include "VoltBroSensors.h"
#include "arduino_mpu9250_VB_routines.h"

uint16_t VB_BMP280::read16_LE(uint8_t reg) {
    uint8_t buf[2];
    arduino_i2c_read(dev_addr, reg, 2, buf);
    return ((uint16_t) buf[1] << 8) | buf[0];
}

bool VB_BMP280::begin(uint8_t address, uint8_t config, uint8_t control) {
    Wire.begin();
    dev_addr = address;
    dev_control = control;

    if (test()) {
        initialize(config);
        return true;
    } else {
        return false;
    }
}

void VB_BMP280::initialize(uint8_t config) {
    // Читаем калибровочные коэффициенты (константы для корректировки показаний температуры и давления) из EEPROM
    uint16_t *v = &dig_T1;
    for (uint8_t reg = BMP280_REGISTER_DIG_T1; reg <= BMP280_REGISTER_DIG_P9; reg += 2, v += 1) {
        *v = read16_LE(reg);
    }
    SLP = 0;
    /*Serial.print("t1 = ");
    Serial.println(dig_T1);
    Serial.print("t2 = ");
    Serial.println(dig_T2);
    Serial.print("t3 = ");
    Serial.println(dig_T3);
    Serial.print("p1 = ");
    Serial.println(dig_P1);
    Serial.print("p2 = ");
    Serial.println(dig_P2);
    Serial.print("p3 = ");
    Serial.println(dig_P3);
    Serial.print("p4 = ");
    Serial.println(dig_P4);
    Serial.print("p5 = ");
    Serial.println(dig_P5);
    Serial.print("p6 = ");
    Serial.println(dig_P6);
    Serial.print("p7 = ");
    Serial.println(dig_P7);
    Serial.print("p8 = ");
    Serial.println(dig_P8);
    Serial.print("p9 = ");
    Serial.println(dig_P9);*/

    arduino_i2c_write_byte(dev_addr, BMP280_REGISTER_CONFIG, config);

    if ((dev_control & BMP280_MODE_NORMAL) == BMP280_MODE_NORMAL) {
        arduino_i2c_write_byte(dev_addr, BMP280_REGISTER_CONTROL, dev_control);
        wait_measuring();
    }

    // инициируем чтение для получения текущего давления
    read();

    reset_SLP();
}

// Проверка соединения с девайсом
bool VB_BMP280::test() {
    return 0x58 == arduino_i2c_read_byte(dev_addr, BMP280_REGISTER_CHIPID);
}

void VB_BMP280::reset_SLP() {
    // расчетное давление на уровне моря в мм.рт.ст.
    SLP = pres / pow(1 - (start_altitude / 44330), 5.255);
}

using BMP280_S32_t = int32_t;
using BMP280_U32_t = uint32_t;

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC. // t_fine carries fine temperature as global value
//BMP280_S32_t t_fine;
BMP280_S32_t VB_BMP280::compensate_T_int32(BMP280_S32_t adc_T)
{
    BMP280_S32_t var1, var2;
    const BMP280_S32_t T1 = dig_T1;
    const BMP280_S32_t T2 = dig_T2;
    const BMP280_S32_t T3 = dig_T3;

    var1 = adc_T >> 3;
    var1 = var1 - (T1 << 1);
    var1 = var1 * T2;
    var1 = var1 >> 11;

    var2 = adc_T >> 4;
    var2 = var2 - T1;
    var2 = var2 * var2;
    var2 = var2 >> 12;
    var2 = var2 * T3;
    var2 = var2 >> 14;

    t_fine = var1 + var2;
    return (t_fine * 5 + 128) >> 8;
}

// Returns pressure in Pa as unsigned 32 bit integer. Output value of “96386” equals 96386 Pa = 963.86 hPa
BMP280_U32_t VB_BMP280::compensate_P_int32(BMP280_S32_t adc_P) const {
    BMP280_S32_t var1, var2, var3, var4, var5;
    BMP280_U32_t p;

    const BMP280_S32_t P1 = dig_P1;
    const BMP280_S32_t P2 = dig_P2;
    const BMP280_S32_t P4 = dig_P4;
    const BMP280_S32_t P5 = dig_P5;
    const BMP280_S32_t P6 = dig_P6;
    const BMP280_S32_t P7 = dig_P7;
    const BMP280_S32_t P8 = dig_P8;
    const BMP280_S32_t P9 = dig_P9;

    var1 = t_fine >> 1;
    var1 = var1 - 64000;

    var2 = var1 >> 2;
    var2 = (var2 * var2) >> 11;
    var2 = var2 * P6;
    var2 = var2 + ((var1 * P5) << 1);
    var2 = var2 >> 2;
    var2 = var2 + (P4 << 16);

    var3 = var1 >> 2;
    var3 = (var3 * var3) >> 13;
    var3 = (dig_P3 * var3) >> 3;
    var3 = var3 + ((P2 * var1) >> 1);
    var3 = var3 >> 18;
    var3 = 32768 + var3;
    var3 = (var3 * P1) >> 15;
    if (var3 == 0) {
        return 0;
    }

    p = 1048576 - adc_P;
    p = p - (var2 >> 12);
    p = p * 3125;

    if (p < 0x80000000) {
        p = (p << 1) / ((BMP280_U32_t) var3);
    } else {
        p = (p / (BMP280_U32_t) var3) * 2;
    }

    var4 = p >> 3;
    var4 = (var4 * var4) >> 13;
    var4 = (P9 * var4) >> 12;

    var5 = ((p >> 2) * P8) >> 13;

    return p + ((var4 + var5 + P7) >> 4);
}

int32_t VB_BMP280::read_temperature(int32_t adc_T)
{
  int32_t var1, var2;

  var1  = ((((adc_T >> 3) - ((int32_t) dig_T1 << 1))) *
       ((int32_t) dig_T2)) >> 11;

  var2  = (((((adc_T >> 4) - ((int32_t) dig_T1)) *
             ((adc_T >> 4) - ((int32_t) dig_T1))) >> 12) *
            ((int32_t) dig_T3)) >> 14;

  t_fine = var1 + var2;

  return (t_fine * 5 + 128) >> 8;
}

float VB_BMP280::compensate_P_float(BMP280_S32_t adc_P) const {
    float var1, var2, p;

    var1 = ((float) t_fine / 2.0) - 64000.0;
    var2 = var1 * var1 * ((float) dig_P6) / 32768.0;
    var2 = var2 + var1 * ((float) dig_P5) * 2.0;
    var2 = (var2 / 4.0) + (((float) dig_P4) * 65536.0);
    var1 = (((float) dig_P3) * var1 * var1 / 524288.0 + ((float) dig_P2) * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * ((float) dig_P1);
    if (var1 == 0.0) {
        return 0; // avoid exception caused by division by zero
    }
    p = 1048576.0 - (float) adc_P;
    p = (p - (var2 / 4096.0)) * 6250.0 / var1;
    var1 = ((float) dig_P9) * (p) * (p) / 2147483648.0;
    var2 = p * ((float) dig_P8) / 32768.0;
    p = p + (var1 + var2 + ((float) dig_P7)) / 16.0;

    return p;
}

bool VB_BMP280::read() {
    int32_t adc_T, adc_P;
    uint8_t buffer[6];

    if ((dev_control & BMP280_MODE_NORMAL) == BMP280_MODE_FORCED) {
        arduino_i2c_write_byte(dev_addr, BMP280_REGISTER_CONTROL, dev_control);
        arduino_i2c_write_byte(dev_addr, BMP280_REGISTER_CONTROL, dev_control);
        wait_measuring();
    }

    arduino_i2c_read(dev_addr, BMP280_REGISTER_PRESSUREDATA, 6, buffer);

    adc_P = ((int32_t) buffer[0] << 12)
          | ((int32_t) buffer[1] << 4)
          | (buffer[2] >> 4);

    adc_T = ((int32_t) buffer[3] << 12)
          | ((int32_t) buffer[4] << 4)
          | (buffer[5] >> 4);

    temp = read_temperature(adc_T) / 100.0;

    temp_i = compensate_T_int32(adc_T);
    //pres_i = compensate_P_int32(adc_P);

    //temp = temp_i / 100.0;
    pres = compensate_P_float(adc_P);

    // высоты
    alti = 44330 * (1 - pow(pres / SLP, 1 / 5.255));
    return true;
}

void VB_BMP280::wait_measuring() {
    int i = 0;
    while (arduino_i2c_read_byte(dev_addr, BMP280_REGISTER_STATUS) & (1 << 3)) {
        if (i >= 50) {
            ErrData = true;
            return;
        }
        delay(1);
        i++;
    }
}
