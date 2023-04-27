/*
++VVM created by V.Medinskiy 14/03/18 

This library implements motion processing functions of Invensense's MPU-9250.
It is based on their Emedded MotionDriver 6.12 library.
	https://www.invensense.com/developers/software-download

Also Based on log_stm32.c from Invensense motion_driver_6.12
*/

#include "arduino_mpu9250_VB_routines.h"
#include <Arduino.h>
#include "../artl/twi.h"

#define BUF_SIZE        (256)
#define PACKET_LENGTH   (23)

#define PACKET_DEBUG    (1)
#define PACKET_QUAT     (2)
#define PACKET_DATA     (3)

int arduino_get_clock_ms(unsigned long *count)
{
	*count = millis();
	return 0;
}

int arduino_delay_ms(unsigned long num_ms)
{
	delay(num_ms);
	return 0;
}

int arduino_i2c_write(unsigned char slave_addr, unsigned char reg_addr,
                       unsigned char length, unsigned char * data)
{
    artl::twi::buffer()[0] = reg_addr;
    memcpy(artl::twi::buffer() + 1, data, length);

    return artl::twi::write(slave_addr, length + 1, true);
}

int arduino_i2c_write_byte(unsigned char slave_addr, unsigned char reg_addr,
                       unsigned char data)
{
    artl::twi::buffer()[0] = reg_addr;
    artl::twi::buffer()[1] = data;

    return artl::twi::write(slave_addr, 2, true);
}

int arduino_i2c_read(unsigned char slave_addr, unsigned char reg_addr,
                       unsigned char length, unsigned char * data)
{
    uint8_t res = artl::twi::reg_read(slave_addr, reg_addr, length);

    memcpy(data, artl::twi::buffer(), length);

    return res;
}

int arduino_i2c_read_byte(unsigned char slave_addr, unsigned char reg_addr)
{
    artl::twi::reg_read(slave_addr, reg_addr, 1);

    return artl::twi::buffer()[0];
}

//dummy code that should be written

void logString(char * /* string */)
{
}

void eMPL_send_quat(long * /* quat */)
{
}

void eMPL_send_data(unsigned char /* type */, long * /* data */)
{
}
