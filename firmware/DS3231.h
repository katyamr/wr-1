#pragma once

#include <Wire.h>

#define CLOCK_ADDRESS 0x68

// Binary Coded Decimal
struct bcd_value {
    void str(char *s) const {
        s[0] = '0' + (v >> 4);
        s[1] = '0' + (v & 0x0F);
    }

    bcd_value& operator=(uint8_t d) { v = d; return *this; }

    uint8_t v;
};

struct date_time {
    void read() {
        Wire.requestFrom(CLOCK_ADDRESS, 7, 0, 1, true);

        ss = Wire.read() & 0x7F;
        mm = Wire.read();
        hh = Wire.read();
        Wire.read();
        d = Wire.read();
        m = Wire.read();
        y = Wire.read();
    }

    bcd_value year() const { return y; }
    bcd_value month() const { return m; }
    bcd_value day() const { return d; }

    bcd_value hour() const { return hh; }
    bcd_value minute() const { return mm; }
    bcd_value second() const { return ss; }

    bcd_value y, m, d, hh, mm, ss;
};

template<typename T>
T& operator += (T& s, bcd_value b) {
    char str[3];
    b.str(str);
    str[2] = 0;
    return (s += str);
}

template<typename T>
T& operator << (T& s, bcd_value b) {
    char str[3];
    b.str(str);
    str[2] = 0;
    return (s << str);
}
