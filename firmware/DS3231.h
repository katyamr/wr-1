#pragma once

#include <Wire.h>
#include "buffer.h"

#define CLOCK_ADDRESS 0x68

// Binary Coded Decimal
struct bcd_value {
    void str(char *s) const {
        s[0] = '0' + (v >> 4);
        s[1] = '0' + (v & 0x0F);
    }

    buffer& append_to(buffer& b) const {
        str(b.buf + b.len);
        b.len += 2;
        b.buf[b.len] = 0;
        return b;
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

buffer& operator += (buffer& s, bcd_value b) {
    return b.append_to(s);
}

buffer& operator << (buffer& s, bcd_value b) {
    return b.append_to(s);
}
