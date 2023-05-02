#pragma once

#include "twi.h"
#include "buffer.h"

#define DS3231_ADDRESS 0x68

// Binary Coded Decimal
struct bcd_value {
    void str(char *s) const {
        s[0] = '0' + (v >> 4);
        s[1] = '0' + (v & 0x0F);
    }

    buffer& append_to(buffer& b) const {
        str((char *) b.buf + b.len);
        b.len += 2;
        b.buf[b.len] = 0;
        return b;
    }

    bcd_value& operator=(uint8_t d) { v = d; return *this; }

    explicit operator uint8_t() const { return 10 * (v >> 4) + (v & 0x0F); }

    uint8_t v;
};

struct date_time {
    void read() {
        twi::reg_read(DS3231_ADDRESS, 0, this, sizeof(*this));

        ss.v &= 0x7F;
    }

    bcd_value year() const { return y; }
    bcd_value month() const { return m; }
    bcd_value day() const { return d; }

    bcd_value hour() const { return hh; }
    bcd_value minute() const { return mm; }
    bcd_value second() const { return ss; }

    bcd_value ss, mm, hh, dummy, d, m, y;
};

buffer& operator += (buffer& s, bcd_value b) {
    return b.append_to(s);
}

buffer& operator << (buffer& s, bcd_value b) {
    return b.append_to(s);
}

buffer& operator << (buffer& s, const date_time& d) {
    return s << d.year() << d.month() << d.day() << '-'
        << d.hour() << d.minute() << d.second();
}
