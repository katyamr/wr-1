#pragma once

struct buffer {

    buffer& append(const char* s, uint8_t size) {
        if (size >= capacity - len) {
            size = capacity - len - 1;
        }
        memcpy(buf + len, s, size);
        buf[len + size] = 0;
        len += size;

        return *this;
    }

    buffer& append(const char* s) {
        return append(s, strlen(s));
    }

    buffer& append(uint32_t n) {
        char b[1 + 3 * sizeof(n)];
        ultoa(n, b, 10);
        return append(b, strlen(b));
    }

    buffer& append(int32_t n) {
        char b[1 + 3 * sizeof(n)];
        ltoa(n, b, 10);
        return append(b);
    }

    buffer& append(char c) {
        if (len + 1 < capacity) {
            buf[len] = c;
            ++len;
            buf[len] = 0;
        }

        return *this;
    }

    buffer& append(int16_t n) {
        char b[1 + 3 * sizeof(n)];
        ltoa(n, b, 10);
        return append(b);
    }

    buffer& append(uint16_t n) {
        char b[1 + 3 * sizeof(n)];
        ultoa(n, b, 10);
        return append(b);
    }

    buffer& append(double d) {
        char b[20];
        const char* s = dtostrf(d, 4, 2, b);
        return append(s);
    }

    buffer& operator<<(char c) { return append(c); }
    buffer& operator<<(const char* s) { return append(s); }
    buffer& operator<<(int32_t n) { return append(n); }
    buffer& operator<<(uint32_t n) { return append(n); }
    buffer& operator<<(int16_t n) { return append(n); }
    buffer& operator<<(uint16_t n) { return append(n); }
    buffer& operator<<(float n) { return append(n); }
    buffer& operator<<(double n) { return append(n); }

    buffer& operator+=(char c) { return append(c); }
    buffer& operator+=(const char* s) { return append(s); }
    buffer& operator+=(int32_t n) { return append(n); }
    buffer& operator+=(uint32_t n) { return append(n); }
    buffer& operator+=(int16_t n) { return append(n); }
    buffer& operator+=(uint16_t n) { return append(n); }
    buffer& operator+=(float n) { return append(n); }
    buffer& operator+=(double n) { return append(n); }

    void remove(uint16_t pos, uint16_t size) {
        uint16_t l = (pos + size >= len) ? 0 : len - (pos + size);
        memmove(buf + pos, buf + pos + size, l);
        len = pos + l;
    }

    void reserve(uint16_t n) {
        len = 0;
        buf = realloc(buf, n);
        capacity = n;
    }

    void ensure_capacity(uint16_t n) {
        if (capacity >= n) return;

        reserve(n);
    }

    void reset() {
        len = 0;
        buf[0] = 0;
    }

    const uint8_t* c_str() const { return buf; }
    uint16_t length() const { return len; }
    bool empty() const { return len == 0; }
    bool full() const { return len + 1 >= capacity; }

    uint8_t* buf = NULL;
    uint16_t len = 0;
    uint16_t capacity = 0;
};
