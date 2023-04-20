#pragma once

struct buffer {
    using size_type = uint16_t;

    buffer& append(const char* s, size_type size) {
        if (size >= capacity - len) {
            size = capacity - len - 1;
        }
        memcpy(buf + len, s, size);
        buf[len + size] = 0;
        len += size;

        return *this;
    }

    buffer& append(const char* s) {
        size_type size = strlen(s);
        memcpy(buf + len, s, size + 1);
        len += size;
        return *this;
    }

    buffer& append(uint32_t n) {
        const char *s = ultoa(n, (char *) buf + len, 10);
        len += strlen(s);
        return *this;
    }

    buffer& append(int32_t n) {
        const char *s = ltoa(n, (char *) buf + len, 10);
        len += strlen(s);
        return *this;
    }

    buffer& append(char c) {
        buf[len] = c;
        ++len;
        buf[len] = 0;

        return *this;
    }

    buffer& append(int16_t n) {
        return append((int32_t) n);
    }

    buffer& append(uint16_t n) {
        return append((uint32_t) n);
    }

    buffer& append(float d) {
        const char* s = dtostrf(d, 4, 2, (char *) buf + len);
        len += strlen(s);
        return *this;
    }

    buffer& assign(const char* s) {
        return reset().append(s);
    }

    buffer& operator<<(char c) { return append(c); }
    buffer& operator<<(const char* s) { return append(s); }
    buffer& operator<<(int32_t n) { return append(n); }
    buffer& operator<<(uint32_t n) { return append(n); }
    buffer& operator<<(int16_t n) { return append(n); }
    buffer& operator<<(uint16_t n) { return append(n); }
    buffer& operator<<(float n) { return append(n); }

    buffer& operator+=(char c) { return append(c); }
    buffer& operator+=(const char* s) { return append(s); }
    buffer& operator+=(int32_t n) { return append(n); }
    buffer& operator+=(uint32_t n) { return append(n); }
    buffer& operator+=(int16_t n) { return append(n); }
    buffer& operator+=(uint16_t n) { return append(n); }
    buffer& operator+=(float n) { return append(n); }

    buffer& operator=(const char* s) { return assign(s); }

    void remove(size_type pos, size_type size) {
        size_type l = (pos + size >= len) ? 0 : len - (pos + size);
        memmove(buf + pos, buf + pos + size, l);
        len = pos + l;
    }

    void reserve(size_type n) {
        len = 0;
        buf = (uint8_t *) realloc(buf, n);
        capacity = n;
    }

    void ensure_capacity(size_type n) {
        if (capacity >= n) return;

        reserve(n);
    }

    buffer& reset() {
        len = 0;
        buf[0] = 0;
        return *this;
    }

    const char* c_str() const { return (char *) buf; }
    const uint8_t* data() const { return buf; }
    size_type length() const { return len; }
    bool empty() const { return len == 0; }
    bool full() const { return len + 1 >= capacity; }

    uint8_t *buf = NULL;
    size_type len = 0;
    size_type capacity = 0;
};
