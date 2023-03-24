#pragma once

#include <stdint.h>

template<uint8_t MAX, typename T>
struct oversample_ring {
    using value_t = T;
    using index_t = uint8_t;

    enum {
        capacity = MAX
    };

    value_t update(value_t v) {
        value_t res = data[pos];
        data[pos] = v;
        pos = (pos + 1) % MAX;

        sum += v - res;

        return res;
    }

    value_t avg() const {
        return sum / MAX;
    }

    index_t pos = 0;
    value_t sum;
    value_t data[MAX];
};
