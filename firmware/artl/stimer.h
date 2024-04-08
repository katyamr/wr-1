
#pragma once

namespace artl {

template<uint8_t N>
struct stimer {
    enum {
        NONE = 0,
    };

    constexpr unsigned long max_delay() const {
        return ((unsigned long) -1) / 2;
    }

    uint8_t update(unsigned long t) {
        uint8_t i = list[NONE].next;

        if (i != NONE && t - list[i].at < max_delay()) {
            remove(i);
            return i;
        }

        return NONE;
    }

    void schedule(uint8_t id, unsigned long at) {
        uint8_t i;

        cancel(id);

        for (i = list[NONE].next; i != NONE; i = list[i].next) {
            if (list[i].at - at < max_delay()) {
                break;
            }
        }

        insert_before(id, i);
        list[id].at = at;
    }

    void cancel(uint8_t id) {
        if (active(id)) {
            remove(id);
        }
    }

    bool active(uint8_t id) const {
        return list[id].next != NONE || list[id].prev != NONE;
    }

    void setup() {
        for (uint8_t i = 0; i < N; ++i) {
            list[i].next = NONE;
            list[i].prev = NONE;
        }
    }

private:

    void insert_before(uint8_t id, uint8_t i) {
        uint8_t prev = list[i].prev;
        list[id].next = i;
        list[id].prev = prev;
        list[i].prev = id;
        list[prev].next = id;
    }

    void remove(uint8_t id) {
        uint8_t next = list[id].next;
        uint8_t prev = list[id].prev;

        list[next].prev = prev;
        list[prev].next = next;

        list[id].next = NONE;
        list[id].prev = NONE;
    }

    struct {
        unsigned long at;
        uint8_t next;
        uint8_t prev;
    } list[N];
};

}
