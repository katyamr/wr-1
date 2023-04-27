#pragma once

struct tone_sweep_t {

    static const unsigned long max_t = (unsigned long) (-1);
    static const unsigned long half_t = max_t / 2;
    static const unsigned long min_delay = 2;

    void setup() {
        // pinMode(A1, OUTPUT);
        freq = 0;
    }

    void update(unsigned long t) {
        if (freq && (t - next_t) < half_t) {
            if ((t - stop_t) > half_t) {
                freq += freq_step * (t - next_t + min_delay);
                // tone(A1, freq);
                next_t = t + min_delay;
            } else {
                freq = 0;
                noTone(A1);
            }
        }
    }

    void start(unsigned long t, int start_freq, int stop_freq,
        long duration) {
        freq = start_freq;
        freq_step = (stop_freq - start_freq) / duration;
        // tone(A1, freq);
        stop_t = t + duration;
        next_t = t + min_delay;
    }

    void stop() {
        freq = 0;
        noTone(A1);
    }

    static void beep(unsigned int /* f */, unsigned long d) {
        // tone(A1, f);
        delay(d);
        noTone(A1);
    }

    bool active() const { return freq != 0; }

    unsigned int freq;
    int freq_step;
    unsigned long next_t;
    unsigned long stop_t;
};
