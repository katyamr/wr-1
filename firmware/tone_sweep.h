#pragma once

struct tone_sweep_t {

    static void setup();

    static void start(unsigned int start_freq, unsigned int stop_freq,
        long duration) {
        tone(start_freq, stop_freq, duration);
    }

    static void stop() {
        no_tone();
    }

    static void beep(unsigned int f, unsigned long d) {
        tone(f, f, d);
        wait();
    }

    static void tone(unsigned int fr0, unsigned int fr1 = 0, unsigned long d = 0);
    static void no_tone();
    static void wait();
    static bool active();
};
