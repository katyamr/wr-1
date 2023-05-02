#include "tone_sweep.h"

#include <avr/interrupt.h>
#include "artl/tc.h"
#include "artl/digital_out.h"
#include "artl/yield.h"

namespace {

using tone_tc = artl::tc<3>;
using tone_pin = artl::digital_out<artl::port::F, 6>;

volatile long tc3_limit = 0;
volatile uint32_t freq = 0;

uint32_t f0_x_ktick = 0;
int32_t dfreq = 0;
uint32_t max_ktick = 1;
uint32_t tick = 0;

}

void tone_sweep_t::setup() {
    freq = 0;
    tone_pin::setup();
    tone_tc().setup(0, 0, 4, tone_tc::cs::presc_1);
}

void tone_sweep_t::tone(unsigned int fr0, unsigned int fr1, unsigned long d) {
    // two choices for the 16 bit timers: ck/1 or ck/64
    uint32_t ocr = F_CPU / fr0 / 2 - 1;

    uint8_t prescalarbits = 0b001;
    if (ocr > 0xffff)
    {
      ocr = F_CPU / fr0 / 2 / 64 - 1;
      prescalarbits = 0b011;
    }

    if (prescalarbits != (tone_tc().crb() & 0x07)) {
        tone_tc().crb() = (tone_tc().crb() & 0xF8) | prescalarbits;
    }

    long limit;

    // Calculate the toggle count
    if (d > 0) {
        if (fr1 == 0) {
            fr1 = fr0;
        }

        limit = 2 * fr0 * d / 1000;

        max_ktick = F_CPU / 1000000 * d; // number of ticks x1000
        f0_x_ktick = fr0 * max_ktick;
        dfreq = (int32_t) fr1 - (int32_t) fr0;
        tick = 0;
    } else {
        max_ktick = 0;
        limit = -1;
    }

    freq = fr0;

    tone_tc().ocra() = ocr;
    tone_tc().oca().enable();

    tc3_limit = limit;
}

void tone_sweep_t::no_tone() {
    tone_tc().oca().disable();
    tone_pin::low();
    freq = 0;
}

void tone_sweep_t::wait() {
    while (freq != 0) {
        artl::yield();
    }
}

bool tone_sweep_t::active() {
    return freq != 0;
}

ISR(TIMER3_COMPA_vect)
{
    if (tc3_limit != 0) {
        // toggle the pin
        tone_pin::toggle();

        if (max_ktick > 0) {
            uint16_t old_ocra = tone_tc().ocra();

            tick += old_ocra;
            uint32_t ktick = tick / 1000;

            if (dfreq) {
                freq = (f0_x_ktick + dfreq * ktick) / max_ktick;

                uint32_t ocr = F_CPU / freq / 2 - 1;
                if (ocr != old_ocra) {
                    tone_tc().ocra() = ocr;
                }
            }

            if (ktick >= max_ktick) {
                tc3_limit = 0;
            }
        }
/*
        if (tc3_limit > 0) {
            --tc3_limit;
        }
*/
    } else {
        tone_sweep_t::no_tone();
    }
}

