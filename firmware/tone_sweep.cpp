#define HAVE_TONE_SWEEP 1

#include "tone_sweep.h"

#include <avr/interrupt.h>
#include "artl/tc.h"
#include "artl/digital_out.h"
#include "artl/yield.h"

namespace {

using tone_tc = artl::tc<3>;
using tone_pin = artl::digital_out<artl::port::F, 6>;

volatile uint32_t freq = 0;

#if HAVE_TONE_SWEEP
uint32_t f0_x_ktick = 0;
int32_t dfreq = 0;
int32_t max_ktick = 0;
uint32_t tick = 0;
#endif

}

void tone_sweep_t::setup() {
    freq = 0;

#if HAVE_TONE_SWEEP
    tone_pin::setup();
    tone_tc::setup(0, 0, 4, tone_tc::cs::presc_1);
#endif
}

void tone_sweep_t::tone(unsigned int fr0, unsigned int fr1, unsigned long d) {
#if HAVE_TONE_SWEEP
    uint32_t ocr = F_CPU / 2 / fr0 - 1;

    if (ocr > 0xffff) {
        ocr = 0xffff;
    }

    // Calculate the tick count
    if (d > 0) {
        if (fr1 == 0) {
            fr1 = fr0;
        }

        max_ktick = F_CPU / 1000000 * d; // number of ticks x1000
        f0_x_ktick = fr0 * max_ktick;
        dfreq = (int32_t) fr1 - (int32_t) fr0;

        tick = 0;
    } else {
        max_ktick = -1;
    }

    freq = fr0;

    tone_tc::ocra(ocr);
    tone_tc::oca::enable();
#else
    (void) fr0;
    (void) fr1;
    (void) d;
#endif
}

void tone_sweep_t::no_tone() {
#if HAVE_TONE_SWEEP
    tone_tc::oca::disable();
    tone_pin::low();
    freq = 0;
#endif
}

void tone_sweep_t::wait() {
    while (freq != 0) {
        artl::yield();
    }
}

bool tone_sweep_t::active() {
    return freq != 0;
}

#if HAVE_TONE_SWEEP
ISR(TIMER3_COMPA_vect)
{
    if (max_ktick != 0) {
        // toggle the pin
        tone_pin::toggle();

        if (max_ktick > 0) {
            uint16_t ocr = tone_tc::ocra();

            tick += ocr;
            int32_t ktick = tick / 1000;

            if (dfreq) {
                freq = (f0_x_ktick + dfreq * ktick) / max_ktick;
                tone_tc::ocra(F_CPU / freq / 2 - 1);
            }

            if (ktick >= max_ktick) {
                max_ktick = 0;
            }
        }
    } else {
        tone_sweep_t::no_tone();
    }
}
#endif

