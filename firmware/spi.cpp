
#include "spi.h"

#include "artl/digital_out.h"
#include "artl/digital_in.h"
#include "artl/yield.h"

namespace {

using spi_ss = artl::digital_out<artl::port::B, 0>;
using spi_sclk = artl::digital_out<artl::port::B, 1>;
using spi_mosi = artl::digital_out<artl::port::B, 2>;
using spi_miso = artl::digital_in<artl::port::B, 3>;

uint8_t *spi_in;
uint8_t *spi_out;
volatile uint16_t spi_wait;

spi::callback_t *spi_isr_cb;

void isr_cb_recv();
void isr_cb_send();
void isr_cb_exchg();

}

void spi::init() {
    spi_ss::high();
    spi_ss::setup();

    // Warning: if the SS pin ever becomes a LOW INPUT then SPI
    // automatically switches to Slave, so the data direction of
    // the SS pin MUST be kept as OUTPUT.
    SPCR =  (1 << MSTR) // master
          | (0 << SPIE) // interrupt
          | (1 << SPE) // enable
          | (0 << DORD) // MSB
          | (0 << CPOL) | (0 << CPHA) // SPI mode 0
          | /* (0 << SPI2X) | */ (0 << SPR1) | (0 << SPR0); // F_CPU / 4

    // Set direction register for SCK and MOSI pin.
    // MISO pin automatically overrides to INPUT.
    // By doing this AFTER enabling SPI, we avoid accidentally
    // clocking in a single bit since the lines go directly
    // from "input" to SPI control.
    // http://code.google.com/p/arduino/issues/detail?id=888
    spi_sclk::setup();
    spi_mosi::setup();
}

inline void spi::wait_spif() {
    while (!(SPSR & _BV(SPIF))) ; // wait
}

uint8_t spi::recv() {
    return transfer(0xFF);
}

// Write to the SPI bus (MOSI pin) and also receive (MISO pin)
inline uint8_t spi::transfer(uint8_t data) {
    send(data);
    return SPDR;
}

inline void spi::send(uint8_t data) {
    SPDR = data;
    /*
     * The following NOP introduces a small delay that can prevent the wait
     * loop form iterating when running at the maximum speed. This gives
     * about 10% more speed, even if it seems counter-intuitive. At lower
     * speeds it is unnoticed.
     */
    asm volatile("nop");
    wait_spif();
}

inline void spi::transfer(void *buf, uint16_t count) {
    if (count == 0) return;

    uint8_t *p = (uint8_t *) buf;
    SPDR = *p;
    while (--count > 0) {
        uint8_t out = *(p + 1);
        wait_spif();
        uint8_t in = SPDR;
        SPDR = out;
        *p++ = in;
    }
    wait_spif();
    *p = SPDR;
}

inline void spi::send(const void *buf, uint16_t count) {
    if (count == 0) return;

    const uint8_t *p = (const uint8_t *) buf;
    while (count != 0) {
        SPDR = *p++;
        wait_spif();
        --count;
    }
}

void spi::send_async(void *buf, uint16_t count) {
    if (count == 0) return;

    spi_isr_cb = isr_cb_send;

    spi_out = (uint8_t *) buf;
    SPDR = *spi_out;
    spi_wait = count;
    SPCR |= (1 << SPIE);
}

void spi::recv_async(void *buf, uint16_t count) {
    if (count == 0) return;

    spi_isr_cb = isr_cb_recv;

    spi_in = (uint8_t *) buf;
    SPDR = 0xFF;
    spi_wait = count;
    SPCR |= (1 << SPIE);
}

void spi::exchg_async(void *buf, uint16_t count) {
    if (count == 0) return;

    spi_isr_cb = isr_cb_exchg;

    spi_out = spi_in = (uint8_t *) buf;
    SPDR = 0xFF;
    spi_wait = count;
    SPCR |= (1 << SPIE);
}

void spi::skip(uint16_t count) {
    while (count-- > 0) {
        SPDR = 0xFF;
        wait_spif();
    }
}

inline uint8_t spi::last_data() {
    return SPDR;
}

void spi::cs_low() {
    spi_ss::low();
}

void spi::cs_high() {
    spi_ss::high();
}

ISR(SPI_STC_vect)
{
    uint8_t in = SPDR;
    if (spi_read) {
        *spi_in++ = in;
    }

    --spi_wait;

    if (spi_wait == 0) {
        SPCR &= ~(1 << SPIE);
        return;
    }

    uint8_t out;
    if (spi_write) {
        out = *spi_out++;
    } else {
        out = 0xFF;
    }

    SPDR = out;
}

namespace {

void isr_cb_recv() {
    *spi_in++ = SPDR;

    if (--spi_wait == 0) {
        SPCR &= ~(1 << SPIE);
        return;
    }

    SPDR = 0xFF;
}

void isr_cb_send() {
    //uint8_t in = SPDR;

    if (--spi_wait == 0) {
        SPCR &= ~(1 << SPIE);
        return;
    }

    SPDR = *spi_out++;
}

void isr_cb_exchg() {
    *spi_in++ = SPDR;

    if (--spi_wait == 0) {
        SPCR &= ~(1 << SPIE);
        return;
    }

    SPDR = *spi_out++;
}

}
