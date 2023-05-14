#pragma once

#include "artl/digital_out.h"
#include "artl/digital_in.h"

#include "crit_sec.h"

template<unsigned long RATE, unsigned long f_cpu>
struct baud_traits { };

template<>
struct baud_traits<115200, 16000000> {
// F_CPU = 16 MHz, BAUD = 115200, UBRRH = 0, UBRRL = 8
    enum { ubrrh = 0x00, ubrrl = 0x08, ucsra = 0, };
};

template<>
struct baud_traits<31250, 16000000> {
// F_CPU = 16 MHz, BAUD = 31250, UBRRH = 0, UBRRL = 31 (0x1F)
    enum { ubrrh = 0x00, ubrrl = 0x1F, ucsra = 0, };
};

template<>
struct baud_traits<19200, 16000000> {
// F_CPU = 16 MHz, BAUD = 19200, UBRRH = 0, UBRRL = 51 (0x33)
    enum { ubrrh = 0x00, ubrrl = 0x33, ucsra = 0, };
};

template<>
struct baud_traits<9600, 16000000> {
// F_CPU = 16 MHz, BAUD = 9600, UBRRH = 0, UBRRL = 103 (0x67)
    enum { ubrrh = 0x00, ubrrl = 0x67, ucsra = 0, };
};

template<
    unsigned long RATE,
    typename BAUD_TRAITS = baud_traits<RATE, F_CPU>>
struct uart_t {
    using rx = artl::digital_in<artl::port::D, 2>;
    using tx = artl::digital_out<artl::port::D, 3>;

    static volatile uint8_t& csra() { return UCSR1A; }
    static volatile uint8_t& csrb() { return UCSR1B; }

    static uint8_t rxc() { return csra() & (1 << RXC1); }
    static uint8_t txc() { return csra() & (1 << TXC1); }
    static uint8_t dre() { return csra() & (1 << UDRE1); }

    static void rxc_int_off() { csrb() &= ~(1 << RXCIE1); }
    static void rxc_int_on() { csrb() |= (1 << RXCIE1); }

    static void txc_int_off() { csrb() &= ~(1 << TXCIE1); }
    static void txc_int_on() { csrb() |= (1 << TXCIE1); }

    static void dre_int_off() { csrb() &= ~(1 << UDRIE1); }
    static void dre_int_on() { csrb() |= (1 << UDRIE1); }

    static void rx_off() { csrb() &= ~(1 << RXEN1); }
    static void rx_on() { csrb() |= (1 << RXEN1); }

    static void tx_off() { csrb() &= ~(1 << TXEN1); }
    static void tx_on() { csrb() |= (1 << TXEN1); }

    static uint8_t data() { return UDR1; }
    static void data(uint8_t d) { UDR1 = d; }

    static void setup() {
        UBRR1H = BAUD_TRAITS::ubrrh;
        UBRR1L = BAUD_TRAITS::ubrrl;
        UCSR1A = BAUD_TRAITS::ucsra;

        UCSR1C = 0x06; // 8N1

        csrb() = (1 << TXEN1);
        // Disable RX for now
        // csrb() = (1 << RXEN1) | (1 << TXEN1) | (1 << RXCIE1);
    }

    static bool write_ready() { return write_size == 0; }

    static void write(const void *d, uint8_t s) {
        write_size = s;
        write_data = (const uint8_t *) d;

        if (write_size > 0 && dre()) {
            data(*write_data);
            --write_size;
            ++write_data;
        }

        if (write_size) {
            crit_sec cs;

            dre_int_on();
        }
    }

    static void on_dre_int() {
        if (write_size) {
            data(*write_data);
            --write_size;
            ++write_data;
        }

        // clear the TXC bit -- "can be cleared by writing a one to its bit
        // location". This makes sure flush() won't return until the bytes
        // actually got written. Other r/w bits are preserved, and zeroes
        // written to the rest.

        csra() = txc();

        if (write_size == 0) {
            dre_int_off();
        }
    }

private:
    static uint8_t write_size;
    static const uint8_t *write_data;
};
