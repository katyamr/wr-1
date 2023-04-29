
#include "twi.h"
#include "bits/twi.h"
#include "artl/digital_out.h"
#include "artl/yield.h"

enum {
    TWI_READY,
    TWI_MRX,
    TWI_MTX,
};

namespace {

#define TWI_FREQ 100000L

using twi_sda = artl::digital_out<artl::port::D, 1>;
using twi_scl = artl::digital_out<artl::port::D, 0>;

volatile uint8_t twi_state;
volatile uint8_t twi_slarw;
volatile uint8_t twi_sendStop;   // should the transaction end with a stop
volatile uint8_t twi_inRepStart; // in the middle of a repeated start

#define HAVE_TWI_TIMEOUT 0

// twi_timeout_us > 0 prevents the code from getting stuck in various while loops here
// if twi_timeout_us == 0 then timeout checking is disabled (the previous Wire lib behavior)
// at some point in the future, the default twi_timeout_us value could become 25000
// and twi_do_reset_on_timeout could become true
// to conform to the SMBus standard
// http://smbus.org/specs/SMBus_3_1_20180319.pdf

#if HAVE_TWI_TIMEOUT
volatile uint32_t twi_timeout_us = 0ul;
volatile bool twi_timed_out_flag = false;  // a timeout has been seen
volatile bool twi_do_reset_on_timeout = false;  // reset the TWI registers on timeout
#endif

uint8_t twi_defaultBuffer[32];

uint8_t *twi_masterBuffer;
volatile uint8_t twi_masterBufferIndex;
volatile uint8_t twi_masterBufferLength;

volatile uint8_t twi_error;
volatile uint8_t twi_cb_ready;

volatile uint32_t twi_isr = 0;
uint32_t twi_wait = 0;
uint32_t twi_wc = 0;

twi::callback_t *twi_cb;

}

uint8_t twi::last_error;

/*
 * Function twi_init
 * Desc     readys twi pins and sets twi bitrate
 */
void twi::init()
{
    // initialize state
    twi_state = TWI_READY;
    twi_inRepStart = false;
    twi_cb_ready = false;

    // activate internal pullups for twi.
    twi_sda::high();
    twi_scl::high();

    // initialize twi prescaler and bit rate
    TWSR &= _BV(TWPS0) | _BV(TWPS1);
    TWBR = ((F_CPU / TWI_FREQ) - 16) / 2;

    /* twi bit rate formula from atmega128 manual pg 204
    SCL Frequency = CPU Clock Frequency / (16 + (2 * TWBR))
    note: TWBR should be 10 or higher for master mode
    It is 72 for a 16mhz Wiring board with 100kHz TWI */

    // enable twi module, acks, and twi interrupt
    TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA);

    last_error = TWI_OK;
}

/*
 * Function twi_disable
 * Desc     disables twi pins
 */
void twi::disable()
{
    // disable twi module, acks, and twi interrupt
    TWCR &= ~(_BV(TWEN) | _BV(TWIE) | _BV(TWEA));

    // deactivate internal pullups for twi.
    twi_sda::low();
    twi_scl::low();
}

uint8_t *twi::buffer() {
    return twi_defaultBuffer;
}

/*
 * Function twi_read
 * Desc     attempts to become twi bus master and read a
 *          series of bytes from a device on the bus
 * Input    sla: 7bit i2c device address
 *          data: pointer to byte array
 *          len: number of bytes to read into array
 *          sendStop: Boolean indicating whether to send a stop at the end
 * Output   number of bytes read
 */
uint8_t twi::read(uint8_t sla, void *data, uint8_t len, uint8_t sendStop, callback_t cb)
{
    if (TWI_READY != twi_state) {
        last_error = TWI_ERR_NOT_READY;
        return TWI_ERR_NOT_READY;
    }

    twi_cb_ready = true;
    twi_cb = cb;

    twi_state = TWI_MRX;
    twi_sendStop = sendStop;
    // reset error state (0xFF.. no error occurred)
    twi_error = 0xFF;

    // initialize buffer iteration vars
    twi_masterBuffer = (uint8_t *) data;
    twi_masterBufferIndex = 0;
    twi_masterBufferLength = len - 1;  // This is not intuitive, read on...
    // On receive, the previously configured ACK/NACK setting is transmitted in
    // response to the received byte before the interrupt is signalled. 
    // Therefore we must actually set NACK when the _next_ to last byte is
    // received, causing that NACK to be sent in response to receiving the last
    // expected byte of data.

    // build sla+w, slave device address + w bit
    twi_slarw = TW_READ;
    twi_slarw |= sla << 1;

    if (true == twi_inRepStart) {
        // if we're in the repeated start state, then we've already sent the start,
        // (@@@ we hope), and the TWI statemachine is just waiting for the address byte.
        // We need to remove ourselves from the repeated start state before we enable interrupts,
        // since the ISR is ASYNC, and we could get confused if we hit the ISR before cleaning
        // up. Also, don't enable the START interrupt. There may be one pending from the 
        // repeated start that we sent ourselves, and that would really confuse things.
        twi_inRepStart = false; // remember, we're dealing with an ASYNC ISR

        do {
            TWDR = twi_slarw;
#if HAVE_TWI_TIMEOUT
            if  ((twi_timeout_us > 0ul) && ((micros() - startMicros) > twi_timeout_us)) {
                twi_handleTimeout(twi_do_reset_on_timeout);
                last_error = TWI_ERR_TIMEOUT;
                return TWI_ERR_TIMEOUT;
            }
#endif
            ++twi_wc;
        } while (TWCR & _BV(TWWC));

        TWCR = _BV(TWINT) | _BV(TWEA) | _BV(TWEN) | _BV(TWIE); // enable INTs, but not START
    } else {
        // send start condition
        TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT) | _BV(TWSTA);
    }

    last_error = TWI_OK;
    return TWI_OK;
}

namespace {

void dummy_cb() { }

uint8_t twi_err2res() {
    switch (twi_error) {
    case 0xFF: return TWI_OK;
    case TW_BUS_ERROR: return TWI_ERR_BUS_ERROR;
    case TW_MT_SLA_NACK: return TWI_ERR_SLA_NACK;
    case TW_MT_DATA_NACK: return TWI_ERR_DATA_NACK;
    case TW_MT_ARB_LOST: return TWI_ERR_ARB_LOST;
    }

    return TWI_ERR;
}

uint8_t twi_wait_ready() {
    for (uint32_t i = 0; i < 16000; ++i) {
        if (twi_cb_ready && twi_state == TWI_READY) {
            twi_cb_ready = false;
            return (twi::last_error = twi_err2res());
        }

        ++twi_wait;
        artl::yield();
    }

    twi::last_error = TWI_ERR_TIMEOUT;
    return TWI_ERR_TIMEOUT;
}

}

uint8_t twi::read(uint8_t sla, void *data, uint8_t len, uint8_t sendStop)
{
    uint8_t res = read(sla, data, len, sendStop, dummy_cb);
    if (res == TWI_OK) {
        return twi_wait_ready();
    }
    return res;
}

uint8_t twi::reg_read(uint8_t sla, uint8_t reg, void *data, uint8_t len)
{
    twi_defaultBuffer[0] = reg;

    uint8_t res = write(sla, twi_defaultBuffer, 1, false);
    if (res == TWI_OK) {
        res = read(sla, data, len, true);
    }

    return res;
}

uint8_t twi::reg_read(uint8_t sla, uint8_t reg)
{
    twi_defaultBuffer[0] = reg;

    uint8_t res = write(sla, twi_defaultBuffer, 1, false);
    if (res == TWI_OK) {
        res = read(sla, twi_defaultBuffer, 1, true);
    }

    if (res != TWI_OK) {
        return 0xFF;
    }

    return twi_defaultBuffer[0];
}

/*
 * Function twi_write
 * Desc     attempts to become twi bus master and write a
 *          series of bytes to a device on the bus
 * Input    sla: 7bit i2c device address
 *          data: pointer to byte array
 *          length: number of bytes in array
 *          wait: boolean indicating to wait for write or not
 *          sendStop: boolean indicating whether or not to send a stop at the end
 * Output   0 .. success
 *          1 .. length to long for buffer
 *          2 .. address send, NACK received
 *          3 .. data send, NACK received
 *          4 .. other twi error (lost bus arbitration, bus error, ..)
 *          5 .. timeout
 */
uint8_t twi::write(uint8_t sla, void *data, uint8_t len, uint8_t sendStop, callback_t cb)
{
    if (TWI_READY != twi_state) {
        last_error = TWI_ERR_NOT_READY;
        return TWI_ERR_NOT_READY;
    }

    twi_cb_ready = true;
    twi_cb = cb;

    // wait until twi is ready, become master transmitter
    twi_state = TWI_MTX;
    twi_sendStop = sendStop;
    // reset error state (0xFF.. no error occurred)
    twi_error = 0xFF;

    // initialize buffer iteration vars
    twi_masterBuffer = (uint8_t *) data;
    twi_masterBufferIndex = 0;
    twi_masterBufferLength = len;

    // build sla+w, slave device address + w bit
    twi_slarw = TW_WRITE;
    twi_slarw |= sla << 1;

    // if we're in a repeated start, then we've already sent the START
    // in the ISR. Don't do it again.
    //
    if (true == twi_inRepStart) {
        // if we're in the repeated start state, then we've already sent the start,
        // (@@@ we hope), and the TWI statemachine is just waiting for the address byte.
        // We need to remove ourselves from the repeated start state before we enable interrupts,
        // since the ISR is ASYNC, and we could get confused if we hit the ISR before cleaning
        // up. Also, don't enable the START interrupt. There may be one pending from the 
        // repeated start that we sent ourselves, and that would really confuse things.
        twi_inRepStart = false; // remember, we're dealing with an ASYNC ISR

        do {
            TWDR = twi_slarw;
#if HAVE_TWI_TIMEOUT
            if  ((twi_timeout_us > 0ul) && ((micros() - startMicros) > twi_timeout_us)) {
                twi_handleTimeout(twi_do_reset_on_timeout);
                return 0;
            }
#endif
            ++twi_wc;
        } while (TWCR & _BV(TWWC));

        TWCR = _BV(TWINT) | _BV(TWEA) | _BV(TWEN) | _BV(TWIE); // enable INTs, but not START
    } else {
        // send start condition
        TWCR = _BV(TWINT) | _BV(TWEA) | _BV(TWEN) | _BV(TWIE) | _BV(TWSTA); // enable INTs
    }

    last_error = TWI_OK;
    return TWI_OK;
}

uint8_t twi::write(uint8_t sla, void *data, uint8_t len, uint8_t sendStop)
{
    uint8_t res = write(sla, data, len, sendStop, dummy_cb);
    if (res == TWI_OK) {
        return twi_wait_ready();
    }
    return res;
}

uint8_t twi::reg_write(uint8_t sla, uint8_t reg, uint8_t val)
{
    twi_defaultBuffer[0] = reg;
    twi_defaultBuffer[1] = val;

    return write(sla, twi_defaultBuffer, 2, true);
}

/*
 * Function twi_reply
 * Desc     sends byte or readys receive line
 * Input    ack: byte indicating to ack or to nack
 * Output   none
 */
#define twi_reply_ack() \
    TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT) | _BV(TWEA)

#define twi_reply() \
    TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT)

/* 
 * Function twi_stop
 * Desc     relinquishes bus master status
 * Input    none
 * Output   none
 */
void twi_stop()
{
    // send stop condition
    TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT) | _BV(TWSTO);

    // wait for stop condition to be executed on bus
    // TWINT is not set after a stop condition!
    // We cannot use micros() from an ISR, so approximate the timeout with cycle-counted delays
#if HAVE_TWI_TIMEOUT
    const uint8_t us_per_loop = 8;
    uint32_t counter = (twi_timeout_us + us_per_loop - 1) / us_per_loop; // Round up
#endif

    while (TWCR & _BV(TWSTO)) {
#if HAVE_TWI_TIMEOUT
        if (twi_timeout_us > 0ul) {
            if (counter > 0ul) {
                _delay_us(us_per_loop);
                counter--;
            } else {
                twi_handleTimeout(twi_do_reset_on_timeout);
                return;
            }
        }
#endif
    }

    // update twi state
    twi_state = TWI_READY;
}

void twi::update()
{
    if (twi_cb_ready && twi_state == TWI_READY) {
        twi_cb_ready = false;

        twi_cb();
    }
}

/* 
 * Function twi_releaseBus
 * Desc     releases bus control
 * Input    none
 * Output   none
 */
void twi_releaseBus(void)
{
    // release bus
    TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT);

    // update twi state
    twi_state = TWI_READY;
}

ISR(TWI_vect)
{
    ++twi_isr;

    switch(TW_STATUS){
    // All Master
    case TW_START:     // sent start condition
    case TW_REP_START: // sent repeated start condition
        // copy device address and r/w bit to output register and ack
        TWDR = twi_slarw;
        twi_reply_ack();
        break;

    // Master Transmitter
    case TW_MT_SLA_ACK:  // slave receiver acked address
    case TW_MT_DATA_ACK: // slave receiver acked data
        // if there is data to send, send it, otherwise stop 
        if (twi_masterBufferIndex < twi_masterBufferLength){
            // copy data to output register and ack
            TWDR = twi_masterBuffer[twi_masterBufferIndex++];
            twi_reply_ack();
        } else {
            if (twi_sendStop) {
                twi_stop();
            } else {
                twi_inRepStart = true; // we're gonna send the START
                // don't enable the interrupt. We'll generate the start, but we
                // avoid handling the interrupt until we're in the next transaction,
                // at the point where we would normally issue the start.
                TWCR = _BV(TWINT) | _BV(TWSTA)| _BV(TWEN) ;
                twi_state = TWI_READY;
            }
        }
        break;
    case TW_MT_SLA_NACK:  // address sent, nack received
        twi_error = TW_MT_SLA_NACK;
        twi_stop();
        break;
    case TW_MT_DATA_NACK: // data sent, nack received
        twi_error = TW_MT_DATA_NACK;
        twi_stop();
        break;
    case TW_MT_ARB_LOST: // lost bus arbitration
        twi_error = TW_MT_ARB_LOST;
        twi_releaseBus();
        break;

    // Master Receiver
    case TW_MR_DATA_ACK: // data received, ack sent
        // put byte into buffer
        twi_masterBuffer[twi_masterBufferIndex++] = TWDR;
        __attribute__ ((fallthrough));
    case TW_MR_SLA_ACK:  // address sent, ack received
        // ack if more bytes are expected, otherwise nack
        if (twi_masterBufferIndex < twi_masterBufferLength) {
            twi_reply_ack();
        } else {
            twi_reply();
        }
        break;
    case TW_MR_DATA_NACK: // data received, nack sent
        // put final byte into buffer
        twi_masterBuffer[twi_masterBufferIndex++] = TWDR;
        if (twi_sendStop) {
            twi_stop();
        } else {
            twi_inRepStart = true;	// we're gonna send the START
            // don't enable the interrupt. We'll generate the start, but we
            // avoid handling the interrupt until we're in the next transaction,
            // at the point where we would normally issue the start.
            TWCR = _BV(TWINT) | _BV(TWSTA)| _BV(TWEN) ;
            twi_state = TWI_READY;
        }
        break;
    case TW_MR_SLA_NACK: // address sent, nack received
        twi_stop();
        break;
    // TW_MR_ARB_LOST handled by TW_MT_ARB_LOST case

    // All
    case TW_NO_INFO:   // no state information
        break;
    case TW_BUS_ERROR: // bus error, illegal stop/start
        twi_error = TW_BUS_ERROR;
        twi_stop();
        break;
    }
}
