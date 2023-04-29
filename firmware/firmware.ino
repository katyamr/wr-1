
#define HAVE_LOG_METRICS 1
#define HAVE_DEBUG_PRINT 1
#define HAVE_TRANSMITTER 1

#include "BMP280.h"
#include "MPU9250.h"
#include "DS3231.h"

#include <Servo.h>
#include <SD.h>

#include "buffer.h"
#include "kalman_filter.h"
#include "tone_sweep.h"

#include "uart.h"
#include "ring.h"

#include "artl/button.h"
#include "artl/digital_in.h"
#include "artl/digital_out.h"
#include "artl/timer.h"
#include "artl/tc.h"
#include "artl/yield.h"
#include "crit_sec.h"

#include "twi.h"

enum {
    SERVO_UNKNOWN,
    SERVO_CLOSED,
    SERVO_OPEN,
} servo_state = SERVO_UNKNOWN;

Servo save_servo;
const int servo_close_pos = 67;
const int servo_open_pos = 30;
const uint32_t servo_timeout = 500;
const int save_servo_pin = 9;
const float servo_open_threshold = 0.4;

using test_button = artl::digital_in<artl::port::F, 4>; // pin A3
const int sd_cs_pin = 4;    /* SD card chip select */
const int battery_pin = A2;

// AD0/SDO to change BMP280 & MPU9250 i2c IDs
using gy91_id = artl::digital_out<artl::port::F, 7>; // pin A0

artl::button<> button_state;
artl::timer<> servo_timer;

BMP280 barometer;
artl::timer<> measure_timer;
const uint32_t measure_delay = 10; // ms

MPU9250 imu;

using green_led = artl::digital_out<artl::port::C, 6>; // pin D5
using yellow_led = artl::digital_out<artl::port::D, 7>; // pin D6

const char TeamID[] = "RM";

int Va;
float bat = 0;
float acc = 0;

uint32_t start_point = 0;
uint32_t apogee_point = 0;
uint32_t activate_point = 0;
int landing_point = 0;
const float min_vel = -2.0; // m / s

kalman_filter_t<float> alti_filter(0.0, 1.0, 1.0, 0.5);
float apogee = 0;
float ascent_start = 0;
float descent_start = 0;

uint32_t last_wt = 0;
uint32_t measures = 0;

uint32_t last_log_t = 0;

#if HAVE_DEBUG_PRINT
artl::timer<> debug_print_timer;
const uint32_t debug_print_delay = 300;
#endif

struct alti_history {
    uint16_t t;
    float alti;
};

ring<5, alti_history> alti_ring;

const uint32_t loop_delay_ms = 500;

File log_file;
buffer log_buffer;
buffer debug_buffer;

uint32_t transmitter_bytes = 0;

#if HAVE_TRANSMITTER
using transmitter = uart_t<9600>;
template<> uint8_t transmitter::write_size = 0;
template<> const uint8_t *transmitter::write_data = 0;

buffer transmitter_buffer;

ISR(USART1_UDRE_vect) {
    transmitter::on_dre_int();
}
#endif

void transmitter_setup() {
#if HAVE_TRANSMITTER
    transmitter::setup();
    transmitter_buffer.reserve(100);
#endif
}

float adc_to_battery(int v) {
    return 3.0 * 5.0 * v / 1023.0 + 0.29 /* diode dropout */;
}

ISR(ADC_vect) {
    Va = ADC;
    bat = adc_to_battery(Va);
}

using input_tc = artl::tc<4>;

#if HAVE_DEBUG_PRINT
uint32_t tc4_count = 0;
#endif

ISR(TIMER4_COMPA_vect)
{
#if HAVE_DEBUG_PRINT
    ++tc4_count;
#endif
    ADCSRA |= (1 << ADSC); // Start Conversion
    input_tc().cnt() = 0;
}

tone_sweep_t tone_sweep;
const float tone_sweep_threshold = 0.4;

void alti_ring_push(uint32_t t, float v) {
    if (alti_ring.full()) alti_ring.pop_front();
    alti_history h{(uint16_t) t, v};
    alti_ring.push_back(h);
}

void servo_close(uint32_t t);

void setup() {
    {
        crit_sec cs;

        // setup battery ADC pin
        // pinMode(battery_pin, INPUT);
        ADCSRB = 0; // Free running
        ADMUX = (1 << 6) // AVCC with external capacitor on AREF pin
                | (analogPinToChannel(battery_pin - 18) & 0x07);
        ADCSRA |= (1 << ADIE) // Interrupt Enable
                | (1 << ADSC); // Start Conversion

        transmitter_setup();

        input_tc().setup(0, 0, 4, input_tc::cs::presc_2048);
        input_tc().ocra() = 75;
        input_tc().cnt() = 0;
        input_tc().oca().enable();
    }

    Serial.begin(115200);

    twi::init();

    test_button::setup();
    green_led::setup();
    yellow_led::setup();

    int buttonState = test_button::read();

    // Pull UP AD0/SDO to change BMP280 & MPU9250 i2c IDs
    gy91_id::setup();
    gy91_id::high();

    tone_sweep.setup();
    barometer.setup();
    imu.setup();

    bool sd_ready = SD.begin(sd_cs_pin);
    if (sd_ready) {
        buffer fname;
        fname.reserve(12);
        int n = 20;
        do {
            n += 1;
            fname.reset() << "data-" << n << ".txt";
        } while (SD.exists(fname.c_str()));

        log_file = SD.open(fname.c_str(), FILE_WRITE);
        log_buffer.reserve(256); //512 + 256);

        date_time now;
        now.read();

        log_buffer << now << '\n';

        log_file.write(log_buffer.data(), log_buffer.length());
        log_file.flush();

        tone_sweep.beep(1046, 100);
        tone_sweep.beep(1318, 200);
        tone_sweep.beep(1567, 100);
    }

    if (barometer.ready()) {
        for (uint8_t i = 0; i < 5; ++i) {
            delay(measure_delay);
            barometer.read();
        }
        barometer.reset_slp(0);
        while (!alti_ring.full()) {
            delay(measure_delay);
            barometer.read();
            alti_ring_push(millis(), alti_filter(barometer.alti));
        }

        tone_sweep.beep(880, 100);
        measure_timer.schedule(millis());
    }

    if (bat < 6.5) {
        for (uint8_t i = 0; i < 3; ++i) {
            delay(100);
            tone_sweep.beep(440, 200);
        }
    }

    uint32_t t = millis();

    green_led::high();

    if (buttonState) {
        servo_close(t);
    }

#if HAVE_DEBUG_PRINT
    debug_print_timer.schedule(t);
#endif
}

#if HAVE_TRANSMITTER
void send_metrics(uint32_t t) {
    transmitter_buffer.reset()
        << TeamID << ';'
        << t << ';'
        << barometer.alti << ';'
        << bat << ';'
        << acc << ';'
        << start_point << ';'
        << apogee_point << ';'
        << activate_point << ';'
        << landing_point << '\n';

    transmitter::write(transmitter_buffer.data(), transmitter_buffer.length());

    transmitter_bytes += transmitter_buffer.length();
}
#endif

void log_metrics(uint32_t t) {
#if HAVE_LOG_METRICS
    log_buffer.reset()
        << TeamID << ';'
        << t << ';'
        << barometer.alti << ';'
        << (float) alti_filter << ';'
        << bat << ';'
        << acc << ';'
        << start_point << ';'
        << apogee_point << ';'
        << activate_point << ';'
        << landing_point << ';'
        << last_wt << ';'
        << barometer.pres << ';'
        << barometer.temp << ';'
        << transmitter_bytes << ";"
        << servo_state << ';'
        << '\n';

    green_led::low();
    yellow_led::high();

    uint32_t t1 = millis();
    log_file.write(log_buffer.data(), log_buffer.length());
    log_file.flush();
    last_wt = millis() - t1;

    yellow_led::low();
    green_led::high();
#endif
}

void servo_activate(uint32_t t, int pos) {
    yellow_led::high();
    save_servo.attach(save_servo_pin);
    save_servo.write(pos);

    servo_timer.schedule(t + servo_timeout);
}

void servo_open(uint32_t t) {
    servo_activate(t, servo_open_pos);
    servo_state = SERVO_OPEN;
}

void servo_close(uint32_t t) {
    servo_activate(t, servo_close_pos);
    servo_state = SERVO_CLOSED;
}

void servo_update(uint32_t t) {
    if (servo_timer.update(t)) {
        yellow_led::low();
        save_servo.detach();
    }
}

void loop() {
    uint32_t t = millis();

    bool b = test_button::read();
    if (button_state.update(b, t) && button_state.down()) {
        if (!servo_timer.active()) {
            servo_open(t);
        } else {
            servo_close(t);
        }
    }

    //artl::twi::update();
    servo_update(t);
    tone_sweep.update(t);

    bool up = false;

    if (measure_timer.update(t)) {
        measure_timer.schedule(t + measure_delay);

        if (barometer.ready()) {
            barometer.read();
        }

        if (imu.ready()) {
            imu.read();
        }

        ++measures;

        float f = alti_filter(barometer.alti);
        alti_history front = alti_ring.front();
        float d = f - front.alti;

        if (d < -servo_open_threshold && servo_state != SERVO_OPEN) {
            servo_open(t);
        }

        if (!tone_sweep.active()) {
            if (d > tone_sweep_threshold) {
                tone_sweep.start(t, 880, 880 * 4, 300);
            }
            if (d < -tone_sweep_threshold) {
                tone_sweep.start(t, 880 * 4, 880, 300);
            }
        }

        alti_ring_push(t, f);

        up = true;
    }

#if HAVE_DEBUG_PRINT
    if (debug_print_timer.update(t)) {
        debug_print_timer.schedule(t + debug_print_delay);

        if (Serial.dtr()) {
            date_time now;
            now.read();

            debug_buffer.ensure_capacity(90);
            debug_buffer.reset()
                << now << '\t'
                << barometer.alti << '\t'
                << (float) alti_filter << '\t'
                << bat << '\t'
                << tc4_count << '\t'
                << measures << '\t'
                << barometer.temp << '\t'
                << barometer.pres << '\t'
                << imu.acc[0] << ',' << imu.acc[1] << ',' << imu.acc[2] << '\t'
                //<< v[3] << '\t'
                //<< v[4] << ',' << v[5] << ',' << v[6] << '\t'
                << imu.temp << '\t'
                << imu.mag[0] << ',' << imu.mag[1] << ',' << imu.mag[2] << '\t'
                //<< barometer.temp_i << '\t'
                //<< now.year() << '/' << now.month() << '/' << now.day() << ' '
                //<< now.hour() << ':' << now.minute() << ':' << now.second()
                << '\n' << '\r';

            {
                crit_sec cs;

                measures = 0;
                tc4_count = 0;
            }

            Serial.write(debug_buffer.c_str(), debug_buffer.length());
        }
    }
#endif

#if HAVE_TRANSMITTER
    if (transmitter::write_ready()) {
        send_metrics(t);
    }
#endif

    if (log_file && up) { // && (t - last_log_t) > 100) {
        last_log_t = t;
        log_metrics(t);
    }

    artl::yield();
}
