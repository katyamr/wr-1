
#include "config.h"

#include "BMP280.h"
#include "MPU9250.h"
#include "DS3231.h"

#include <Servo.h>

#include "buffer.h"
#include "kalman_filter.h"
#include "tone_sweep.h"

#include "ring.h"

#include "artl/button.h"
#include "artl/digital_in.h"
#include "artl/digital_out.h"
#include "artl/timer.h"
#include "artl/tc.h"
#include "artl/yield.h"
#include "crit_sec.h"

#if HAVE_TWI
#include "twi.h"
#endif

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
const float xy_ready_threshold = 0.25;
bool xy_ready;

enum move_dir_t {
    MOVE_DIR_NONE,
    MOVE_DIR_UP,
    MOVE_DIR_DOWN,
};
move_dir_t move_dir = MOVE_DIR_NONE;
float dir_start;

const float start_min_ascent = 2.0; // m
const float start_min_acc = 9.81 * 2; // m/s**2

const float apogee_min_descent = 2.0; // m

const float activate_min_speed = 4.0; // m/s
const float activate_min_descent = 2.0; // m

uint8_t start_point = 0;
uint8_t apogee_point = 0;
uint8_t activate_point = 0;
uint8_t landing_point = 0;

kalman_filter_t<float> alti_filter(0.0, 1.0, 1.0, 0.5);
float apogee = 0;
float ascent_start = 0;
float descent_start = 0;

uint32_t measures = 0;
uint32_t loop_count = 0;

const uint32_t debug_print_delay = 500;

void debug_setup(uint32_t t);
void debug_header();
void debug_metrics(uint32_t t);

void heartbeat_setup();
void heartbeat_start(uint32_t t);
void heartbeat(uint32_t t);

struct alti_history {
    uint32_t t;
    float alti;
};

ring<10, alti_history> alti_ring;

void log_header();
void log_setup();
void log_metrics(uint32_t t);

void transmitter_setup();
void transmitter_send_metrics(uint32_t t);

float adc_to_battery(int v) {
    return 3.0 * 5.0 * v / 1023.0 + 0.29 /* diode dropout */;
}

ISR(ADC_vect) {
    Va = ADC;
    bat = adc_to_battery(Va);
}

using input_tc = artl::tc<4>;

uint32_t tc4_count = 0;
volatile uint8_t time_to_measure = 0;

ISR(TIMER4_COMPA_vect)
{
    ++tc4_count;

    ADCSRA |= (1 << ADSC); // Start Conversion
    input_tc::cnt(0);
    time_to_measure = 1;
}

tone_sweep_t tone_sweep;
const float tone_sweep_threshold = 4.0; // m/s
float last_v = 0;

void alti_ring_push(uint32_t t, float v) {
    if (alti_ring.full()) alti_ring.pop_front();
    alti_history h{t, v};
    alti_ring.push_back(h);
}

void servo_activate(uint32_t t, int pos);
void servo_open(uint32_t t);
void servo_close(uint32_t t);
void servo_update(uint32_t t);

void setup() {
    heartbeat_setup();

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

        // F_CPU / 2048 / 37 = 104 Hz
        input_tc::setup(0, 0, 4, input_tc::cs::presc_2048);
        input_tc::ocra(75);
        input_tc::cnt(0);
        input_tc::oca::enable();
    }

    test_button::setup();
    green_led::setup();
    yellow_led::setup();

    int buttonState = test_button::read();

    // Pull UP AD0/SDO to change BMP280 & MPU9250 i2c IDs
    gy91_id::setup();
    gy91_id::high();

    tone_sweep.setup();

#if HAVE_TWI
    twi::init();

    barometer.setup();
    imu.setup();
#endif

    log_setup();

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

    debug_setup(t);
    heartbeat_start(t);
}

void loop() {
    ++loop_count;

    uint32_t t = millis();

    bool b = test_button::read();
    if (button_state.update(b, t) && button_state.down()) {
        if (!servo_timer.active()) {
            servo_open(t);
        } else {
            servo_close(t);
        }
    }

    //twi::update();
    servo_update(t);

    bool measure_done = false;

    if (measure_timer.update(t)) {
        measure_timer.schedule(t + measure_delay);
    }

    if (time_to_measure) {
        time_to_measure = 0;

        if (imu.ready()) {
            imu.read();

            float xy_acc_2 =
                imu.acc[0] * imu.acc[0] +
                imu.acc[2] * imu.acc[2];

            acc = sqrt(
                xy_acc_2 +
                imu.acc[1] * imu.acc[1]);

            float xy_acc = sqrt(xy_acc_2);

            xy_ready = (xy_acc / acc) < xy_ready_threshold;

            green_led::write(xy_ready);
        }

        if (barometer.ready()) {
            barometer.read();
        }

        ++measures;

        float f = alti_filter(barometer.alti);

        alti_history front = alti_ring.front();
        float d = (f - front.alti) * 1000 / (t - front.t);

        move_dir_t new_move_dir;

        if (d > 0) {
            new_move_dir = MOVE_DIR_UP;
        } else if (d < 0) {
            new_move_dir = MOVE_DIR_DOWN;
        } else {
            new_move_dir = MOVE_DIR_NONE;
        }

        if (move_dir != new_move_dir) {
            move_dir = new_move_dir;
            dir_start = f;
        }

        if (!start_point
            && move_dir == MOVE_DIR_UP
            && (f - dir_start) >= start_min_ascent
            && acc >= start_min_acc) {
            start_point = 1;
        }

        if (!apogee_point
            && move_dir == MOVE_DIR_DOWN
            && (dir_start - f) >= apogee_min_descent) {
            apogee_point = 1;
        }

        if (move_dir == MOVE_DIR_DOWN
            && (dir_start - f) >= activate_min_descent
            && d < -activate_min_speed
            && servo_state != SERVO_OPEN) {
            activate_point = 1;
            servo_open(t);
        }

        if (!landing_point
            && start_point
            && apogee_point
            && move_dir != MOVE_DIR_DOWN) {
            landing_point = 1;
        }

        if (!tone_sweep.active()) {
            if (d > tone_sweep_threshold) {
                tone_sweep.start(880, 880 * 4, 300);
            } else if (d < -tone_sweep_threshold) {
                tone_sweep.start(880 * 4, 880, 300);
            }
        }

        last_v = d;

        alti_ring_push(t, f);

        measure_done = true;
    }

    (void) measure_done;

    heartbeat(t);
    debug_metrics(t);
    transmitter_send_metrics(t);

    if (measure_done) {
        log_metrics(t);
    }

    artl::yield();
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

#if HAVE_TRANSMITTER
#include "uart.h"

using transmitter = uart_t<9600>;
template<> uint8_t transmitter::write_size = 0;
template<> const uint8_t *transmitter::write_data = 0;

buffer transmitter_buffer;

ISR(USART1_UDRE_vect) {
    transmitter::on_dre_int();
}

void transmitter_setup() {
    transmitter::setup();
    transmitter_buffer.reserve(100);
}

void transmitter_send_metrics(uint32_t t) {
    if (transmitter::write_ready()) {
        transmitter_buffer.reset()
            << TeamID << ';'
            << t << ';'
            << barometer.alti << ';'
            << bat << ';'
            << acc << ';'
            << start_point << ';'
            << apogee_point << ';'
            << activate_point << ';'
            << landing_point << ';'

            << loop_count << ';'
            << '\n';

        transmitter::write(transmitter_buffer.data(), transmitter_buffer.length());
    }
}
#else
void transmitter_setup() { }
void transmitter_send_metrics(uint32_t) { }
#endif

#if HAVE_LOG
//# include <SD.h>
#include "SdFat.h"

#define SD sd
#define SPI_CLOCK SD_SCK_MHZ(50)
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SPI_CLOCK)

SdFat sd;

File log_file;
buffer log_buffer;
uint32_t log_size = 0;

void fat_date_time(uint16_t *date, uint16_t *time) {
    date_time now;
    now.read();

    *date = FAT_DATE(2000 + (uint8_t) now.year(), (uint8_t) now.month(), (uint8_t) now.day());
    *time = FAT_TIME((uint8_t) now.hour(), (uint8_t) now.minute(), (uint8_t) now.second());
}

void log_setup() {
    bool sd_ready = SD.begin(sd_cs_pin);
    if (sd_ready) {
        buffer fname;
        fname.reserve(12);
        int n = 20;
        do {
            n += 1;
            fname.reset() << "data-" << n << ".txt";
        } while (SD.exists(fname.c_str()));

        SdFile::dateTimeCallback(fat_date_time);
        log_file = SD.open(fname.c_str(), FILE_WRITE);
        SdFile::dateTimeCallbackCancel();

        log_file.preAllocate(4 * 1000 * 1000);

        log_buffer.reserve(256); //512 + 256);

        log_header();

        tone_sweep.beep(1046, 100);
        tone_sweep.beep(1318, 200);
        tone_sweep.beep(1567, 100);
    }
}

void log_header() {
    date_time now;
    now.read();

    log_buffer.reset() << now << '\n'
        << "Ti" << ';'
        << 'H' << ';'
        << "Temp" << ';'
        << "Pressure" << ';'
        << "H*" << ';'
        << "Vin" << ';'
        << "Ax" << ';' << "Ay" << ';' << "Az" << ';'
        << "Temp*" << ';'
        << "Rx" << ';' << "Ry" << ';' << "Rz" << ';'
        << "Mx" << ';' << "My" << ';' << "Mz" << ';'
        << "Servo" << ';'
        << "Start" << ';'
        << "Apogee" << ';'
        << "Activate" << ';'
        << "Landing" << ';'
        << "Loop" << ';'
        << '\n';

    log_file.write(log_buffer.data(), log_buffer.length());
    //log_file.flush();
}

void log_metrics(uint32_t t) {
    if (log_file) {
        log_buffer.reset()
            << t << ';'
            << barometer.alti << ';'
            << barometer.temp << ';'
            << barometer.pres << ';'
            << (float) alti_filter << ';'
            << bat << ';'
            << imu.acc[0] << ';' << imu.acc[1] << ';' << imu.acc[2] << ';'
            << imu.temp << ';'
            << imu.gyro[0] << ';' << imu.gyro[1] << ';' << imu.gyro[2] << ';'
            << imu.mag[0] << ';' << imu.mag[1] << ';' << imu.mag[2] << ';'
            << servo_state << ';'
            << start_point << ';'
            << apogee_point << ';'
            << activate_point << ';'
            << landing_point << ';'
            << loop_count << ';'
            << '\n';

        green_led::low();
        yellow_led::high();

        log_file.write(log_buffer.data(), log_buffer.length());
        //log_file.flush();

        yellow_led::low();
        green_led::high();

        uint32_t n0 = log_size / 512;
        log_size += log_buffer.length();
        uint32_t n1 = log_size / 512;

        if (n0 != n1) {
            log_file.sync();
        }
    }
}
#else
void log_header() { }
void log_setup() { }
void log_metrics(uint32_t) { }
#endif

#if HAVE_DEBUG
artl::timer<> debug_print_timer;
buffer debug_buffer;
bool debug_header_done = false;

void debug_setup(uint32_t t) {
    Serial.begin(115200);

    debug_print_timer.schedule(t);
}

void debug_header() {
    debug_buffer.ensure_capacity(90);
    debug_buffer.reset()
        << "Ti" << '\t'
        << "H" << '\t'
        << "H*" << '\t'
        << "Vin" << '\t'
        << "Vel" << '\t'
        << "C" << '\t'
        << "M" << '\t'
        << "L" << '\t'
        << "Temp" << '\t'
        << "Pressure" << '\t'
        << "Ax" << ',' << "Ay" << ',' << "Ax" << '\t'
        << "Acc" << '\t'
        << '\n' << '\r';

    Serial.write(debug_buffer.c_str(), debug_buffer.length());
}

void debug_metrics(uint32_t t) {
    while (Serial.available()) {
        (void) Serial.read();
    }

    if (!debug_print_timer.update(t)) {
        return;
    }

    debug_print_timer.schedule(t + debug_print_delay);

    if (Serial.dtr() == 0) {
        debug_header_done = false;
        return;
    }

    if (!debug_header_done) {
        debug_header();
        debug_header_done = true;
    }

    debug_buffer.ensure_capacity(90);
    debug_buffer.reset()
        << t << '\t'
        << barometer.alti << '\t'
        << (float) alti_filter << '\t'
        << bat << '\t'
        << last_v << '\t'
        << tc4_count << '\t'
        << measures << '\t'
        << loop_count << '\t'
        << barometer.temp << '\t'
        << barometer.pres << '\t'
        << imu.acc[0] << ',' << imu.acc[1] << ',' << imu.acc[2] << '\t'
        << acc << '\t'
        //<< v[3] << '\t'
        //<< v[4] << ',' << v[5] << ',' << v[6] << '\t'
        //<< imu.temp << '\t'
        //<< imu.gyro[0] << ',' << imu.gyro[1] << ',' << imu.gyro[2] << '\t'
        //<< imu.mag[0] << ',' << imu.mag[1] << ',' << imu.mag[2] << '\t'
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
#else
void debug_setup(uint32_t) { }
void debug_header() { }
void debug_metrics(uint32_t) { }
#endif

#if HAVE_HEARTBEAT
artl::timer<> heartbeat_timer;
const uint32_t heartbeat_delay = 100;

using rxled = artl::digital_out<artl::port::B, 0>;
using txled = artl::digital_out<artl::port::D, 5>;

void heartbeat_setup() {
    rxled::setup();
    txled::setup();

    for (uint8_t i = 0; i < 6; ++i) {
        rxled::toggle();
        delay(heartbeat_delay);
    }
}

void heartbeat_start(uint32_t t) {
    for (uint8_t i = 0; i < 6; ++i) {
        txled::toggle();
        delay(heartbeat_delay);
    }

    heartbeat_timer.schedule(t + heartbeat_delay);
}

void heartbeat(uint32_t t) {
    if (heartbeat_timer.update(t)) {
        heartbeat_timer.schedule(t + heartbeat_delay);

        rxled::toggle();
    }
}
#else
void heartbeat_setup() { }
void heartbeat_start(uint32_t) { }
void heartbeat(uint32_t) { }
#endif

