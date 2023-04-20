
#define HAVE_LOG_METRICS 1
#define HAVE_DEBUG_PRINT 0
#define HAVE_TRANSMITTER 1

#include "VoltBroSensors/VB_BMP280.h"
#include "VoltBroSensors/VB_BMP280.cpp"
#include "VoltBroSensors/arduino_mpu9250_VB_routines.cpp"
#include "VoltBroSensors/VB_MPU9250.h"
// include "VoltBroSensors/VB_MPU9250.cpp"

#include <Servo.h>
#include <SD.h>
#include <Wire.h>

#include "buffer.h"
#include "kalman_filter.h"
#include "tone_sweep.h"

#include "DS3231.h"
#include "uart.h"
#include "ring.h"

#include "artl/button.h"
#include "artl/timer.h"
#include "artl/tc.h"
#include "artl/yield.h"
#include "crit_sec.h"

Servo save_servo;
const int save_servo_pin = 9;
const int button_pin = A3;
const int sd_cs_pin = 4;    /* SD card chip select */
const int battery_pin = A2;

artl::button<> button_state;
artl::timer<> servo_timer;

VB_BMP280 barometer; 
bool barometer_connection;
artl::timer<> barometer_timer;
const uint32_t barometer_measure_delay = 20; // ms

const int green_led_pin = 5;
const int yellow_led_pin = 6;
const char TeamID[] = "RM";

int Va;
float bat = 0;
float acc = 0;

int start_point = 0;
int apogee_point = 0;
uint32_t activate_point = 0;
int landing_point = 0;
const float min_vel = -2.0; // m / s

kalman_filter_t<float> alti_filter(0.0, 1.0, 1.0, 0.5);

uint32_t last_t = 0;
float last_vel = 0;
uint32_t last_wt = 0;
uint32_t measures = 0;

uint32_t last_log_t = 0;

#if HAVE_DEBUG_PRINT
artl::timer<> debug_print_timer;
const uint32_t debug_print_delay = 1000;
#endif

float last_sweep_alti = 0;
artl::timer<> sweep_timer;
ring<10, float> alti_ring;

const uint32_t loop_delay_ms = 500;

File log_file;
buffer log_buffer;
buffer debug_buffer;
uint32_t log_failed = 0;
uint32_t log_ovf = 0;

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
uint32_t tc4_count = 0;

ISR(TIMER4_COMPA_vect)
{
    ++tc4_count;
    ADCSRA |= (1 << ADSC); // Start Conversion
    input_tc().cnt() = 0;
}

tone_sweep_t tone_sweep;

void alti_ring_push(uint32_t t, float v) {
    if (alti_ring.full()) alti_ring.pop_front();
    alti_ring.push_back(v);
}

void setup_sensors() {
    barometer.start_altitude = 0;
    barometer_connection = barometer.begin(
        BMP280_ALTERNATIVE_ADDRESS,
        (BMP280_TSB_0_5 << 5) | (BMP280_FILTER_OFF << 2) | (BMP280_SPI_OFF),
        (BMP280_OVERSAMPLING_T1 << 5) | (BMP280_OVERSAMPLING_P2 << 2) | (BMP280_MODE_NORMAL)
    );
}

void setup() {
    // setup battery ADC pin
    pinMode(battery_pin, INPUT);
    ADCSRB = 0; // Free running
    ADMUX = (1 << 6) // AVCC with external capacitor on AREF pin
            | (analogPinToChannel(battery_pin - 18) & 0x07);
    ADCSRA |= (1 << ADIE) // Interrupt Enable
            | (1 << ADSC); // Start Conversion

    transmitter_setup();

    input_tc().setup(0, 0, 4, input_tc::cs::presc_2048);
    input_tc().ocra() = 150;
    input_tc().cnt() = 0;
    input_tc().oca().enable();

    Serial.begin(115200);
    pinMode(button_pin, INPUT);
    pinMode(green_led_pin, OUTPUT);
    int buttonState = digitalRead(button_pin);

    // Pull UP AD0/SDO to change BMP280 & MPU9250 i2c IDs
    pinMode(A0, OUTPUT);
    digitalWrite(A0, HIGH);

    tone_sweep.setup();
    setup_sensors();

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

        log_buffer << now.year() << now.month() << now.day()
            << '-' << now.hour() << now.minute() << now.second() << '\n';

        log_file.write(log_buffer.c_str(), log_buffer.length());
        log_file.flush();

        pinMode(yellow_led_pin, OUTPUT);

        tone_sweep.beep(1046, 100);
        tone_sweep.beep(1318, 200);
        tone_sweep.beep(1567, 100);
    }

    if (barometer_connection) {
        for (uint8_t i = 0; i < 5; ++i) {
            delay(20);
            barometer.read();
        }
        barometer.reset_SLP();
        while (!alti_ring.full()) {
            delay(barometer_measure_delay);
            barometer.read();
            alti_ring_push(millis(), alti_filter(barometer.alti));
        }
        last_sweep_alti = alti_filter;

        tone_sweep.beep(880, 50);
        barometer_timer.schedule(millis());
    }

    uint32_t t = millis();

    digitalWrite(green_led_pin, HIGH);

    if (buttonState) {
        activate_servo(t, 65);
    }

#if HAVE_DEBUG_PRINT
    debug_print_timer.schedule(t);
#endif
    sweep_timer.schedule(t);
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
        << last_vel << ';'
  //      << log_ovf << ';'
  //      << log_failed << ";"
        << '\n';
  /*
    if (log_buffer.full()) {
      ++log_ovf;
      log_buffer.remove(log_pos, log_buffer.length() - log_pos);
    }
  */
    unsigned int chunkSize = log_buffer.length(); //log_file.availableForWrite();
    if (chunkSize == 0) {
        ++log_failed;
    }

    if (chunkSize && log_buffer.length() >= chunkSize) {
        digitalWrite(green_led_pin, LOW);
        digitalWrite(yellow_led_pin, HIGH);

        uint32_t t1 = millis();
        log_file.write(log_buffer.c_str(), chunkSize);
        log_file.flush();
        last_wt = millis() - t1;

        digitalWrite(yellow_led_pin, LOW);
        digitalWrite(green_led_pin, HIGH);
        //log_buffer.remove(0, chunkSize);
        log_buffer.reset();
        //last_wt += chunkSize;
    }

    //log_file.flush();

    //uint32_t t2 = millis();
    //last_wt = t2 - t1;
#endif
}

void activate_servo(uint32_t t, int value) {
    digitalWrite(yellow_led_pin, HIGH);
    save_servo.attach(save_servo_pin);
    save_servo.write(value);

    servo_timer.schedule(t + 500);
}

void update_servo(uint32_t t) {
    if (servo_timer.update(t)) {
        digitalWrite(yellow_led_pin, LOW);
        save_servo.detach();
    }
}

void loop() {
    uint32_t t = millis();

    int b = digitalRead(button_pin);
    if (button_state.update(b, t)
        && button_state.down()
        && !servo_timer.active()) {
        activate_servo(t, 30);
    }

    update_servo(t);
    tone_sweep.update(t);

    bool up = false;

    if (barometer_connection && barometer_timer.update(t)) {
        barometer_timer.schedule(t + barometer_measure_delay);

        barometer.read();

        ++measures;

        float last_alti = alti_filter;
        float f = alti_filter(barometer.alti);
        alti_ring_push(t, f);

        float vel = 1000.0 * (f - last_alti) / (t - last_t);

        last_t = t;
        last_vel = vel;
        up = true;
    }

/*
    if (activate_point == 0 && vel < min_vel && last_vel < min_vel) {
        activate_point = t;
        digitalWrite(green_led_pin, HIGH);
        save_servo.attach(save_servo_pin);
        save_servo.write(30); 
    }
*/

#if HAVE_DEBUG_PRINT
    if (debug_print_timer.update(t)) {
        debug_print_timer.schedule(t + debug_print_delay);

        if (Serial.dtr()) {
            debug_buffer.ensure_capacity(80);
            debug_buffer.reset()
                << last_vel << '\t'
                << (float) alti_filter << '\t'
                << bat << '\t'
                << tc4_count << '\t'
                << measures << '\t'
                << barometer.temp << '\t'
                //<< barometer.temp_i << '\t'
                << barometer.pres << '\t'
                //<< barometer.pres_i << '\t'
                //<< now.year() << '/' << now.month() << '/' << now.day() << ' '
                //<< now.hour() << ':' << now.minute() << ':' << now.second()
                << '\n';

            {
                crit_sec cs;

                measures = 0;
                tc4_count = 0;
            }

            Serial.write(debug_buffer.c_str(), debug_buffer.length());
        }
    }
#endif

    if (sweep_timer.update(t)) {
        sweep_timer.schedule(t + 300);
/*
        float d = (float) alti_filter - last_sweep_alti;
        if (fabs(d) > 0.5) {
            if (d > 0.0) {
                tone_sweep.start(t, 880, 880 * 4, 300);
            } else {
                tone_sweep.start(t, 880 * 4, 880, 300);
            }
        }
*/
        last_sweep_alti = alti_filter;
    }

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
