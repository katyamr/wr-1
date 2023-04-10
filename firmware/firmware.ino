#include "VoltBroSensors/VB_BMP280.h"
#include "VoltBroSensors/VB_BMP280.cpp"
#include "VoltBroSensors/arduino_mpu9250_VB_routines.cpp"
#include "VoltBroSensors/VB_MPU9250.h"
// include "VoltBroSensors/VB_MPU9250.cpp"

#include <Servo.h>
#include <SD.h>
#include <Wire.h>

#include "buffer.hpp"
#include "oversample_ring.h"
#include "tone_sweep.h"

#include "DS3231.h"

Servo save_servo;
const int save_servo_pin = 9;
const int button_pin = A3;
const int sd_cs_pin = 4;    /* SD card chip select */
const int battery_pin = A2;

VB_BMP280 barometer; 
bool barometer_connection;
const int green_led_pin = 5;
const int yellow_led_pin = 6;
const char TeamID[] = "RM";
float bat = 0;
float acc = 0;

int start_point = 0;
int apogee_point = 0;
uint32_t activate_point = 0;
int landing_point = 0;
const double min_vel = -2.0; // m / s

oversample_ring<10, float> alt_ring;

float h_sum = 0;

uint32_t last_t = 0;
double last_vel = 0;
uint32_t last_wt = 0;
uint32_t measures = 0;

uint32_t last_metrics_t = 0;
uint32_t last_log_t = 0;
uint32_t last_debug_t = 0;

uint32_t last_sweep_t = 0;
float last_sweep_h = 0;

uint32_t delay_ms = 20;

File log_file;
//String log_buffer;
buffer log_buffer;
buffer send_buffer;
buffer debug_buffer;
uint32_t log_ovf = 0;
uint32_t log_failed = 0;

tone_sweep_t tone_sweep;

void setup_sensors() {
  barometer.start_altitude = 0;
  barometer_connection = barometer.begin(
      BMP280_ALTERNATIVE_ADDRESS,
      (BMP280_TSB_0_5 << 5) | (BMP280_FILTER_OFF << 2)|(BMP280_SPI_OFF),
      (BMP280_OVERSAMPLING_T1 << 5)|(BMP280_OVERSAMPLING_P4 << 2)|(BMP280_MODE_NORMAL)
  );
}

void read_battery() {
  int Va = analogRead(battery_pin);
  bat = 3.0 * 5.0 * Va / 1023.0 + 0.29 /* diode dropout */;
}

void setup() {
  Serial1.begin(9600);
  send_buffer.reserve(100);

  Serial.begin(115200);
  pinMode(button_pin, INPUT);
  pinMode(battery_pin, INPUT);
  pinMode(green_led_pin, OUTPUT);

  pinMode(A0, OUTPUT);
  digitalWrite(A0, HIGH);

  tone_sweep.setup();
  setup_sensors();

  bool sd_ready = SD.begin(sd_cs_pin);
  if (sd_ready) {
    String fname;
    int n = 20;
    String data_name = "data-";
/*
    date_time now;
    now.read();
    data_name += now.year();
    data_name += now.month();
    data_name += now.day();
    data_name += "-";
    data_name += now.hour();
    data_name += now.minute();
    data_name += now.second();
    data_name += "-";
*/
    do {
      n += 1;
      fname = data_name + n;
      fname += ".txt";
    } while (SD.exists(fname));

    log_file = SD.open(fname, FILE_WRITE);
    log_buffer.reserve(256); //512 + 256);

    pinMode(yellow_led_pin, OUTPUT);

    tone_sweep.beep(1046, 50);
    tone_sweep.beep(1318, 50);
    tone_sweep.beep(1567, 50);
  }

  if (barometer_connection) {

    memset(&alt_ring, 0, sizeof(alt_ring));
    for (uint8_t i = 0; i < alt_ring.capacity * 2; ++i) {
      delay(20);
      barometer.read();
      alt_ring.update(barometer.alti);
    }

    setup_sensors();

    tone_sweep.beep(880, 50);
  }

  digitalWrite(green_led_pin, HIGH);
}

void send_metrics(uint32_t t, double h, double acc, double vel, int buttonState) {
    send_buffer.reset();

    send_buffer
        << TeamID << ';'
        << t << ';'
        << h << ';'
        << bat << ';'
        << acc << ';'
        << start_point << ';'
        << apogee_point << ';'
        << activate_point << ';'
        << landing_point << '\n';

    Serial1.write(send_buffer.c_str(), send_buffer.length());
}

#define HAVE_LOG_METRICS 1

void log_metrics(uint32_t t, double h, double acc, double vel) {
#if HAVE_LOG_METRICS
  //uint32_t t1 = millis();

  //log_buffer = "";

  uint16_t log_pos = log_buffer.length();

  log_buffer += TeamID;
  log_buffer += ";";
  log_buffer += t;
  log_buffer += ";";
  log_buffer += h;
  log_buffer += ";";
  log_buffer += bat;
  log_buffer += ";";
  log_buffer += acc;
  log_buffer += ";";
  log_buffer += start_point;
  log_buffer += ";";
  log_buffer += apogee_point;
  log_buffer += ";";
  log_buffer += activate_point;
  log_buffer += ";";
  log_buffer += landing_point;

  log_buffer += ";";
  log_buffer += last_wt;
  log_buffer += ";";
  log_buffer += barometer.pres;
  log_buffer += ";";
  log_buffer += barometer.temp;
  log_buffer += ";";
  log_buffer += vel;
  log_buffer += ";";
  log_buffer += log_ovf;
  log_buffer += ";";
  log_buffer += log_failed;
  log_buffer += "\n";

  if (log_buffer.full()) {
    ++log_ovf;
    log_buffer.remove(log_pos, log_buffer.length() - log_pos);
  }

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
    log_buffer.remove(0, chunkSize);
    //last_wt += chunkSize;
  }

  //log_file.flush();

  //uint32_t t2 = millis();
  //last_wt = t2 - t1;
#endif
}

void loop() {
  int buttonState = digitalRead(button_pin);
/*
  if (buttonState != 0) { 
    digitalWrite(green_led_pin, HIGH);
    save_servo.attach(save_servo_pin);
    save_servo.write(30);
  } else {
    digitalWrite(green_led_pin, LOW);
    save_servo.detach();
  }
*/

  read_battery();

  if (barometer_connection) {
    barometer.read();

    ++measures;

    uint32_t t = millis();
    float h = barometer.alti;

    float h_sum_0 = alt_ring.sum;

    alt_ring.update(h);

    float h_avg = alt_ring.avg();
    float vel = 1000.0 * (alt_ring.sum - h_sum_0) / alt_ring.capacity / (t - last_t);
/*
    if (activate_point == 0 && vel < min_vel && last_vel < min_vel) {
      activate_point = t;
      digitalWrite(green_led_pin, HIGH);
      save_servo.attach(save_servo_pin);
      save_servo.write(30); 
    }
*/
    last_t = t;
    last_vel = vel;

    tone_sweep.update(t);

    if ((t - last_debug_t) > 1000) {

      if (Serial.dtr()) {
          debug_buffer.ensure_capacity(80);
          debug_buffer.reset();
          //date_time now;
          //now.read();
          debug_buffer
              << vel << '\t'
              << h_avg << '\t'
              << measures << '\t'
              << barometer.temp << '\t'
              //<< barometer.temp_i << '\t'
              << barometer.pres << '\t'
              //<< barometer.pres_i << '\t'
              //<< now.year() << '/' << now.month() << '/' << now.day() << ' '
              //<< now.hour() << ':' << now.minute() << ':' << now.second()
              << '\n';

          measures = 0;

          Serial.write(debug_buffer.c_str(), debug_buffer.length());
      }

      //log_file.flush();

      last_debug_t = t;
    }

    if (last_sweep_t + 300 < t) {
      if (fabs(last_sweep_h - h_avg) > 0.5) {
          if (h_avg > last_sweep_h) {
              tone_sweep.start(t, 880, 880 * 4, 300);
          } else {
              tone_sweep.start(t, 880 * 4, 880, 300);
          }
      }

      last_sweep_h = h_avg;
      last_sweep_t = t;
    }

    if (1 && (t - last_metrics_t) > 100) {
      last_metrics_t = t;
      send_metrics(t, h, h_avg, vel, buttonState);
    }

    if (log_file) { // && (t - last_log_t) > 100) {
      last_log_t = t;
      log_metrics(t, h, h_avg, vel);
    }
  } else {
    if (Serial.dtr()) {
      Serial.println("Подключены не все датчики");
    }
    setup_sensors();
  }
  
  //delay(delay_ms);
}
