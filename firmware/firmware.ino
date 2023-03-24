#include <VB_BMP280.h>
#include <Servo.h>
#include <SD.h>

#include "buffer.hpp"
#include "oversample_ring.h"
#include "tone_sweep.h"

Servo save_servo;
const int save_servo_pin = 9;
const int pushButton = A3;
const int sd_cs_pin = 4;

VB_BMP280 barometer; 
bool barometer_connection;
const int green_led_pin = 5;
const int yellow_led_pin = 6;
const char TeamID[] = "RM";
double bat = 0;
double acc = 0;

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

uint32_t last_metrics_t = 0;
uint32_t last_log_t = 0;
uint32_t last_debug_t = 0;
float last_debug_h = 0;

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
      BMP280_DEFAULT_ADDRESS,
      (BMP280_TSB_0_5 << 5) | (BMP280_FILTER_OFF << 2)|(BMP280_SPI_OFF),
      (BMP280_OVERSAMPLING_T1 << 5)|(BMP280_OVERSAMPLING_P2 << 2)|(BMP280_MODE_NORMAL)
  );
}

void read_battery() {
  int Va = analogRead(A2);
  bat = 3.0 * 5.0 * Va / 1023.0 + 0.29;
}

void setup() {
  Serial1.begin(9600);
  send_buffer.reserve(100);

  Serial.begin(115200);
  pinMode(pushButton, INPUT);
  pinMode(green_led_pin, OUTPUT);

  tone_sweep.setup();

  setup_sensors();

  bool sd_ready = SD.begin(sd_cs_pin);
  if (sd_ready) {
    String fname;
    int n = 50;

    do {
      n += 10;
      fname = "data";
      fname += n;
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
    for (uint8_t i = 0; i < alt_ring.capacity; ++i) {
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

void log_metrics(uint32_t t, double h, double acc, double vel) {
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
}

void loop() {
  int buttonState = digitalRead(pushButton);
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
          debug_buffer
              << vel << '\t'
              << h_avg << '\t'
              << barometer.temp << '\t'
              << barometer.pres << '\n';

          Serial.write(debug_buffer.c_str(), debug_buffer.length());
      }

      //log_file.flush();

      if (fabs(last_debug_h - h_avg) > 0.5) {
          if (h_avg > last_debug_h) {
              tone_sweep.start(t, 880, 880 * 4, 300);
          } else {
              tone_sweep.start(t, 880 * 4, 880, 300);
          }
      }

      last_debug_h = h_avg;
      last_debug_t = t;
    }

    if (0 && (t - last_metrics_t) > 100) {
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
