#pragma once

#include <Arduino.h>
#include <PinChangeInterrupt.h>

class FansManager {
 public:
  static const uint8_t fans_count = 3;

  void setup() {
    instance_ = this;

    pinMode(relay_pin_, OUTPUT);
    for (uint8_t i = 0; i < fans_count; ++i) {
      pinMode(pwm_pins_[i], OUTPUT);
      pinMode(tach_pins_[i], INPUT_PULLUP);
    }

    attachPCINT(digitalPinToPCINT(tach_pins_[0]), countFan0, FALLING);
    attachPCINT(digitalPinToPCINT(tach_pins_[1]), countFan1, FALLING);
    attachPCINT(digitalPinToPCINT(tach_pins_[2]), countFan2, FALLING);

    last_measurement_ms_ = millis();
    set_speed(0);
  }

  // Устанавливает одинаковую PWM-скорость всем кулерам.
  void set_speed(uint8_t speed) {
    // Реле активно по LOW. При нулевой скорости питание кулеров отключено.
    digitalWrite(relay_pin_, speed == 0 ? HIGH : LOW);
    for (uint8_t i = 0; i < fans_count; ++i) {
      analogWrite(pwm_pins_[i], speed);
    }
  }

  // Возвращает последние измеренные обороты кулера с индексом idx (0..2).
  uint16_t get_rpm(uint8_t idx) {
    update_rpm();
    return idx < fans_count ? rpm_[idx] : 0;
  }

 private:
  static const uint8_t relay_pin_ = 8;
  static const uint8_t pulses_per_revolution_ = 2;
  static const uint16_t measurement_interval_ms_ = 1000;
  static const uint8_t pwm_pins_[fans_count];
  static const uint8_t tach_pins_[fans_count];

  void update_rpm() {
    const unsigned long now = millis();
    const unsigned long elapsed = now - last_measurement_ms_;
    if (elapsed < measurement_interval_ms_) {
      return;
    }

    unsigned long pulses[fans_count];
    noInterrupts();
    for (uint8_t i = 0; i < fans_count; ++i) {
      pulses[i] = pulse_count_[i];
      pulse_count_[i] = 0;
    }
    interrupts();

    for (uint8_t i = 0; i < fans_count; ++i) {
      rpm_[i] = (uint16_t)((pulses[i] * 60000UL) /
                           ((unsigned long)pulses_per_revolution_ * elapsed));
    }
    last_measurement_ms_ = now;
  }

  static void countFan0() { increment(0); }
  static void countFan1() { increment(1); }
  static void countFan2() { increment(2); }

  static void increment(uint8_t idx) {
    if (instance_ != nullptr) {
      instance_->pulse_count_[idx]++;
    }
  }

  volatile unsigned long pulse_count_[fans_count] = {0, 0, 0};
  uint16_t rpm_[fans_count] = {0, 0, 0};
  unsigned long last_measurement_ms_ = 0;

  static FansManager* instance_;
};

const uint8_t FansManager::pwm_pins_[FansManager::fans_count] = {9, 11, 2};
const uint8_t FansManager::tach_pins_[FansManager::fans_count] = {10, 12, 13};
FansManager* FansManager::instance_ = nullptr;
