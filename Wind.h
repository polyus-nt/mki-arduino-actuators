#pragma once

#include <Arduino.h>

// Управление 12-вольтовым кулером с PWM и тахометром.
// Класс рассчитан на один экземпляр: обработчик прерывания Arduino не может
// напрямую вызывать нестатический метод объекта.
class Wind {
 public:
  Wind(uint8_t relayPin, uint8_t pwmPin, uint8_t tachPin,
       uint8_t pulsesPerRevolution = 2)
      : relayPin_(relayPin),
        pwmPin_(pwmPin),
        tachPin_(tachPin),
        pulsesPerRevolution_(pulsesPerRevolution),
        pulseCount_(0),
        lastMeasurementMs_(0),
        rpm_(0) {}

  void begin() {
    instance_ = this;

    pinMode(relayPin_, OUTPUT);
    digitalWrite(relayPin_, LOW);  // Реле с инверсной логикой: LOW = питание включено.
    pinMode(pwmPin_, OUTPUT);
    pinMode(tachPin_, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(tachPin_), onTachPulse, FALLING);
    lastMeasurementMs_ = millis();
  }

  void set_speed(uint8_t speed) {
    // Реле активно по LOW: при нулевой скорости снимаем питание с кулера.
    digitalWrite(relayPin_, speed == 0 ? HIGH : LOW);
    analogWrite(pwmPin_, speed);
  }

  // Возвращает последнее измеренное значение. Обновление происходит раз в
  // measurementIntervalMs (по умолчанию раз в секунду), без delay().
  uint16_t get_rpm(uint16_t measurementIntervalMs = 1000) {
    const unsigned long now = millis();
    const unsigned long elapsed = now - lastMeasurementMs_;
    if (elapsed < measurementIntervalMs || pulsesPerRevolution_ == 0) {
      return rpm_;
    }

    noInterrupts();
    const unsigned long pulses = pulseCount_;
    pulseCount_ = 0;
    interrupts();

    rpm_ = (uint16_t)((pulses * 60000UL) /
                      ((unsigned long)pulsesPerRevolution_ * elapsed));
    lastMeasurementMs_ = now;
    return rpm_;
  }

 private:
  static void onTachPulse() {
    if (instance_ != nullptr) {
      instance_->pulseCount_++;
    }
  }

  uint8_t relayPin_;
  uint8_t pwmPin_;
  uint8_t tachPin_;
  uint8_t pulsesPerRevolution_;
  volatile unsigned long pulseCount_;
  unsigned long lastMeasurementMs_;
  uint16_t rpm_;

  static Wind* instance_;
};

// Header используется только в одном .ino, поэтому определение допустимо здесь.
Wind* Wind::instance_ = nullptr;
