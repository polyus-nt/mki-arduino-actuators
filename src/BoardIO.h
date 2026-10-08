#pragma once
#include <Arduino.h>
#include "ActuatorApi.h"

// Optional header defines API_PWM_PINS {p0,...,p5}, API_TACH_PINS {...},
// API_PULSES_PER_REV (optional, default 2), and optionally API_LINK_* pins.
// NO guessed six-fan pinout is enabled by default.
#ifdef HARDWARE_API_BOARD_HEADER
#include HARDWARE_API_BOARD_HEADER
#endif
#ifndef API_LINK_MISO
#define API_LINK_MISO 2
#define API_LINK_MOSI 3
#define API_LINK_CLK 4
#define API_LINK_CS 5
#define API_LINK_IRQ 7
#endif

#if defined(API_PWM_PINS) != defined(API_TACH_PINS)
#error "both API_PWM_PINS and API_TACH_PINS must be defined"
#endif
#if defined(API_PWM_PINS) && defined(API_TACH_PINS)
#include <PinChangeInterrupt.h>
#ifndef API_PULSES_PER_REV
#define API_PULSES_PER_REV 2
#endif
static_assert(API_PULSES_PER_REV > 0, "tach pulses/revolution must be positive");
class BoardIO : public hardware_api::FanHardware {
 public:
  BoardIO() : valid_(false), lastMs_(0) {
    for (uint8_t i = 0; i < 6; ++i) { pulses_[i] = 0; rpm_[i] = 0; pwm_[i] = 0; }
  }
  bool setup() {
    instance_ = this;
    const uint8_t used[17] = {
      API_LINK_MISO, API_LINK_MOSI, API_LINK_CLK, API_LINK_CS, API_LINK_IRQ,
      pwmPins_[0], pwmPins_[1], pwmPins_[2], pwmPins_[3], pwmPins_[4], pwmPins_[5],
      tachPins_[0], tachPins_[1], tachPins_[2], tachPins_[3], tachPins_[4], tachPins_[5]
    };
    for (uint8_t i = 0; i < 17; ++i) {
      if (used[i] >= NUM_DIGITAL_PINS) return false;
      for (uint8_t j = 0; j < i; ++j) if (used[i] == used[j]) return false;
    }
    for (uint8_t i = 0; i < 6; ++i)
      if (!digitalPinHasPWM(pwmPins_[i]) || digitalPinToPCINT(tachPins_[i]) == NOT_AN_INTERRUPT)
        return false;
    for (uint8_t i = 0; i < 6; ++i) {
      pinMode(pwmPins_[i], OUTPUT); analogWrite(pwmPins_[i], 0);
      pinMode(tachPins_[i], INPUT_PULLUP);
    }
    void (*handlers[6])() = {pulse0, pulse1, pulse2, pulse3, pulse4, pulse5};
    for (uint8_t i = 0; i < 6; ++i)
      attachPCINT(digitalPinToPCINT(tachPins_[i]), handlers[i], FALLING);
    valid_ = true; lastMs_ = millis(); return true;
  }
  void poll(uint32_t now) {
    const uint32_t elapsed = static_cast<uint32_t>(now - lastMs_);
    if (!valid_ || elapsed < 1000UL) return;
    uint32_t snapshot[6];
    noInterrupts();
    for (uint8_t i = 0; i < 6; ++i) { snapshot[i] = pulses_[i]; pulses_[i] = 0; }
    interrupts();
    for (uint8_t i = 0; i < 6; ++i) {
      const uint64_t value = static_cast<uint64_t>(snapshot[i]) * 60000ULL /
                            (static_cast<uint64_t>(API_PULSES_PER_REV) * elapsed);
      rpm_[i] = value > UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(value);
    }
    lastMs_ = now;
  }
  bool available(uint8_t idx) const override { return valid_ && idx < 6; }
  bool writePwm(uint8_t idx, uint8_t pwm) override {
    if (!available(idx)) return false;
    analogWrite(pwmPins_[idx], pwm); pwm_[idx] = pwm; return true;
  }
  uint8_t readPwm(uint8_t idx) const override { return available(idx) ? pwm_[idx] : 0; }
  uint32_t readRpm(uint8_t idx) const override { return available(idx) ? rpm_[idx] : 0; }
  uint8_t randomPwm() override { return static_cast<uint8_t>(random(256)); }
 private:
  static void pulse(uint8_t idx) { if (instance_) ++instance_->pulses_[idx]; }
  static void pulse0() { pulse(0); } static void pulse1() { pulse(1); }
  static void pulse2() { pulse(2); } static void pulse3() { pulse(3); }
  static void pulse4() { pulse(4); } static void pulse5() { pulse(5); }
  static BoardIO* instance_;
  static const uint8_t pwmPins_[6], tachPins_[6];
  bool valid_;
  volatile uint32_t pulses_[6];
  uint32_t rpm_[6], lastMs_;
  uint8_t pwm_[6];
};
BoardIO* BoardIO::instance_ = nullptr;
const uint8_t BoardIO::pwmPins_[6] = API_PWM_PINS;
const uint8_t BoardIO::tachPins_[6] = API_TACH_PINS;
#else
// Explicitly unconfigured target: never claims that a physical PWM was applied.
class BoardIO : public hardware_api::FanHardware {
 public:
  bool setup() { return false; }
  void poll(uint32_t) {}
  bool available(uint8_t) const override { return false; }
  bool writePwm(uint8_t, uint8_t) override { return false; }
  uint8_t readPwm(uint8_t) const override { return 0; }
  uint32_t readRpm(uint8_t) const override { return 0; }
  uint8_t randomPwm() override { return static_cast<uint8_t>(random(256)); }
};
#endif
