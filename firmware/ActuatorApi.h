#pragma once
#include "common/HardwareProtocol.h"

namespace hardware_api {
class FanHardware {
 public:
  virtual ~FanHardware() {}
  virtual bool available(uint8_t idx) const = 0;
  virtual bool writePwm(uint8_t idx, uint8_t pwm) = 0;
  virtual uint8_t readPwm(uint8_t idx) const = 0;
  virtual uint32_t readRpm(uint8_t idx) const = 0;
  virtual uint8_t randomPwm() = 0;
};

// No PID, EEPROM, temperature decisions, locks or connection watchdog.
class ActuatorApi {
 public:
  explicit ActuatorApi(FanHardware& hardware)
      : hardware_(hardware), mode_(0), idx_(0xff), lastTickMs_(0) {}
  bool begin(uint32_t now) {
    mode_ = 0; idx_ = 0xff; lastTickMs_ = now;
    bool success = true;
    for (uint8_t i = 0; i < kFanCount; ++i)
      if (hardware_.available(i) && !hardware_.writePwm(i, 0)) success = false;
    return success;
  }
  bool tick(uint32_t now) {
    const uint32_t elapsed = static_cast<uint32_t>(now - lastTickMs_);
    if (mode_ != 1 || elapsed < 30000UL) return true;
    // Missed intervals do not cause an unbounded burst of output writes.
    lastTickMs_ += (elapsed / 30000UL) * 30000UL;
    return hardware_.writePwm(idx_, hardware_.randomPwm());
  }
  Frame handle(const Frame& request, uint32_t now) {
    const Status checked = validateRequest(request, kActuatorNode);
    Frame reply = response(request, checked);
    if (checked != OK) return reply;
    if (request.command == kGetMode) { modeResult(reply); return reply; }
    if (request.command == kSetMode) {
      const uint8_t mode = request.data[0];
      if (mode == 0) { mode_ = 0; idx_ = 0xff; modeResult(reply); return reply; }
      if (!hardware_.available(request.idx)) return response(request, NOT_IMPLEMENTED);
      if (mode_ == mode && idx_ == request.idx) { modeResult(reply); return reply; }
      // Disable the former scenario without restoring any previous PWM.
      mode_ = 0; idx_ = 0xff;
      if (mode == 1 && !hardware_.writePwm(request.idx, hardware_.randomPwm()))
        return response(request, INTERNAL_ERROR);
      mode_ = mode; idx_ = request.idx; lastTickMs_ = now;
      modeResult(reply); return reply;
    }
    if (!hardware_.available(request.idx)) return response(request, NOT_IMPLEMENTED);
    switch (request.command) {
      case kGetPwm: reply.data[0] = hardware_.readPwm(request.idx); break;
      case kGetRpm: writeU32(reply.data, hardware_.readRpm(request.idx)); break;
      case kSetPwm: {
        const uint8_t pwm = mode_ == 2 && idx_ == request.idx
            ? hardware_.randomPwm() : request.data[0];
        if (!hardware_.writePwm(request.idx, pwm)) return response(request, INTERNAL_ERROR);
        reply.data[0] = hardware_.readPwm(request.idx);
        break;
      }
      default: return response(request, UNKNOWN_COMMAND);
    }
    return reply;
  }
  uint8_t mode() const { return mode_; }
  uint8_t activeIdx() const { return idx_; }
 private:
  void modeResult(Frame& reply) const { reply.data[0] = mode_; reply.data[1] = idx_; }
  FanHardware& hardware_;
  uint8_t mode_, idx_;
  uint32_t lastTickMs_;
};
}  // namespace hardware_api
