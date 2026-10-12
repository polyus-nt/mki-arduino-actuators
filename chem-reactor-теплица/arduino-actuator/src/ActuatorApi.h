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
};

// Химреактор: одна Arduino, node14, два вентилятора idx=0..1.
// Только прямое управление аппаратурой, без PID и режимов отказа.
// Потеря связи не меняет последние PWM и состояния реле.
class ActuatorApi {
 public:
  explicit ActuatorApi(FanHardware& hardware, uint8_t node = kActuatorNode,
                       uint8_t fanCount = kFanCount)
      : hardware_(hardware), node_(node), fanCount_(fanCount) {}
  bool begin(uint32_t) {
    // При запуске выключаем питание всех доступных вентиляторов и задаём PWM=0.
    bool success = true;
    for (uint8_t i = 0; i < fanCount_; ++i)
      if (hardware_.available(i) && !hardware_.writePwm(i, 0)) success = false;
    return success;
  }
  Frame handle(const Frame& request, uint32_t) {
    // Валидация выполняется до доступа к GPIO. Старые команды 0x40/0x41
    // возвращают UNKNOWN_COMMAND и не меняют PWM.
    const Status checked = validateRequest(request, node_, fanCount_);
    Frame reply = response(request, checked);
    if (checked != OK) return reply;
    if (!hardware_.available(request.idx)) return response(request, NOT_IMPLEMENTED);
    switch (request.command) {
      // GET_PWM (0x30): idx=0..1, data=0. Ответ: применённый PWM 0..255
      // в data[0], остальные байты нулевые. Это состояние выхода, не измерение.
      case kGetPwm: reply.data[0] = hardware_.readPwm(request.idx); break;
      // GET_RPM (0x31): idx=0..1, data=0. Ответ: измеренные обороты в минуту,
      // unsigned u32 big-endian в data[0..3], остальные байты нулевые.
      case kGetRpm: writeU32(reply.data, hardware_.readRpm(request.idx)); break;
      // SET_PWM (0x32): idx=0..1, data[0]=PWM 0..255, остальные data=0.
      // PWM=0 выключает реле, PWM>0 включает; уровни заданы в BoardConfig.h (HIGH-активное).
      // Переданное значение применяется без подмены и удерживается до новой
      // команды или перезапуска платы. Ошибка записи: INTERNAL_ERROR.
      // Успешный ответ содержит применённый PWM в data[0], остальные байты=0.
      case kSetPwm:
        if (!hardware_.writePwm(request.idx, request.data[0]))
          return response(request, INTERNAL_ERROR);
        reply.data[0] = hardware_.readPwm(request.idx);
        break;
      default: return response(request, UNKNOWN_COMMAND);
    }
    return reply;
  }
 private:
  FanHardware& hardware_;
  uint8_t node_, fanCount_;
};
}  // namespace hardware_api
