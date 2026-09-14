#include "PersistentConfig.h"

#include <math.h>
#include <string.h>

namespace cooling {
namespace {
enum Offset : uint8_t {
  MAGIC = 0, VERSION = 2, KP = 4, KI = 6, KD = 8, WEIGHTS = 10,
  POLICY = 14, LIMITED_PWM = 15, TARGET = 16, CHECKSUM = 17
};

void putU16(uint8_t* bytes, uint8_t at, uint16_t value) {
  bytes[at] = static_cast<uint8_t>(value >> 8);
  bytes[at + 1] = static_cast<uint8_t>(value);
}

uint16_t getU16(const uint8_t* bytes, uint8_t at) {
  return (static_cast<uint16_t>(bytes[at]) << 8) | bytes[at + 1];
}

uint16_t coefficient(float value) {
  if (!isfinite(value) || value < 0.0f) return 0xFFFF;
  if (value > 65.535f) return 0xFFFF;
  return static_cast<uint16_t>(value * 1000.0f + 0.5f);
}
}  // namespace

uint16_t persistedConfigChecksum(const uint8_t bytes[kPersistedConfigSize]) {
  uint16_t checksum = 0xA55A;
  for (uint8_t i = 0; i < CHECKSUM; ++i)
    checksum = static_cast<uint16_t>((checksum << 5) | (checksum >> 11)) ^ bytes[i];
  return checksum;
}

bool encodePersistedConfig(const ControlConfig& config,
                           uint8_t bytes[kPersistedConfigSize]) {
  const uint16_t kp = coefficient(config.pid.kp);
  const uint16_t ki = coefficient(config.pid.ki);
  const uint16_t kd = coefficient(config.pid.kd);
  if (config.targetC > 60 || config.doorPolicy > CONTINUE || kp == 0xFFFF ||
      ki == 0xFFFF || kd == 0xFFFF)
    return false;
  memset(bytes, 0, kPersistedConfigSize);
  putU16(bytes, MAGIC, kPersistedConfigMagic);
  putU16(bytes, VERSION, config.version);
  putU16(bytes, KP, kp);
  putU16(bytes, KI, ki);
  putU16(bytes, KD, kd);
  for (uint8_t i = 0; i < kFanCount; ++i) bytes[WEIGHTS + i] = config.weights[i];
  bytes[POLICY] = static_cast<uint8_t>(config.doorPolicy);
  bytes[LIMITED_PWM] = config.limitedPwm;
  bytes[TARGET] = config.targetC;
  putU16(bytes, CHECKSUM, persistedConfigChecksum(bytes));
  return true;
}

bool decodePersistedConfig(const uint8_t bytes[kPersistedConfigSize],
                           ControlConfig* config) {
  if (config == 0 || getU16(bytes, MAGIC) != kPersistedConfigMagic ||
      getU16(bytes, CHECKSUM) != persistedConfigChecksum(bytes) ||
      bytes[TARGET] > 60 || bytes[POLICY] > CONTINUE)
    return false;
  uint16_t weightSum = 0;
  for (uint8_t i = 0; i < kFanCount; ++i) weightSum += bytes[WEIGHTS + i];
  if (weightSum == 0) return false;
  config->version = getU16(bytes, VERSION);
  config->pid.kp = getU16(bytes, KP) / 1000.0f;
  config->pid.ki = getU16(bytes, KI) / 1000.0f;
  config->pid.kd = getU16(bytes, KD) / 1000.0f;
  config->pid.derivativeFilterAlpha = 0.25f;
  for (uint8_t i = 0; i < kFanCount; ++i) config->weights[i] = bytes[WEIGHTS + i];
  config->doorPolicy = static_cast<DoorPolicy>(bytes[POLICY]);
  config->limitedPwm = bytes[LIMITED_PWM];
  config->targetC = bytes[TARGET];
  return true;
}

}  // namespace cooling
