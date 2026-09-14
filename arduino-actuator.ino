#include "AustereSerialMaster.h"
#include <EEPROM.h>

#include "./FansManager.h"
#include "control/ControlCore.h"
#include "control/PersistentConfig.h"

static const uint8_t PIN_MISO = 2;
static const uint8_t PIN_MOSI = 3;
static const uint8_t PIN_CLK = 4;
static const uint8_t PIN_CS = 5;
static const uint8_t PIN_IRQ = 7;

AustereSerialMaster link(PIN_MISO, PIN_MOSI, PIN_CLK, PIN_CS, PIN_IRQ);
FansManager fans;
cooling::ControlCore control;

static const uint8_t GET_ACTUATOR_STATE = 0x01;
static const uint8_t OK_CODE = 0x03;
static const uint8_t UPDATE_ENVIRONMENT = 0x06;
static const uint8_t SET_TARGET = 0x08;
static const uint8_t PROPOSE_PID = 0x09;
static const uint8_t PROPOSE_WEIGHTS = 0x0A;
static const uint8_t CONFIRM_CONFIG = 0x0B;
static const uint8_t ROLLBACK_CONFIG = 0x0C;
static const uint8_t SET_DOOR_POLICY = 0x0D;
static const uint8_t PROBE_FAN = 0x0E;
static const uint8_t CLEAR_EMERGENCY = 0x0F;
static const uint8_t GET_CONTROL_CONFIG = 0x10;
static const uint8_t MAX_PAYLOAD = 8;
static_assert(cooling::kPersistedConfigSize <= E2END + 1,
              "stable configuration must fit Arduino Uno EEPROM");

uint16_t readU16(const uint8_t* p) {
  return (static_cast<uint16_t>(p[0]) << 8) | p[1];
}

void sendU16(uint16_t value) {
  link.sendByte(static_cast<uint8_t>(value >> 8));
  link.sendByte(static_cast<uint8_t>(value));
}

uint16_t encodeCoefficient(float value) {
  if (value <= 0.0f) return 0;
  if (value >= 65.535f) return 65535;
  return static_cast<uint16_t>(lroundf(value * 1000.0f));
}

void saveStableConfig() {
  uint8_t stored[cooling::kPersistedConfigSize];
  if (cooling::encodePersistedConfig(control.stableConfig(), stored))
    EEPROM.put(0, stored);
}

void loadStableConfig() {
  uint8_t stored[cooling::kPersistedConfigSize];
  EEPROM.get(0, stored);
  cooling::ControlConfig config = control.stableConfig();
  if (cooling::decodePersistedConfig(stored, &config)) control.loadStableConfig(config);
}

uint8_t payloadLength(uint8_t command) {
  switch (command) {
    case GET_ACTUATOR_STATE:
    case CLEAR_EMERGENCY:
    case GET_CONTROL_CONFIG: return 0;
    case SET_TARGET:
    case PROBE_FAN: return 1;
    case SET_DOOR_POLICY:
    case CONFIRM_CONFIG:
    case ROLLBACK_CONFIG: return command == SET_DOOR_POLICY ? 4 : 2;
    case UPDATE_ENVIRONMENT: return 5;
    case PROPOSE_WEIGHTS: return 6;
    case PROPOSE_PID: return 8;
    default: return 0xFF;
  }
}

void sendActuatorState() {
  const cooling::Snapshot& s = control.snapshot();
  link.sendByte(s.targetC);
  link.sendByte(static_cast<uint8_t>(s.mode));
  link.sendByte(static_cast<uint8_t>(s.doorPolicy));
  sendU16(s.stableVersion);
  sendU16(s.candidateVersion);
  for (uint8_t i = 0; i < cooling::kFanCount; ++i) link.sendByte(s.pwm[i]);
  for (uint8_t i = 0; i < cooling::kFanCount; ++i) sendU16(s.rpm[i]);
  link.sendByte(s.failedMask);
  link.sendByte(OK_CODE);
}

void sendControlConfig() {
  const cooling::ControlConfig& config = control.activeConfig();
  const cooling::Snapshot& s = control.snapshot();
  sendU16(s.stableVersion);
  sendU16(s.candidateVersion);
  sendU16(encodeCoefficient(config.pid.kp));
  sendU16(encodeCoefficient(config.pid.ki));
  sendU16(encodeCoefficient(config.pid.kd));
  for (uint8_t i = 0; i < cooling::kFanCount; ++i) link.sendByte(config.weights[i]);
  link.sendByte(static_cast<uint8_t>(config.doorPolicy));
  link.sendByte(config.limitedPwm);
  link.sendByte(s.targetC);
  link.sendByte(OK_CODE);
}

void processCommand(uint8_t command, const uint8_t* payload, uint32_t nowMs) {
  cooling::Result result = cooling::INVALID_ARGUMENT;
  switch (command) {
    case GET_ACTUATOR_STATE: sendActuatorState(); return;
    case GET_CONTROL_CONFIG: sendControlConfig(); return;
    case UPDATE_ENVIRONMENT: {
      const int8_t whole = static_cast<int8_t>(payload[0]);
      const float fraction = roundf(payload[1] * 99.0f / 255.0f) / 100.0f;
      control.updateEnvironment(whole + fraction, (payload[2] & 0x01) != 0,
          (payload[2] & 0x02) != 0, payload[3] != 0, payload[4], nowMs);
      result = cooling::RESULT_OK;
      break;
    }
    case SET_TARGET: result = control.setTarget(payload[0]); break;
    case PROPOSE_PID: {
      cooling::PidConfig pid = control.stableConfig().pid;
      pid.kp = readU16(payload + 2) / 1000.0f;
      pid.ki = readU16(payload + 4) / 1000.0f;
      pid.kd = readU16(payload + 6) / 1000.0f;
      result = control.proposePid(readU16(payload), pid, nowMs);
      break;
    }
    case PROPOSE_WEIGHTS:
      result = control.proposeWeights(readU16(payload), payload + 2, nowMs);
      break;
    case CONFIRM_CONFIG: result = control.confirmConfig(readU16(payload)); break;
    case ROLLBACK_CONFIG: result = control.rollbackConfig(readU16(payload)); break;
    case SET_DOOR_POLICY:
      result = control.setDoorPolicy(readU16(payload),
          static_cast<cooling::DoorPolicy>(payload[2]), payload[3]);
      break;
    case PROBE_FAN: result = control.probeFan(payload[0], nowMs); break;
    case CLEAR_EMERGENCY:
      control.clearEmergency();
      result = cooling::RESULT_OK;
      break;
  }
  link.sendByte(static_cast<uint8_t>(result));
}

void setup() {
  Serial.begin(9600);
  link.setup();
  fans.setup();
  control.begin(millis());
  loadStableConfig();
}

void loop() {
  static uint8_t command = 0, expected = 0, received = 0;
  static uint8_t payload[MAX_PAYLOAD];
  static uint32_t commandStartedMs = 0;
  const uint32_t now = millis();

  uint8_t value;
  while (link.recvByte(&value)) {
    if (command == 0) {
      const uint8_t length = payloadLength(value);
      if (length == 0xFF) link.sendByte(static_cast<uint8_t>(cooling::INVALID_ARGUMENT));
      else if (length == 0) processCommand(value, payload, now);
      else {
        command = value;
        expected = length;
        received = 0;
        commandStartedMs = now;
      }
    } else {
      payload[received++] = value;
      if (received == expected) {
        processCommand(command, payload, now);
        command = 0;
      }
    }
  }
  if (command != 0 && now - commandStartedMs >= 2000UL) command = 0;

  static uint32_t lastControlMs = 0;
  if (now - lastControlMs >= 1000UL) {
    lastControlMs = now;
    uint16_t rpm[cooling::kFanCount];
    for (uint8_t i = 0; i < cooling::kFanCount; ++i) rpm[i] = fans.get_rpm(i);
    control.tick(rpm, now);
    for (uint8_t i = 0; i < cooling::kFanCount; ++i)
      fans.set_speed(i, control.snapshot().pwm[i]);
  }
  if (control.stableConfigDirty()) saveStableConfig();
}
