#pragma once

#include <stdint.h>

namespace cooling {

static const uint8_t kFanCount = 4;
static const uint16_t kNoCandidate = 0xFFFF;

enum Mode : uint8_t {
  NORMAL = 0,
  DOOR_OPEN = 1,
  DEGRADED = 2,
  EMERGENCY = 3,
};

enum DoorPolicy : uint8_t {
  FANS_OFF = 0,
  LIMITED = 1,
  CONTINUE = 2,
};

enum Result : uint8_t {
  RESULT_OK = 0x03,
  INVALID_ARGUMENT = 0xE0,
  VERSION_CONFLICT = 0xE1,
  DENIED = 0xE2,
  BUSY = 0xE3,
};

struct PidConfig {
  float kp;
  float ki;
  float kd;
  float derivativeFilterAlpha;
};

struct ControlConfig {
  uint16_t version;
  PidConfig pid;
  uint8_t weights[kFanCount];
  DoorPolicy doorPolicy;
  uint8_t limitedPwm;
  uint8_t targetC;
};

struct Snapshot {
  float temperatureC;
  uint8_t targetC;
  bool temperatureValid;
  bool temperatureFresh;
  bool doorOpen;
  uint8_t doorStage;
  Mode mode;
  DoorPolicy doorPolicy;
  uint16_t stableVersion;
  uint16_t candidateVersion;
  uint8_t pwm[kFanCount];
  uint16_t rpm[kFanCount];
  uint8_t failedMask;
};

class ControlCore {
 public:
  ControlCore();

  void begin(uint32_t nowMs);
  void loadStableConfig(const ControlConfig& config);
  void updateEnvironment(float temperatureC, bool valid, bool fresh,
                         bool doorOpen, uint8_t doorStage, uint32_t nowMs);
  void tick(const uint16_t rpm[kFanCount], uint32_t nowMs);

  Result setTarget(uint8_t targetC);
  Result setDoorPolicy(uint16_t expectedVersion, DoorPolicy policy,
                       uint8_t limitedPwm);
  Result proposePid(uint16_t expectedVersion, const PidConfig& pid,
                    uint32_t nowMs);
  Result proposeWeights(uint16_t expectedVersion,
                        const uint8_t weights[kFanCount], uint32_t nowMs);
  Result confirmConfig(uint16_t candidateVersion);
  Result rollbackConfig(uint16_t candidateVersion);
  Result probeFan(uint8_t fanIndex, uint32_t nowMs);
  void clearEmergency();

  const Snapshot& snapshot() const { return snapshot_; }
  const ControlConfig& stableConfig() const { return stable_; }
  const ControlConfig& activeConfig() const;
  bool stableConfigDirty();
  // Internal only: useful to the simulator/tests.  It is deliberately not a
  // telemetry field and must not be sent to the host.
  float lastDemand() const { return lastDemand_; }

 private:
  static const uint32_t kSensorTimeoutMs = 5000UL;
  static const uint32_t kCandidateTimeoutMs = 600000UL;
  static const uint32_t kFanStartGraceMs = 3000UL;
  static const uint32_t kProbeDurationMs = 3000UL;
  static const uint32_t kProbeCooldownMs = 300000UL;
  static const uint8_t kFailurePwmThreshold = 80;
  static const uint8_t kFailureSamples = 3;

  float updatePid(float temperatureC, uint32_t nowMs);
  void updateFanHealth(uint32_t nowMs);
  void allocate(float demand);
  void applyEmergency();
  void updateMode();
  void expireCandidate(uint32_t nowMs);
  bool emergencyCondition(uint32_t nowMs) const;
  bool isFinitePid(const PidConfig& pid) const;
  bool validPidChange(const PidConfig& next) const;

  ControlConfig stable_;
  ControlConfig candidate_;
  bool hasCandidate_;
  uint32_t candidateDeadlineMs_;
  bool stableDirty_;

  Snapshot snapshot_;
  float integral_;
  float previousError_;
  float filteredDerivative_;
  uint32_t lastPidMs_;
  uint32_t lastFreshTemperatureMs_;
  bool hasFreshTemperature_;
  bool emergencyLatched_;
  float lastDemand_;

  uint32_t poweredSinceMs_[kFanCount];
  uint8_t zeroSamples_[kFanCount];
  uint32_t lastProbeMs_[kFanCount];
  int8_t probingFan_;
  uint32_t probeEndsMs_;
};

}  // namespace cooling
