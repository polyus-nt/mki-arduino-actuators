#include "ControlCore.h"

#include <math.h>
#include <string.h>

namespace cooling {

namespace {

bool elapsed(uint32_t now, uint32_t since, uint32_t duration) {
  return static_cast<uint32_t>(now - since) >= duration;
}

float clamp01(float value) {
  if (value < 0.0f) return 0.0f;
  if (value > 1.0f) return 1.0f;
  return value;
}

float relativeChange(float before, float after) {
  if (before == 0.0f) return after == 0.0f ? 0.0f : INFINITY;
  return fabsf(after - before) / fabsf(before);
}

}  // namespace

ControlCore::ControlCore()
    : hasCandidate_(false),
      candidateDeadlineMs_(0),
      stableDirty_(false),
      integral_(0.0f),
      previousError_(0.0f),
      filteredDerivative_(0.0f),
      lastPidMs_(0),
      lastFreshTemperatureMs_(0),
      hasFreshTemperature_(false),
      emergencyLatched_(false),
      lastDemand_(0.0f),
      probingFan_(-1),
      probeEndsMs_(0) {
  stable_.version = 1;
  stable_.pid.kp = 0.20f;
  stable_.pid.ki = 0.01f;
  stable_.pid.kd = 0.0f;
  stable_.pid.derivativeFilterAlpha = 0.25f;
  stable_.doorPolicy = FANS_OFF;
  stable_.limitedPwm = 128;
  stable_.targetC = 25;
  for (uint8_t i = 0; i < kFanCount; ++i) stable_.weights[i] = 1;
  candidate_ = stable_;

  memset(&snapshot_, 0, sizeof(snapshot_));
  snapshot_.targetC = stable_.targetC;
  snapshot_.mode = EMERGENCY;
  snapshot_.doorPolicy = stable_.doorPolicy;
  snapshot_.targetC = stable_.targetC;
  snapshot_.stableVersion = stable_.version;
  snapshot_.candidateVersion = kNoCandidate;
  memset(poweredSinceMs_, 0, sizeof(poweredSinceMs_));
  memset(zeroSamples_, 0, sizeof(zeroSamples_));
  memset(lastProbeMs_, 0, sizeof(lastProbeMs_));
}

void ControlCore::begin(uint32_t nowMs) {
  lastPidMs_ = nowMs;
  lastFreshTemperatureMs_ = nowMs;
}

void ControlCore::loadStableConfig(const ControlConfig& config) {
  stable_ = config;
  candidate_ = stable_;
  hasCandidate_ = false;
  snapshot_.stableVersion = stable_.version;
  snapshot_.candidateVersion = kNoCandidate;
  snapshot_.doorPolicy = stable_.doorPolicy;
  integral_ = 0.0f;
  filteredDerivative_ = 0.0f;
}

const ControlConfig& ControlCore::activeConfig() const {
  return hasCandidate_ ? candidate_ : stable_;
}

bool ControlCore::stableConfigDirty() {
  const bool dirty = stableDirty_;
  stableDirty_ = false;
  return dirty;
}

void ControlCore::updateEnvironment(float temperatureC, bool valid, bool fresh,
                                    bool doorOpen, uint8_t doorStage,
                                    uint32_t nowMs) {
  snapshot_.temperatureValid = valid;
  snapshot_.temperatureFresh = valid && fresh;
  snapshot_.doorOpen = doorOpen;
  snapshot_.doorStage = doorStage;
  if (!valid || !fresh || !isfinite(temperatureC)) return;

  snapshot_.temperatureC = temperatureC;
  lastFreshTemperatureMs_ = nowMs;
  hasFreshTemperature_ = true;

  // LIMITED retains the normal PID meaning: it calculates the same cooling
  // demand as a closed door and only caps the resulting per-fan command in
  // tick().  FANS_OFF intentionally freezes the controller while cooling is
  // disabled, avoiding wind-up during a door-open interval.
  if (!snapshot_.doorOpen || activeConfig().doorPolicy != FANS_OFF) {
    lastDemand_ = updatePid(temperatureC, nowMs);
    allocate(lastDemand_);
  }
}

Result ControlCore::setTarget(uint8_t targetC) {
  if (targetC > 60) return INVALID_ARGUMENT;
  if (hasCandidate_) return BUSY;
  if (targetC == stable_.targetC) return RESULT_OK;
  snapshot_.targetC = targetC;
  stable_.targetC = targetC;
  stable_.version = static_cast<uint16_t>(stable_.version + 1);
  snapshot_.stableVersion = stable_.version;
  stableDirty_ = true;
  integral_ = 0.0f;
  filteredDerivative_ = 0.0f;
  return RESULT_OK;
}

Result ControlCore::setDoorPolicy(uint16_t expectedVersion, DoorPolicy policy,
                                  uint8_t limitedPwm) {
  if (policy != FANS_OFF && policy != LIMITED && policy != CONTINUE)
    return INVALID_ARGUMENT;
  if (hasCandidate_) return BUSY;
  if (expectedVersion != stable_.version) return VERSION_CONFLICT;
  if (stable_.doorPolicy == policy && stable_.limitedPwm == limitedPwm)
    return RESULT_OK;
  stable_.doorPolicy = policy;
  stable_.limitedPwm = limitedPwm;
  stable_.version = static_cast<uint16_t>(stable_.version + 1);
  snapshot_.stableVersion = stable_.version;
  snapshot_.doorPolicy = policy;
  stableDirty_ = true;
  return RESULT_OK;
}

bool ControlCore::isFinitePid(const PidConfig& pid) const {
  return isfinite(pid.kp) && isfinite(pid.ki) && isfinite(pid.kd) &&
         pid.kp >= 0.0f && pid.ki >= 0.0f && pid.kd >= 0.0f;
}

bool ControlCore::validPidChange(const PidConfig& next) const {
  if (!isFinitePid(next)) return false;
  const PidConfig& old = stable_.pid;
  uint8_t changed = 0;
  if (next.kp != old.kp) ++changed;
  if (next.ki != old.ki) ++changed;
  if (next.kd != old.kd) ++changed;
  if (changed != 1) return false;
  if (old.kd == 0.0f && next.kd > 0.0f) return next.kd <= 0.1f;
  if (next.kp != old.kp && relativeChange(old.kp, next.kp) > 0.1001f)
    return false;
  if (next.ki != old.ki && relativeChange(old.ki, next.ki) > 0.1001f)
    return false;
  if (next.kd != old.kd && relativeChange(old.kd, next.kd) > 0.1001f)
    return false;
  return true;
}

Result ControlCore::proposePid(uint16_t expectedVersion, const PidConfig& pid,
                               uint32_t nowMs) {
  if (hasCandidate_) return BUSY;
  if (expectedVersion != stable_.version) return VERSION_CONFLICT;
  if (!validPidChange(pid)) return INVALID_ARGUMENT;
  candidate_ = stable_;
  candidate_.pid = pid;
  candidate_.version = static_cast<uint16_t>(stable_.version + 1);
  hasCandidate_ = true;
  candidateDeadlineMs_ = nowMs + kCandidateTimeoutMs;
  snapshot_.candidateVersion = candidate_.version;
  return RESULT_OK;
}

Result ControlCore::proposeWeights(uint16_t expectedVersion,
                                   const uint8_t weights[kFanCount],
                                   uint32_t nowMs) {
  if (hasCandidate_) return BUSY;
  if (expectedVersion != stable_.version) return VERSION_CONFLICT;
  uint16_t sum = 0;
  for (uint8_t i = 0; i < kFanCount; ++i) sum += weights[i];
  if (sum == 0) return INVALID_ARGUMENT;
  candidate_ = stable_;
  memcpy(candidate_.weights, weights, kFanCount);
  candidate_.version = static_cast<uint16_t>(stable_.version + 1);
  hasCandidate_ = true;
  candidateDeadlineMs_ = nowMs + kCandidateTimeoutMs;
  snapshot_.candidateVersion = candidate_.version;
  return RESULT_OK;
}

Result ControlCore::confirmConfig(uint16_t candidateVersion) {
  if (!hasCandidate_) return INVALID_ARGUMENT;
  if (candidateVersion != candidate_.version) return VERSION_CONFLICT;
  stable_ = candidate_;
  hasCandidate_ = false;
  snapshot_.stableVersion = stable_.version;
  snapshot_.candidateVersion = kNoCandidate;
  stableDirty_ = true;
  return RESULT_OK;
}

Result ControlCore::rollbackConfig(uint16_t candidateVersion) {
  if (!hasCandidate_) return INVALID_ARGUMENT;
  if (candidateVersion != candidate_.version) return VERSION_CONFLICT;
  candidate_ = stable_;
  hasCandidate_ = false;
  snapshot_.candidateVersion = kNoCandidate;
  integral_ = 0.0f;
  filteredDerivative_ = 0.0f;
  return RESULT_OK;
}

Result ControlCore::probeFan(uint8_t fanIndex, uint32_t nowMs) {
  if (fanIndex >= kFanCount) return INVALID_ARGUMENT;
  if (snapshot_.mode == EMERGENCY) return DENIED;
  if ((snapshot_.failedMask & (1U << fanIndex)) == 0) return INVALID_ARGUMENT;
  if (probingFan_ >= 0) return BUSY;
  if (lastProbeMs_[fanIndex] != 0 &&
      !elapsed(nowMs, lastProbeMs_[fanIndex], kProbeCooldownMs))
    return BUSY;
  probingFan_ = static_cast<int8_t>(fanIndex);
  probeEndsMs_ = nowMs + kProbeDurationMs;
  lastProbeMs_[fanIndex] = nowMs;
  return RESULT_OK;
}

void ControlCore::clearEmergency() { emergencyLatched_ = false; }

void ControlCore::expireCandidate(uint32_t nowMs) {
  if (hasCandidate_ && static_cast<int32_t>(nowMs - candidateDeadlineMs_) >= 0) {
    candidate_ = stable_;
    hasCandidate_ = false;
    snapshot_.candidateVersion = kNoCandidate;
    integral_ = 0.0f;
    filteredDerivative_ = 0.0f;
  }
}

bool ControlCore::emergencyCondition(uint32_t nowMs) const {
  if (!hasFreshTemperature_)
    return elapsed(nowMs, lastFreshTemperatureMs_, kSensorTimeoutMs);
  if (elapsed(nowMs, lastFreshTemperatureMs_, kSensorTimeoutMs)) return true;
  return snapshot_.temperatureC >= static_cast<float>(snapshot_.targetC) + 15.0f;
}

float ControlCore::updatePid(float temperatureC, uint32_t nowMs) {
  const float error = temperatureC - static_cast<float>(snapshot_.targetC);
  float dt = static_cast<float>(nowMs - lastPidMs_) / 1000.0f;
  if (dt <= 0.0f || dt > 10.0f) dt = 2.0f;
  lastPidMs_ = nowMs;

  const PidConfig& pid = activeConfig().pid;
  const float rawDerivative = (error - previousError_) / dt;
  filteredDerivative_ +=
      pid.derivativeFilterAlpha * (rawDerivative - filteredDerivative_);
  previousError_ = error;

  const float withoutIntegral = pid.kp * error + pid.kd * filteredDerivative_;
  const float proposedIntegral = integral_ + pid.ki * error * dt;
  const float proposedOutput = withoutIntegral + proposedIntegral;
  if ((proposedOutput < 1.0f || error < 0.0f) &&
      (proposedOutput > 0.0f || error > 0.0f)) {
    integral_ = proposedIntegral;
  }
  if (integral_ < 0.0f) integral_ = 0.0f;
  if (integral_ > 1.0f) integral_ = 1.0f;
  return clamp01(withoutIntegral + integral_);
}

void ControlCore::allocate(float demand) {
  const ControlConfig& config = activeConfig();
  uint16_t activeWeight = 0;
  for (uint8_t i = 0; i < kFanCount; ++i) {
    if ((snapshot_.failedMask & (1U << i)) == 0)
      activeWeight += config.weights[i];
  }
  if (activeWeight == 0) {
    memset(snapshot_.pwm, 0, sizeof(snapshot_.pwm));
    return;
  }
  for (uint8_t i = 0; i < kFanCount; ++i) {
    if ((snapshot_.failedMask & (1U << i)) != 0) {
      snapshot_.pwm[i] = 0;
      continue;
    }
    float command = demand * 255.0f * kFanCount * config.weights[i] /
                    static_cast<float>(activeWeight);
    if (command > 255.0f) command = 255.0f;
    snapshot_.pwm[i] = static_cast<uint8_t>(command + 0.5f);
  }
}

void ControlCore::applyEmergency() {
  for (uint8_t i = 0; i < kFanCount; ++i)
    snapshot_.pwm[i] =
        (snapshot_.failedMask & (1U << i)) == 0 ? 255 : 0;
}

void ControlCore::updateFanHealth(uint32_t nowMs) {
  for (uint8_t i = 0; i < kFanCount; ++i) {
    if (probingFan_ == static_cast<int8_t>(i)) continue;
    if ((snapshot_.failedMask & (1U << i)) != 0) continue;
    if (snapshot_.pwm[i] < kFailurePwmThreshold) {
      poweredSinceMs_[i] = 0;
      zeroSamples_[i] = 0;
      continue;
    }
    if (poweredSinceMs_[i] == 0) poweredSinceMs_[i] = nowMs;
    if (!elapsed(nowMs, poweredSinceMs_[i], kFanStartGraceMs)) continue;
    if (snapshot_.rpm[i] == 0) {
      if (zeroSamples_[i] < 255) ++zeroSamples_[i];
      if (zeroSamples_[i] >= kFailureSamples) {
        snapshot_.failedMask |= (1U << i);
        snapshot_.pwm[i] = 0;
      }
    } else {
      zeroSamples_[i] = 0;
    }
  }

  if (probingFan_ >= 0) {
    const uint8_t fan = static_cast<uint8_t>(probingFan_);
    snapshot_.pwm[fan] = 255;
    if (static_cast<int32_t>(nowMs - probeEndsMs_) >= 0) {
      if (snapshot_.rpm[fan] > 0) {
        snapshot_.failedMask &= ~(1U << fan);
        zeroSamples_[fan] = 0;
        poweredSinceMs_[fan] = nowMs;
      } else {
        // It remains failed and must not keep receiving the probe PWM.
        snapshot_.pwm[fan] = 0;
      }
      probingFan_ = -1;
    }
  }
}

void ControlCore::updateMode() {
  if (emergencyLatched_) {
    snapshot_.mode = EMERGENCY;
  } else if (snapshot_.doorOpen) {
    snapshot_.mode = DOOR_OPEN;
  } else if (snapshot_.failedMask != 0) {
    snapshot_.mode = DEGRADED;
  } else {
    snapshot_.mode = NORMAL;
  }
}

void ControlCore::tick(const uint16_t rpm[kFanCount], uint32_t nowMs) {
  memcpy(snapshot_.rpm, rpm, sizeof(snapshot_.rpm));
  expireCandidate(nowMs);
  if (emergencyCondition(nowMs)) emergencyLatched_ = true;
  updateMode();

  if (snapshot_.mode == EMERGENCY) {
    applyEmergency();
    // Emergency has priority over a probe. A later probe requires an
    // explicit request after the safety condition has actually cleared.
    probingFan_ = -1;
    return;
  } else if (snapshot_.doorOpen) {
    if (activeConfig().doorPolicy == FANS_OFF) {
      memset(snapshot_.pwm, 0, sizeof(snapshot_.pwm));
    } else if (activeConfig().doorPolicy == LIMITED) {
      for (uint8_t i = 0; i < kFanCount; ++i)
        if (snapshot_.pwm[i] > activeConfig().limitedPwm)
          snapshot_.pwm[i] = activeConfig().limitedPwm;
    }
  }

  const uint8_t failedBefore = snapshot_.failedMask;
  updateFanHealth(nowMs);
  // A newly latched fan must be excluded in this control tick, rather than
  // waiting for the next (normally two-second) sensor update.
  if (failedBefore != snapshot_.failedMask && snapshot_.mode != EMERGENCY) {
    allocate(lastDemand_);
    if (snapshot_.doorOpen && activeConfig().doorPolicy == LIMITED) {
      for (uint8_t i = 0; i < kFanCount; ++i)
        if (snapshot_.pwm[i] > activeConfig().limitedPwm)
          snapshot_.pwm[i] = activeConfig().limitedPwm;
    }
  }
  updateMode();
}

}  // namespace cooling
