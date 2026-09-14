#include <unity.h>
#include <string.h>

#include "ControlCore.h"
#include "PersistentConfig.h"

using namespace cooling;

static uint16_t spinning[kFanCount] = {1200, 1200, 1200, 1200};

void test_full_cooling_at_plus_five() {
  ControlCore core;
  core.begin(0);
  TEST_ASSERT_EQUAL(RESULT_OK, core.setTarget(25));
  core.updateEnvironment(30.0f, true, true, false, 1, 2000);
  core.tick(spinning, 2000);
  for (uint8_t i = 0; i < kFanCount; ++i)
    TEST_ASSERT_EQUAL_UINT8(255, core.snapshot().pwm[i]);
}

void test_stale_sensor_enters_emergency() {
  ControlCore core;
  core.begin(0);
  core.updateEnvironment(25.0f, true, true, false, 1, 1000);
  core.tick(spinning, 1000);
  core.tick(spinning, 6001);
  TEST_ASSERT_EQUAL(EMERGENCY, core.snapshot().mode);
}

void test_start_without_measurement_enters_emergency_after_five_seconds() {
  ControlCore core;
  core.begin(1000);
  core.tick(spinning, 5999);
  TEST_ASSERT_NOT_EQUAL(EMERGENCY, core.snapshot().mode);
  core.tick(spinning, 6000);
  TEST_ASSERT_EQUAL(EMERGENCY, core.snapshot().mode);
}

void test_failed_fan_is_latched_and_redistributed() {
  ControlCore core;
  core.begin(0);
  core.updateEnvironment(27.0f, true, true, false, 1, 1000);
  core.tick(spinning, 1000);
  uint16_t oneStopped[kFanCount] = {0, 1200, 1200, 1200};
  for (uint32_t now = 2000; now <= 7000; now += 1000) {
    core.updateEnvironment(27.0f, true, true, false, 1, now);
    core.tick(oneStopped, now);
  }
  TEST_ASSERT_BITS(0x01, 0x01, core.snapshot().failedMask);
  TEST_ASSERT_EQUAL_UINT8(0, core.snapshot().pwm[0]);
  TEST_ASSERT_GREATER_THAN_UINT8(100, core.snapshot().pwm[1]);
}

void test_brief_zero_rpm_does_not_latch_failure() {
  ControlCore core;
  core.begin(0);
  uint16_t stopped[kFanCount] = {0, 1200, 1200, 1200};
  for (uint32_t now = 1000; now <= 5000; now += 1000) {
    core.updateEnvironment(27.0f, true, true, false, 1, now);
    core.tick(now == 4000 ? stopped : spinning, now);
  }
  TEST_ASSERT_EQUAL_UINT8(0, core.snapshot().failedMask);
}

void fail_fan_zero(ControlCore& core, uint32_t start) {
  uint16_t stopped[kFanCount] = {0, 1200, 1200, 1200};
  for (uint32_t now = start; now <= start + 6000; now += 1000) {
    core.updateEnvironment(27.0f, true, true, false, 1, now);
    core.tick(stopped, now);
  }
  TEST_ASSERT_BITS(0x01, 0x01, core.snapshot().failedMask);
}

void test_probe_success_failure_cooldown_and_emergency_denial() {
  ControlCore core;
  core.begin(0);
  fail_fan_zero(core, 1000);
  TEST_ASSERT_EQUAL(RESULT_OK, core.probeFan(0, 8000));
  core.tick(spinning, 11000);
  TEST_ASSERT_EQUAL_UINT8(0, core.snapshot().failedMask);
  // Re-latch it, then show an unsuccessful probe remains failed and is cooled
  // down before another request is accepted.
  fail_fan_zero(core, 12000);
  uint16_t stopped[kFanCount] = {0, 1200, 1200, 1200};
  core.updateEnvironment(27.0f, true, true, false, 1, 308000);
  core.tick(stopped, 308000);
  TEST_ASSERT_EQUAL(RESULT_OK, core.probeFan(0, 308000));
  core.tick(stopped, 311000);
  TEST_ASSERT_BITS(0x01, 0x01, core.snapshot().failedMask);
  TEST_ASSERT_EQUAL_UINT8(0, core.snapshot().pwm[0]);
  TEST_ASSERT_EQUAL(BUSY, core.probeFan(0, 311001));
  core.updateEnvironment(40.0f, true, true, false, 1, 312000);
  core.tick(stopped, 312000);
  TEST_ASSERT_EQUAL(EMERGENCY, core.snapshot().mode);
  TEST_ASSERT_EQUAL(DENIED, core.probeFan(0, 400000));
}

void test_door_policies_cap_pid_demand_and_emergency_wins() {
  ControlCore core;
  core.begin(0);
  const uint16_t version = core.stableConfig().version;
  TEST_ASSERT_EQUAL(RESULT_OK, core.setDoorPolicy(version, FANS_OFF, 100));
  core.updateEnvironment(30.0f, true, true, true, 1, 1000);
  core.tick(spinning, 1000);
  for (uint8_t i = 0; i < kFanCount; ++i) TEST_ASSERT_EQUAL_UINT8(0, core.snapshot().pwm[i]);
  TEST_ASSERT_EQUAL(RESULT_OK, core.setDoorPolicy(core.stableConfig().version, LIMITED, 100));
  core.updateEnvironment(30.0f, true, true, true, 1, 2000);
  core.tick(spinning, 2000);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, core.lastDemand());
  for (uint8_t i = 0; i < kFanCount; ++i) TEST_ASSERT_EQUAL_UINT8(100, core.snapshot().pwm[i]);
  TEST_ASSERT_EQUAL(RESULT_OK, core.setDoorPolicy(core.stableConfig().version, CONTINUE, 100));
  core.updateEnvironment(30.0f, true, true, true, 1, 3000);
  core.tick(spinning, 3000);
  for (uint8_t i = 0; i < kFanCount; ++i) TEST_ASSERT_EQUAL_UINT8(255, core.snapshot().pwm[i]);
  core.updateEnvironment(40.0f, true, true, true, 1, 4000);
  core.tick(spinning, 4000);
  TEST_ASSERT_EQUAL(EMERGENCY, core.snapshot().mode);
  for (uint8_t i = 0; i < kFanCount; ++i) TEST_ASSERT_EQUAL_UINT8(255, core.snapshot().pwm[i]);
}

void test_candidate_expires_to_stable() {
  ControlCore core;
  core.begin(0);
  PidConfig pid = core.stableConfig().pid;
  pid.kp *= 1.1f;
  TEST_ASSERT_EQUAL(RESULT_OK,
                    core.proposePid(core.stableConfig().version, pid, 1000));
  TEST_ASSERT_NOT_EQUAL(kNoCandidate, core.snapshot().candidateVersion);
  core.tick(spinning, 601001);
  TEST_ASSERT_EQUAL(kNoCandidate, core.snapshot().candidateVersion);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.20f, core.activeConfig().pid.kp);
}

void test_clear_emergency_does_not_disable_guard() {
  ControlCore core;
  core.begin(0);
  core.setTarget(25);
  core.updateEnvironment(40.0f, true, true, false, 1, 1000);
  core.tick(spinning, 1000);
  TEST_ASSERT_EQUAL(EMERGENCY, core.snapshot().mode);
  core.clearEmergency();
  core.tick(spinning, 2000);
  TEST_ASSERT_EQUAL(EMERGENCY, core.snapshot().mode);
}

void test_version_pid_candidate_confirm_rollback_and_timeout() {
  ControlCore core;
  core.begin(0);
  PidConfig changed = core.stableConfig().pid;
  changed.kp *= 1.1f;
  TEST_ASSERT_EQUAL(VERSION_CONFLICT, core.proposePid(99, changed, 1));
  changed.ki *= 1.1f;
  TEST_ASSERT_EQUAL(INVALID_ARGUMENT,
                    core.proposePid(core.stableConfig().version, changed, 1));
  changed = core.stableConfig().pid;
  changed.kp *= 1.2f;
  TEST_ASSERT_EQUAL(INVALID_ARGUMENT,
                    core.proposePid(core.stableConfig().version, changed, 1));
  changed.kp = core.stableConfig().pid.kp * 1.1f;
  TEST_ASSERT_EQUAL(RESULT_OK,
                    core.proposePid(core.stableConfig().version, changed, 1));
  const uint16_t candidate = core.snapshot().candidateVersion;
  TEST_ASSERT_EQUAL(VERSION_CONFLICT, core.confirmConfig(candidate + 1));
  TEST_ASSERT_EQUAL(RESULT_OK, core.confirmConfig(candidate));
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, changed.kp, core.stableConfig().pid.kp);
  uint8_t weights[kFanCount] = {2, 1, 1, 1};
  TEST_ASSERT_EQUAL(RESULT_OK,
                    core.proposeWeights(core.stableConfig().version, weights, 2));
  TEST_ASSERT_EQUAL(RESULT_OK, core.rollbackConfig(core.snapshot().candidateVersion));
  TEST_ASSERT_EQUAL(RESULT_OK,
                    core.proposeWeights(core.stableConfig().version, weights, 3));
  core.tick(spinning, 600003);
  TEST_ASSERT_EQUAL(kNoCandidate, core.snapshot().candidateVersion);
}

void test_millis_wrap_and_persisted_config_validation() {
  ControlCore core;
  core.begin(0xFFFFFF00UL);
  core.tick(spinning, 0x00001288UL);  // exactly 5 seconds after begin
  TEST_ASSERT_EQUAL(EMERGENCY, core.snapshot().mode);

  uint8_t bytes[kPersistedConfigSize];
  TEST_ASSERT_EQUAL(true, encodePersistedConfig(core.stableConfig(), bytes));
  TEST_ASSERT_EQUAL_UINT8(19, kPersistedConfigSize);
  ControlConfig restored = core.stableConfig();
  restored.targetC = 0;
  TEST_ASSERT_EQUAL(true, decodePersistedConfig(bytes, &restored));
  TEST_ASSERT_EQUAL_UINT8(core.stableConfig().targetC, restored.targetC);
  bytes[4] ^= 0x01;
  TEST_ASSERT_EQUAL(false, decodePersistedConfig(bytes, &restored));
  memset(bytes, 0xFF, sizeof(bytes));
  TEST_ASSERT_EQUAL(false, decodePersistedConfig(bytes, &restored));
}

void test_reboot_drops_ram_candidate_and_loads_stable_config() {
  ControlCore before;
  before.begin(0);
  uint8_t bytes[kPersistedConfigSize];
  TEST_ASSERT_EQUAL(true, encodePersistedConfig(before.stableConfig(), bytes));
  uint8_t weights[kFanCount] = {4, 1, 1, 1};
  TEST_ASSERT_EQUAL(RESULT_OK,
                    before.proposeWeights(before.stableConfig().version, weights, 1));
  ControlConfig restored;
  TEST_ASSERT_EQUAL(true, decodePersistedConfig(bytes, &restored));
  ControlCore after;
  after.begin(0);
  after.loadStableConfig(restored);
  TEST_ASSERT_EQUAL(kNoCandidate, after.snapshot().candidateVersion);
  TEST_ASSERT_EQUAL_UINT8(1, after.activeConfig().weights[0]);
}

void test_thirty_minute_simulation_holds_target_band_ninety_percent() {
  ControlCore core;
  core.begin(0);
  TEST_ASSERT_EQUAL(RESULT_OK, core.setTarget(30));
  float temperature = 25.0f;
  const float ambient = 23.0f;
  uint16_t rpm[kFanCount] = {0, 0, 0, 0};
  bool enteredBand = false;
  uint32_t remaining = 0;
  uint32_t inBand = 0;
  for (uint32_t second = 1; second <= 1800; ++second) {
    const uint32_t now = second * 1000UL;
    if ((second % 2) == 0)
      core.updateEnvironment(temperature, true, true, false, 1, now);
    for (uint8_t i = 0; i < kFanCount; ++i)
      rpm[i] = static_cast<uint16_t>(core.snapshot().pwm[i]) * 20U;
    core.tick(rpm, now);
    float coolingPower = 0.0f;
    for (uint8_t i = 0; i < kFanCount; ++i)
      coolingPower += core.snapshot().pwm[i] / 255.0f;
    const float noise = 0.04f * sinf(second * 0.37f);
    temperature += 0.045f - 0.003f * (temperature - ambient) -
                   0.012f * coolingPower + noise * 0.02f;
    if (!enteredBand && fabsf(temperature - 30.0f) <= 2.0f) enteredBand = true;
    if (enteredBand) {
      ++remaining;
      if (fabsf(temperature - 30.0f) <= 2.0f) ++inBand;
    }
  }
  TEST_ASSERT_TRUE(enteredBand);
  TEST_ASSERT_GREATER_OR_EQUAL_UINT32((remaining * 90U + 99U) / 100U, inBand);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_full_cooling_at_plus_five);
  RUN_TEST(test_stale_sensor_enters_emergency);
  RUN_TEST(test_start_without_measurement_enters_emergency_after_five_seconds);
  RUN_TEST(test_failed_fan_is_latched_and_redistributed);
  RUN_TEST(test_brief_zero_rpm_does_not_latch_failure);
  RUN_TEST(test_probe_success_failure_cooldown_and_emergency_denial);
  RUN_TEST(test_door_policies_cap_pid_demand_and_emergency_wins);
  RUN_TEST(test_candidate_expires_to_stable);
  RUN_TEST(test_clear_emergency_does_not_disable_guard);
  RUN_TEST(test_version_pid_candidate_confirm_rollback_and_timeout);
  RUN_TEST(test_millis_wrap_and_persisted_config_validation);
  RUN_TEST(test_reboot_drops_ram_candidate_and_loads_stable_config);
  RUN_TEST(test_thirty_minute_simulation_holds_target_band_ninety_percent);
  return UNITY_END();
}
