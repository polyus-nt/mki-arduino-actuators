#include <cassert>
#include <cstdint>

#include "ControlCore.h"

int main() {
  using namespace cooling;
  const uint16_t rpm[kFanCount] = {1200, 1200, 1200, 1200};
  ControlCore core;
  core.begin(0);
  assert(core.setTarget(25) == RESULT_OK);
  core.updateEnvironment(30.0f, true, true, false, 1, 2000);
  core.tick(rpm, 2000);
  for (uint8_t i = 0; i < kFanCount; ++i) assert(core.snapshot().pwm[i] == 255);

  PidConfig pid = core.stableConfig().pid;
  pid.kp *= 1.1f;
  assert(core.proposePid(core.stableConfig().version, pid, 3000) == RESULT_OK);
  core.tick(rpm, 603001);
  assert(core.snapshot().candidateVersion == kNoCandidate);
  return 0;
}
