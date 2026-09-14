#ifndef ARDUINO

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>

#include "ControlCore.h"

// Deterministic first-order thermal model for quick local experiments.
int main(int argc, char** argv) {
  const bool injectFailure = argc > 1 && std::string(argv[1]) == "fan-failure";
  cooling::ControlCore core;
  core.begin(0);
  core.setTarget(30);

  float temperature = 25.0f;
  const float ambient = 23.0f;
  uint16_t rpm[cooling::kFanCount] = {0, 0, 0, 0};
  std::cout << "second,temperature,mode,pwm1,pwm2,pwm3,pwm4,failed_mask\n";
  for (uint32_t second = 1; second <= 1800; ++second) {
    const uint32_t now = second * 1000UL;
    if (second % 2 == 0)
      core.updateEnvironment(temperature, true, true, false, 1, now);

    for (uint8_t i = 0; i < cooling::kFanCount; ++i) {
      const bool failed = injectFailure && i == 1 && second >= 600;
      rpm[i] = failed ? 0 : static_cast<uint16_t>(core.snapshot().pwm[i]) * 20U;
    }
    core.tick(rpm, now);

    float coolingPower = 0.0f;
    for (uint8_t i = 0; i < cooling::kFanCount; ++i)
      coolingPower += core.snapshot().pwm[i] / 255.0f;
    const float deterministicNoise = 0.04f * std::sin(second * 0.37f);
    temperature += 0.045f - 0.003f * (temperature - ambient) -
                   0.012f * coolingPower + deterministicNoise * 0.02f;

    if (second % 10 == 0) {
      const cooling::Snapshot& s = core.snapshot();
      std::cout << second << ',' << temperature << ',' << static_cast<int>(s.mode);
      for (uint8_t i = 0; i < cooling::kFanCount; ++i)
        std::cout << ',' << static_cast<int>(s.pwm[i]);
      std::cout << ',' << static_cast<int>(s.failedMask) << '\n';
    }
  }
  return EXIT_SUCCESS;
}

#endif
