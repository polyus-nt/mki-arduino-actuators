#include <assert.h>
#include <stdio.h>
#include "../src/ActuatorApi.h"
using namespace hardware_api;

struct Hardware : FanHardware {
  uint8_t pwm[6] = {}, samples[8] = {230, 17, 255, 230, 80, 90, 100, 110};
  uint32_t rpm[6] = {};
  unsigned writes = 0, draws = 0;
  bool ready = true, fail = false;
  bool available(uint8_t idx) const override { return ready && idx < 6; }
  bool writePwm(uint8_t idx, uint8_t value) override {
    if (fail) return false;
    pwm[idx] = value; ++writes; return true;
  }
  uint8_t readPwm(uint8_t idx) const override { return pwm[idx]; }
  uint32_t readRpm(uint8_t idx) const override { return rpm[idx]; }
  uint8_t randomPwm() override { return samples[draws++ % 8]; }
};
static Frame request(uint8_t command, uint8_t idx, uint8_t value = 0) {
  Frame frame = {}; frame.version = hardware_api::kVersion; frame.kind = 1; frame.tx_id = 42;
  frame.node = 14; frame.command = command; frame.idx = idx; frame.data[0] = value;
  return frame;
}
static Frame checked(ActuatorApi& api, const Frame& frame, uint32_t now = 0) {
  Frame reply = api.handle(frame, now);
  assert(sameRequest(frame, reply));
  if (reply.status) assert(zeros(reply.data, 8));
  uint8_t bytes[hardware_api::kFrameSize]; encode(reply, bytes); Frame decoded;
  assert(decode(bytes, decoded)); assert(sameRequest(frame, decoded));
  return reply;
}
int main() {
  Hardware hardware; ActuatorApi api(hardware); assert(api.begin(0));
  assert(hardware.writes == 6 && api.mode() == 0 && api.activeIdx() == 255);
  assert(checked(api, request(kGetMode, 255)).data[1] == 255);
  assert(checked(api, request(kSetPwm, 2, 100)).data[0] == 100);
  assert(checked(api, request(kSetMode, 2, 1)).data[0] == 1);
  assert(hardware.pwm[2] == 230 && hardware.draws == 1);
  assert(api.tick(29999)); assert(hardware.pwm[2] == 230);
  checked(api, request(kSetPwm, 2, 100), 10000);
  checked(api, request(kSetMode, 2, 1), 20000);
  assert(hardware.draws == 1 && hardware.pwm[2] == 100);
  assert(api.tick(30000)); assert(hardware.pwm[2] == 17);
  assert(api.tick(35000)); assert(hardware.pwm[2] == 17);
  // Disconnect does not stop ticking the core.
  assert(api.tick(60000)); assert(hardware.pwm[2] == 255);
  checked(api, request(kSetMode, 4, 2), 65000);
  assert(api.activeIdx() == 4 && hardware.pwm[2] == 255 && hardware.draws == 3);
  assert(checked(api, request(kSetPwm, 4, 100), 65500).data[0] == 230);
  assert(checked(api, request(kGetPwm, 4)).data[0] == 230);
  assert(api.tick(95000)); assert(hardware.draws == 4);
  checked(api, request(kSetMode, 0, 0), 96000);
  assert(api.mode() == 0 && api.activeIdx() == 255 && hardware.pwm[4] == 230);
  assert(checked(api, request(kSetPwm, 4, 100)).data[0] == 100);
  for (uint8_t i = 0; i < 6; ++i) {
    assert(checked(api, request(kSetPwm, i, 0)).data[0] == 0);
    assert(checked(api, request(kSetPwm, i, 255)).data[0] == 255);
    hardware.rpm[i] = i == 5 ? UINT32_MAX : 771;
    assert(readU32(checked(api, request(kGetRpm, i)).data) == hardware.rpm[i]);
  }
  unsigned writes = hardware.writes, draws = hardware.draws;
  Frame invalid = request(kSetPwm, 6, 100);
  assert(checked(api, invalid).status == INVALID_INDEX);
  invalid = request(kSetMode, 0, 3);
  assert(checked(api, invalid).status == INVALID_ARGUMENT);
  invalid = request(kSetPwm, 0, 100); invalid.data[7] = 1;
  assert(checked(api, invalid).status == INVALID_ARGUMENT);
  invalid = request(kSetPwm, 0, 100); invalid.node = 13;
  assert(checked(api, invalid).status == INVALID_NODE);
  invalid = request(kSetPwm, 0, 100); invalid.version = 1;
  assert(checked(api, invalid).status == UNSUPPORTED_VERSION);
  assert(checked(api, request(kGetMode, 0)).status == INVALID_INDEX);
  assert(checked(api, request(kSetDoorLock, 0, 1)).status == UNKNOWN_COMMAND);
  assert(checked(api, request(0xff, 0)).status == UNKNOWN_COMMAND);
  assert(writes == hardware.writes && draws == hardware.draws);
  hardware.fail = true;
  assert(checked(api, request(kSetPwm, 0, 100)).status == INTERNAL_ERROR);
  hardware.fail = false; hardware.ready = false;
  assert(checked(api, request(kSetPwm, 0, 100)).status == NOT_IMPLEMENTED);
  assert(checked(api, request(kSetMode, 0, 1)).status == NOT_IMPLEMENTED);
  assert(checked(api, request(kGetMode, 255)).status == OK);
  hardware.ready = true;
  const uint32_t start = 0xfffff000UL;
  assert(api.begin(start)); checked(api, request(kSetMode, 5, 1), start);
  draws = hardware.draws;
  assert(api.tick(static_cast<uint32_t>(start + 29999UL)));
  assert(hardware.draws == draws);
  assert(api.tick(static_cast<uint32_t>(start + 30000UL)));
  assert(hardware.draws == draws + 1);
  // A long pause advances the schedule, not an unbounded loop of RNG/writes.
  assert(api.tick(static_cast<uint32_t>(start + 90001UL)));
  assert(hardware.draws == draws + 2);
  puts("actuator: modes, retention, wrap, six fans, errors passed");
}
