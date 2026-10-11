#include <cassert>
#include <string>
#include <utility>
#include "Arduino.h"
#define F(text) text
#define HEX 16
#ifndef ACTUATOR_TEST_HEADER
#define ACTUATOR_TEST_HEADER "../arduino-actuator-1/src/ActuatorTestInput.h"
#endif
#include ACTUATOR_TEST_HEADER
int main() {
  ActuatorTestInput input; hardware_api::Frame frame={}; uint32_t now=0;
  auto line=[&](const std::string& s, uint8_t count=4) {
    ActuatorTestInput::Event result=ActuatorTestInput::NONE;
    for (char c:s) result=input.feed(c,now++,14,count,frame);
    return result;
  };
  for (const auto& item : {std::make_pair("pwm 0\n",hardware_api::kGetPwm),
                           std::make_pair("rpm 1\n",hardware_api::kGetRpm),
                           std::make_pair("set_pwm 1 255\n",hardware_api::kSetPwm)}) {
    assert(line(item.first)==ActuatorTestInput::REQUEST);
    assert(frame.command==item.second && frame.node==14);
  }
  assert(frame.data[0]==255);
  assert(line("set_pwm 1 0\n",2)==ActuatorTestInput::REQUEST && frame.data[0]==0);
  assert(line("help\n")==ActuatorTestInput::HELP);
  for (const char* bad: {"pwm 4\n","rpm -1\n","set_pwm 1 256\n","set_pwm 0 -1\n",
                        "pwm\n","pwm 0 junk\n","help 0\n","set_mode 2 0\n",
                        "set_pwm 0 99999999999\n","set_pwm 0 1.5\n"})
    assert(line(bad)==ActuatorTestInput::ERROR);
  assert(line("pwm 2\n",2)==ActuatorTestInput::ERROR);
  assert(line(std::string(40,'x')+"\n")==ActuatorTestInput::ERROR);
  line("set_"); now+=5000;
  assert(line("pwm 0\n")==ActuatorTestInput::REQUEST);
}
