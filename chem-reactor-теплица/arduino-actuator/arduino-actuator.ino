#include <Arduino.h>
#include "BoardConfig.h"
#include "src/austere/AustereSerialMaster.h"
#include "src/BoardIO.h"
#include "src/LinkPins.h"

BoardIO board;
hardware_api::ActuatorApi api(board, API_LOCAL_NODE, API_FAN_COUNT);
AustereSerialMaster link(API_LINK_MISO, API_LINK_MOSI, API_LINK_CLK, API_LINK_CS, API_LINK_IRQ);
hardware_api::StreamParser parser;
hardware_api::ReplyBuffer replies;
uint16_t lastOverruns = 0;
bool linkReady = false;
#ifdef TEST
#include "src/ActuatorTestInput.h"
ActuatorTestInput serialInput;
#endif

void setup() {
  Serial.begin(9600);
  if (!validLinkPins()) { Serial.println(F("invalid Austere pin map")); return; }
  if (!board.setup()) Serial.println(F("fan pin map missing or invalid: outputs disabled"));
  if (!api.begin(millis())) Serial.println(F("initial PWM write failed"));
  linkReady = link.setup();
#ifdef TEST
  Serial.println(F("TEST: pwm idx | rpm idx | set_pwm idx 0..255 | help"));
  Serial.print(F("Local indices: 0..")); Serial.println(API_FAN_COUNT-1);
#endif
}

void loop() {
  if (!linkReady) return;
  board.poll(millis());
#ifdef TEST
  // Не смешиваем USB и Austere. Одна текстовая команда использует тот же API.
  if (Serial.available()>0) {
    hardware_api::Frame request;
    const ActuatorTestInput::Event event=serialInput.feed(
        static_cast<char>(Serial.read()), millis(), API_LOCAL_NODE, API_FAN_COUNT, request);
    if (event==ActuatorTestInput::REQUEST) printActuatorReply(Serial, api.handle(request,millis()));
    else if (event==ActuatorTestInput::HELP)
      Serial.println(F("pwm idx | rpm idx | set_pwm idx 0..255; LOCAL indices, not global"));
    else if (event==ActuatorTestInput::ERROR) Serial.println(F("Invalid command/index/PWM"));
  }
#endif
  parser.tick(millis());
  if (link.overrunCount() != lastOverruns) {
    lastOverruns = link.overrunCount(); parser.reset(); link.clearRx();
    Serial.println(F("Austere RX overflow: partial request discarded"));
  }
  // At most one blocking 9-bit Austere transfer on RX per iteration.
  // Do not execute another command while its predecessor still has an unsent reply.
  if (!replies.pending()) {
    uint8_t byte;
    hardware_api::Frame request;
    if (link.recvByte(&byte) && parser.feed(byte, millis(), request) &&
        request.kind == hardware_api::kRequest)
      replies.enqueue(api.handle(request, millis()));
  }
  if (replies.pending() && link.sendByte(replies.nextByte())) replies.accepted();
}
