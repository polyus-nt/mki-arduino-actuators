#pragma once
#include "common/HardwareProtocol.h"

// Только ручной TEST по USB Serial. Производственный API остаётся на Austere.
// Вход ограничен 31 символом, без String/heap; аргументы строго десятичные.
class ActuatorTestInput {
 public:
  enum Event { NONE, REQUEST, HELP, ERROR };
  ActuatorTestInput() : used_(0), overflow_(false), lastMs_(0) {}
  Event feed(char ch, uint32_t now, uint8_t node, uint8_t count,
             hardware_api::Frame& request) {
    if (static_cast<uint32_t>(now-lastMs_) >= 5000UL) { used_=0; overflow_=false; }
    lastMs_=now;
    if (ch!='\r' && ch!='\n') {
      if (ch<32 && ch!='\t') overflow_=true;
      if (used_ < sizeof(line_)-1) line_[used_++]=ch; else overflow_=true;
      return NONE;
    }
    if (overflow_) { used_=0; overflow_=false; return ERROR; }
    if (!used_) return NONE;
    line_[used_]=0; used_=0;
    char* p=line_; skip(p);
    char* name=p; while (*p && !space(*p)) ++p;
    if (*p) *p++=0;
    skip(p);
    if (!strcmp(name,"help")) return *p ? ERROR : HELP;
    uint8_t command=0;
    if (!strcmp(name,"pwm") || !strcmp(name,"get_pwm")) command=hardware_api::kGetPwm;
    else if (!strcmp(name,"rpm") || !strcmp(name,"get_rpm")) command=hardware_api::kGetRpm;
    else if (!strcmp(name,"set_pwm")) command=hardware_api::kSetPwm;
    else return ERROR;
    uint8_t idx=0, pwm=0;
    if (!number(p,idx) || idx>=count) return ERROR;
    if (command==hardware_api::kSetPwm && !number(p,pwm)) return ERROR;
    skip(p); if (*p) return ERROR;
    request=hardware_api::Frame{}; request.version=hardware_api::kVersion;
    request.kind=hardware_api::kRequest; request.node=node; request.idx=idx;
    request.command=command; request.data[0]=pwm; request.tx_id=1; request.session_id=1;
    return REQUEST;
  }
 private:
  static bool space(char c) { return c==' ' || c=='\t'; }
  static void skip(char*& p) { while (space(*p)) ++p; }
  static bool number(char*& p, uint8_t& result) {
    skip(p); if (*p<'0' || *p>'9') return false;
    uint16_t value=0;
    while (*p>='0' && *p<='9') {
      value=value*10+static_cast<uint8_t>(*p++-'0');
      if (value>255) return false;
    }
    if (*p && !space(*p)) return false;
    result=static_cast<uint8_t>(value); return true;
  }
  char line_[32]; uint8_t used_; bool overflow_; uint32_t lastMs_;
};

template<class Output>
void printActuatorReply(Output& out, const hardware_api::Frame& reply) {
  using namespace hardware_api;
  out.print(F("idx=")); out.print(reply.idx);
  out.print(F(" status=")); out.println(reply.status);
  if (reply.status==OK) {
    out.print(reply.command==kGetRpm ? F("RPM=") : F("Applied PWM="));
    if (reply.command==kGetRpm) out.println(readU32(reply.data));
    else out.println(reply.data[0]);
  }
  uint8_t bytes[kFrameSize]; encode(reply,bytes);
  out.print(F("TX HEX:"));
  for (uint8_t i=0;i<kFrameSize;++i) {
    out.print(' '); if (bytes[i]<16) out.print('0'); out.print(bytes[i],HEX);
  }
  out.println();
}
