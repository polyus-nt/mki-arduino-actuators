#include <PinChangeInterrupt.h>

// Install the "PinChangeInterrupt" library through Arduino IDE Library Manager.
// Each tachometer wire must be connected to its own Arduino input.
const byte RELAY_PIN = 8;

const byte FAN_COUNT = 3;
const byte FAN_PWM_PINS[FAN_COUNT] = {9, 11, 2};
const byte TACH_PINS[FAN_COUNT] = {10, 12, 13};
const byte PULSES_PER_REV = 2;  // Most PC fans emit two tach pulses per revolution.
const unsigned long REPORT_INTERVAL_MS = 1000;

volatile unsigned long pulseCount[FAN_COUNT] = {0, 0, 0};
unsigned long lastReportMs = 0;

void countFan1() {
  pulseCount[0]++;
}

void countFan2() {
  pulseCount[1]++;
}

void countFan3() {
  pulseCount[2]++;
}

void setup() {
  // Uncomment these two lines if the relay must power the fans.
  // pinMode(RELAY_PIN, OUTPUT);
  // digitalWrite(RELAY_PIN, LOW);  // For an active-low relay module.

  Serial.begin(9600);
  for (byte i = 0; i < FAN_COUNT; i++) {
    pinMode(FAN_PWM_PINS[i], OUTPUT);
  }  

  for (byte i = 0; i < FAN_COUNT; i++) {
    pinMode(TACH_PINS[i], INPUT_PULLUP);
  }

  // These inputs use pin-change interrupts, so D2/D3 are not required.
  attachPCINT(digitalPinToPCINT(TACH_PINS[0]), countFan1, FALLING);
  attachPCINT(digitalPinToPCINT(TACH_PINS[1]), countFan2, FALLING);
  attachPCINT(digitalPinToPCINT(TACH_PINS[2]), countFan3, FALLING);
}

void loop() {
  // for (byte i = 0; i < FAN_COUNT; i++) {
  //   analogWrite(FAN_PWM_PINS[i], 50);
  // }
  analogWrite(FAN_PWM_PINS[0], 0);
  analogWrite(FAN_PWM_PINS[1], 0);  
  analogWrite(FAN_PWM_PINS[2], 255);
  unsigned long now = millis();
  unsigned long elapsedMs = now - lastReportMs;

  if (elapsedMs < REPORT_INTERVAL_MS) {
    return;
  }

  unsigned long pulses[FAN_COUNT];
  noInterrupts();
  for (byte i = 0; i < FAN_COUNT; i++) {
    pulses[i] = pulseCount[i];
    pulseCount[i] = 0;
  }
  interrupts();

  for (byte i = 0; i < FAN_COUNT; i++) {
    // RPM = pulses / pulses-per-revolution / elapsed-minutes.
    unsigned long rpm = (pulses[i] * 60000UL) /
                        (PULSES_PER_REV * elapsedMs);

    Serial.print("Fan ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(rpm);
    Serial.println(" RPM");
  }

  lastReportMs = now;
}
