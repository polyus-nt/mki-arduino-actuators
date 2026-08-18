#pragma once

#include <Arduino.h>

// Configure a separate input for each reed switch.
void setup_gerkon(uint8_t pin) {
  pinMode(pin, INPUT);
}

// Keeps the original KY-025 threshold logic for the specified analog pin.
int read_gerkon(uint8_t pin) {
  const int analogVal = analogRead(pin);
  Serial.print("Gerkon Analog: ");
  Serial.println(analogVal);
  return analogVal > 10 ? 0 : 1;
}
