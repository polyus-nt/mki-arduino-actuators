#pragma once

#include <Arduino.h>
#include <OneWire.h>

// Reads a DS18B20/KY-001 connected to the specified 1-Wire pin.
// Returns NAN when no sensor is found on that pin.
float read_temperature(uint8_t pin) {
  OneWire ds(pin);
  byte data[9];
  byte address[8];

  if (!ds.search(address)) {
    ds.reset_search();
    return NAN;
  }

  ds.reset();
  ds.select(address);
  ds.write(0x44, 1);
  delay(100);

  ds.reset();
  ds.select(address);
  ds.write(0xBE);
  for (uint8_t i = 0; i < sizeof(data); ++i) {
    data[i] = ds.read();
  }

  int16_t raw = ((int16_t)data[1] << 8) | data[0];
  const byte config = data[4] & 0x60;
  if (config == 0x00) {
    raw &= ~7;
  } else if (config == 0x20) {
    raw &= ~3;
  } else if (config == 0x40) {
    raw &= ~1;
  }

  return (float)raw / 16.0f;
}
