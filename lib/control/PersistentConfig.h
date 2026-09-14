#pragma once

#include <stdint.h>

#include "ControlCore.h"

namespace cooling {

// EEPROM is an on-wire storage format, not a C++ struct.  Explicit offsets
// keep it identical on AVR and native builds and prevent padding surprises.
static const uint8_t kPersistedConfigSize = 19;
static const uint16_t kPersistedConfigMagic = 0xC001;

uint16_t persistedConfigChecksum(const uint8_t bytes[kPersistedConfigSize]);
bool encodePersistedConfig(const ControlConfig& config,
                           uint8_t bytes[kPersistedConfigSize]);
bool decodePersistedConfig(const uint8_t bytes[kPersistedConfigSize],
                           ControlConfig* config);

}  // namespace cooling
