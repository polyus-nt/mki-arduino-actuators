#pragma once
// Edit this tab in Arduino IDE after agreeing the real wiring.
// Current Austere defaults: MISO=2, MOSI=3, CLK=4, CS=5, IRQ=7.
// To relocate the link, define ALL five API_LINK_* constants together.
// Six PWM pins and six tach pins must be unique, not overlap the link,
// and support hardware PWM / PinChangeInterrupt respectively.
// Install PinChangeInterrupt 1.2.9 in Library Manager before enabling fan pins.
// #define API_PWM_PINS { ... six pins ... }
// #define API_TACH_PINS { ... six pins ... }
// #define API_PULSES_PER_REV 2
// Until these arrays are defined, fan requests return NOT_IMPLEMENTED.
