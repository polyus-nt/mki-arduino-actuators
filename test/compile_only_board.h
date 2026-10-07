#pragma once
// COMPILE CHECK ONLY: this deliberately conflicts with the default Austere pins.
// BoardIO::setup must reject it; it is NOT a proposed physical six-fan pinout.
#define API_PWM_PINS {3, 5, 6, 9, 10, 11}
#define API_TACH_PINS {8, 12, 13, 14, 15, 16}
