#pragma once
// COMPILE CHECK ONLY: deliberate conflict with Austere MOSI/CS.
// BoardIO::setup must reject it; never flash this test-only profile.
#define API_PWM_PINS {3, 5, 6, 9}
#define API_TACH_PINS {8, 12, 13, 18}
#define API_RELAY_PINS {14, 15, 16, 17}
