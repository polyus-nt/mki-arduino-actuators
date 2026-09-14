// Legacy sketch retained while the portable control core is integrated.
// Плата ардуино, которая получает данные с 
// Датчик температуры KY-001
// Датчик Холла KY-024
// Датчик Геркона KY-025

#include "AustereSerialMaster.h"
#include "./Led.h"
#include "./FansManager.h"

// Pick any digital pins per spec.
static const uint8_t PIN_MISO = 2;
static const uint8_t PIN_MOSI = 3;
static const uint8_t PIN_CLK  = 4;
static const uint8_t PIN_CS   = 5;
static const uint8_t PIN_IRQ  = 7;

AustereSerialMaster link(PIN_MISO, PIN_MOSI, PIN_CLK, PIN_CS, PIN_IRQ);

static const uint8_t POLLING_CODE = 1;
static const uint8_t ACTUATOR_CODE = 2;
static const uint8_t OK_CODE = 3;
static const uint8_t SEND_CODE = 4;
static const uint8_t TEMPERATURE_WHOLE_CODE = 6;
static const uint8_t TEMPERATURE_FLOAT_CODE = 7;
static const uint8_t ETALON_CODE = 8;
static const uint16_t MAX_FAN_RPM = 5000;
static const uint8_t MAX_FAN_PWM = 150;
static const float FAN_FULL_SPEED_DIFFERENCE_C = 5.0f;
static const bool DEBUG = false;
static const uint8_t FAN_TEST_PWM = 150;
static const unsigned long FAN_TEST_REPORT_INTERVAL_MS = 5000;

// static const LED led = LED(A3);
FansManager fans;
float temperature = 0;
float etalon_temperature = 0;
uint16_t fan_rpm = 0;

// Format: [int8_t whole part][uint8_t fraction normalized to 0..255].
// The fraction represents hundredths of a degree (0..99).
float temperatureFromBytes(uint8_t wholeByte, uint8_t fractionByte) {
  const int8_t wholePart = (int8_t)wholeByte;
  const float hundredths = (float)fractionByte * 99.0f / 255.0f;
  return (float)wholePart + hundredths / 100.0f;
}

void updateFanSpeed() {
  const float difference = temperature - etalon_temperature;
  // +5 °C относительно эталона = 100% разрешённой скорости (PWM 150).
  if (difference < 0) {
    fans.set_speed(0);
    return;
  }
  const uint8_t speed = (uint8_t)constrain(
      lroundf(difference * MAX_FAN_PWM / FAN_FULL_SPEED_DIFFERENCE_C),
      0, MAX_FAN_PWM);
  fans.set_speed(speed);
}

void sendTemperature(float value) {
  value = constrain(value, -128.0f, 127.99f);

  // floorf preserves the sign for values between -1.00 and 0.00.
  int8_t wholePart = (int8_t)floorf(value);
  uint8_t hundredths = (uint8_t)lroundf((value - wholePart) * 100.0f);

  // For example, 23.999 becomes 24.00 after rounding to hundredths.
  if (hundredths == 100) {
    wholePart++;
    hundredths = 0;
  }

  const uint8_t normalizedFraction =
      (uint8_t)lroundf((float)hundredths * 255.0f / 99.0f);

  link.sendByte((uint8_t)wholePart);
  link.sendByte(normalizedFraction);
}

void updateAndPrintFanRpm() {
  static unsigned long lastReportMs = 0;
  const unsigned long now = millis();
  if (now - lastReportMs < 1000) {
    return;
  }
  lastReportMs = now;

  for (uint8_t i = 0; i < fans.fans_count; ++i) {
    const uint16_t rpm = fans.get_rpm(i);
    if (i == 0) {
      fan_rpm = rpm;
    }
    // Serial.print("Fan ");
    // Serial.print(i + 1);
    // Serial.print(": ");
    // Serial.print(rpm);
    // Serial.println(" RPM");
  }
}

// Включает все кулеры и раз в 5 секунд выводит их обороты.
void testFanAvailability() {
  static bool testStarted = false;
  static unsigned long lastReportMs = 0;
  const unsigned long now = millis();

  if (!testStarted) {
    Serial.println("Fan test started");
    testStarted = true;
  }
  fans.set_speed(50);
  lastReportMs = now;

  for (uint8_t i = 0; i < fans.fans_count; ++i) {
    Serial.print("Fan ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(fans.get_rpm(i));
    Serial.println(" RPM");
  }

  delay(1000);
}

// Передаёт RPM как два нормированных байта: 0x0000 = 0 RPM,
// 0xFFFF = MAX_FAN_RPM. Старший байт отправляется первым.
void sendCompressedRpm(uint16_t rpm) {
  rpm = constrain(rpm, 0, MAX_FAN_RPM);
  const uint16_t normalized = (uint16_t)(((unsigned long)rpm * 65535UL +
                                          MAX_FAN_RPM / 2) /
                                         MAX_FAN_RPM);
  uint8_t highByte = normalized >> 8;
  uint8_t lowByte = normalized & 0xFF;
  if (highByte == 3 || lowByte == 3) {
    Serial.println("УПРАВЛЯЮЩИЙ КОД");
    if (highByte == 3) {
      highByte = 255;
    }
    if (lowByte == 3) {
      lowByte = 255;
    }
  }
  // link.sendByte(normalized >> 8);
  // link.sendByte(normalized & 0xFF);
  // Serial.println(">> 8");
  // Serial.println(normalized >> 8);
  // if (normalized >> 8 > 255 || normalized & 0xFF > 255) {
  //   Serial.println("SDADASDASDAS");
  // }
  // Serial.println(">> FF");
  // Serial.println(normalized & 0xFF);
  link.sendByte(highByte);
  link.sendByte(lowByte);
}

void setup() {
  Serial.begin(9600);
  link.setup();
  Serial.println("AustereSerialMaster ready");
  fans.setup();
  Serial.println("Actuator ready");
}

void doUsefulWork() {
  // // Example TX once per second.
  // static uint32_t lastTxMs = 0;
  // static uint8_t counter = 0;
  // const uint32_t now = millis();
  // if (now - lastTxMs >= 1000) {
  //   lastTxMs = now;
  enum ReceiveState {
    WAIT_CODE,
    WAIT_TEMPERATURE_WHOLE,
    WAIT_TEMPERATURE_FLOAT_CODE,
    WAIT_TEMPERATURE_FRACTION,
    WAIT_ETALON
  };
  static ReceiveState receiveState = WAIT_CODE;
  static uint8_t receivedWholeByte = 0;
  uint8_t b;
  while (link.recvByte(&b)) {
    switch (receiveState) {
      case WAIT_CODE:
        if (b == ACTUATOR_CODE) {
          // зажечь светодиод
          // led.toggle();
          link.sendByte(OK_CODE);
          updateFanSpeed();
        } else if (b == POLLING_CODE) {
          for (uint8_t i = 0; i < fans.fans_count; ++i) {
            sendCompressedRpm(fans.get_rpm(i));
          }
          link.sendByte(OK_CODE);
        } else if (b == TEMPERATURE_WHOLE_CODE) {
          receiveState = WAIT_TEMPERATURE_WHOLE;
        } else if (b == ETALON_CODE) {
          receiveState = WAIT_ETALON;
        }
        break;
      case WAIT_TEMPERATURE_WHOLE:
        receivedWholeByte = b;
        receiveState = WAIT_TEMPERATURE_FLOAT_CODE;
        break;
      case WAIT_TEMPERATURE_FLOAT_CODE:
        if (b == TEMPERATURE_FLOAT_CODE) {
          receiveState = WAIT_TEMPERATURE_FRACTION;
        } else {
          // Пакет повреждён или неполон: синхронизируемся по следующему коду.
          receiveState = (b == TEMPERATURE_WHOLE_CODE)
              ? WAIT_TEMPERATURE_WHOLE
              : WAIT_CODE;
        }
        break;
      case WAIT_TEMPERATURE_FRACTION:
        temperature = temperatureFromBytes(receivedWholeByte, b);
        Serial.println(temperature);
        receiveState = WAIT_CODE;
        link.sendByte(OK_CODE);
        updateFanSpeed();
        break;
      case WAIT_ETALON:
        etalon_temperature = (float)(int8_t)b;
        receiveState = WAIT_CODE;
        updateFanSpeed();
        break;
    }
  }

  updateAndPrintFanRpm();
}

void loop() {
  if (DEBUG) {
    testFanAvailability();
  } else {
    doUsefulWork();
  }
}
