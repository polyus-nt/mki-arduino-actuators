// датчик Холла

// #define KY_024_DO_PIN 13
// #define AO_PIN A0
static uint8_t ky24_do_pin = 13;

void setup_hall(int digital_pin) {
  ky24_do_pin = digital_pin;
  pinMode(ky24_do_pin, INPUT);
}

int read_hall() {
  int digitalVal = digitalRead(ky24_do_pin);
  // int analogVal = analogRead(AO_PIN);

  // Serial.print("KY-024 Digital: .");
  // Serial.println(digitalVal);
  // Serial.print(" | Analog: ");
  // Serial.println(analogVal);
  
  delay(500);
}