#include <Arduino.h>
#include <Wire.h>
#include <US.h>
#include "config.h"

US_Sensor us;

#define I2C_ADDR 0x50
#define SCALE 1000.0f // mm

volatile int16_t cached_xi = 0;
volatile int16_t cached_yi = 0;

void writeInt16LE(const int16_t v) {
  uint8_t b[2];
  b[0] = v & 0xFF;
  b[1] = (v >> 8) & 0xFF;
  Wire.write(b, 2);
}

void onRequest() {
  writeInt16LE(cached_xi);
  writeInt16LE(cached_yi);
}

void onReceive(int numBytes) {
  while (Wire.available()) {
    if (const uint8_t b = Wire.read(); b == 'c') {
      US_Sensor::calibrateSensor();
    }
  }
}

void setup() {
  delay(50);
  Serial.begin(115200);

  US_Sensor::initPins();
  US_Sensor::calibrateSensor();

  Wire.begin(I2C_ADDR);
  Wire.onRequest(onRequest);
  Wire.onReceive(onReceive);
}

void loop() {
  US_Sensor::update();

  cli();
  cached_xi = static_cast<int16_t>(US_Sensor::x_pos * SCALE);
  cached_yi = static_cast<int16_t>(US_Sensor::y_pos * SCALE);
  sei();

  Serial.print(US_Sensor::x_pos);
  Serial.print(", ");
  Serial.println(US_Sensor::y_pos);
  delay(sensorDelay);
}
