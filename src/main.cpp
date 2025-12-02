#include <Arduino.h>
#include <Wire.h>
#include <US.h>
#include "config.h"

US_Sensor us;

#define I2C_ADDR 0x50
#define SCALE 1000.0f // mm

void writeInt16LE(const int16_t v) {
  uint8_t b[2];
  b[0] = v & 0xFF;
  b[1] = (v >> 8) & 0xFF;
  Wire.write(b, 2);
}

void onRequest() {
  const auto xi = static_cast<int16_t>(US_Sensor::x_pos * SCALE);
  const auto yi = static_cast<int16_t>(US_Sensor::y_pos * SCALE);

  writeInt16LE(xi);
  writeInt16LE(yi);
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
  delay(sensorDelay);
}
