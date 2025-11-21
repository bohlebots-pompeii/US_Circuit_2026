#include <Arduino.h>
#include <US.h>

US_Sensor us;

void setup() {
  Serial.begin(115200);
  US_Sensor::initPins();
  delay(50);
}

void loop() {
  auto data = US_Sensor::getData();  // one measurement per cycle
  for (size_t i = 0; i < data.size(); ++i) {
    Serial.print("US ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(data[i]);
    Serial.println(" cm");
  }
  Serial.println("---");
  delay(200); // give sensors breathing room
}
