// US.cpp
#include "US.h"
#include "config.h"
#include <Arduino.h>
#include <array>

void US_Sensor::initPins() {
    for (size_t i = 0; i < echoPins.size(); i++)
        pinMode(echoPins[i], INPUT);

    for (size_t i = 0; i < triggerPins.size(); i++) {
        pinMode(triggerPins[i], OUTPUT);
        digitalWrite(triggerPins[i], LOW);
    }
}

std::array<float, 4> US_Sensor::getData() {
    constexpr size_t N = echoPins.size();
    std::array<unsigned long, N> duration{};
    std::array<float, N> distances{};
    constexpr unsigned long timeout = 30000UL;  // 30 ms timeout
    constexpr unsigned long interSensorDelay = 50; // ms between sensors
    constexpr int maxRetries = 3;

    // Order: trigger GPIO5 first if it exists
    std::array<size_t, N> order = {0, 1, 2, 3};
    for (size_t i = 0; i < N; i++) {
        if (echoPins[i] == 5) {
            order[0] = i;
            break;
        }
    }

    for (size_t idx = 0; idx < N; idx++) {
        const size_t i = order[idx];

        // Ensure all triggers are LOW before firing this one
        for (size_t j = 0; j < N; j++) digitalWrite(triggerPins[j], LOW);
        delayMicroseconds(2);

        // Trigger this sensor (10us pulse)
        digitalWrite(triggerPins[i], HIGH);
        delayMicroseconds(10);
        digitalWrite(triggerPins[i], LOW);

        unsigned long dur = 0;
        for (int retry = 0; retry < maxRetries && dur == 0; retry++) {
            dur = pulseIn(echoPins[i], HIGH, timeout);
            if (dur == 0) delay(10);  // small wait before retry
        }
        duration[i] = dur;

        // Debug print
        Serial.print("idx=");
        Serial.print(i);
        Serial.print(" trig=");
        Serial.print(triggerPins[i]);
        Serial.print(" echo=");
        Serial.print(echoPins[i]);
        Serial.print(" dur=");
        Serial.println(duration[i]);

        // Convert to cm
        distances[i] = (static_cast<float>(duration[i]) * 0.0343f) / 2.0f;

        // Delay before next sensor (avoid echo overlap)
        delay(interSensorDelay);
    }

    // Ensure all triggers off
    for (size_t j = 0; j < N; j++) digitalWrite(triggerPins[j], LOW);

    return distances;
}

void US_Sensor::update() {
    // Not used;
}
