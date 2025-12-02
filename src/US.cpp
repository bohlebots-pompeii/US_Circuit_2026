//
// created by Julius 02.12.2025 (patched 02.12.2025)
//

#include "US.h"
#include "config.h"
#include "Kalman1D.h"
#include <Arduino.h>
#include <array>
#include <cmath>
#include <algorithm>

float US_Sensor::x_pos = 0;
float US_Sensor::y_pos = 0;

float US_Sensor::x_size = 0;
float US_Sensor::y_size = 0;

static std::array<float, 4> lastData = {0, 0, 0, 0};

static Kalman1D kx(0.001f, 0.05f);
static Kalman1D ky(0.001f, 0.05f);

void US_Sensor::initPins() {
    for (const int echoPin : echoPins)
        pinMode(echoPin, INPUT);

    for (const int triggerPin : triggerPins) {
        pinMode(triggerPin, OUTPUT);
        digitalWrite(triggerPin, LOW);
    }
}

std::array<float, 4> US_Sensor::getData() {
    constexpr size_t N = echoPins.size();
    std::array<unsigned long, N> duration{};
    std::array<float, N> distances{};

    for (size_t idx = 0; idx < N; idx++) {
        for (size_t j = 0; j < N; j++) digitalWrite(triggerPins[j], LOW);
        delayMicroseconds(2);

        digitalWrite(triggerPins[idx], HIGH);
        delayMicroseconds(10);
        digitalWrite(triggerPins[idx], LOW);

        duration[idx] = pulseIn(echoPins[idx], HIGH);
        distances[idx] = (static_cast<float>(duration[idx]) * 0.0343f) / 2.0f;
    }

    for (size_t j = 0; j < N; j++) digitalWrite(triggerPins[j], LOW);

    return distances;
}

void US_Sensor::calibrateSensor() {
    float _x_average = 0;
    float _y_average = 0;

    constexpr int _runs = calibRuns;
    for (int i = 0; i < _runs; i++) {
        std::array<float, 4> data = getData();
        const float _x = data[1] + data[3]; // left + right
        const float _y = data[0] + data[2]; // front + back
        _x_average += _x;
        _y_average += _y;
        delay(sensorDelay);
    }

    if constexpr (_runs > 0) {
        x_size = _x_average / _runs;
        y_size = _y_average / _runs;
    }
}

std::array<float, 2> US_Sensor::calculatePosition(const std::array<float, 4>& data) {
    static std::array<bool, 4> blocked = {false, false, false, false};
    static bool first_update = true;

    if (first_update) { // make sure we have lastData else -> Error
        lastData = data;
        first_update = false;
    }

    for (int i = 0; i < 4; ++i) {
        if (std::fabs(data[i] - lastData[i]) > maxJump) {
            blocked[i] = !blocked[i];
        }
    }

    if (x_size > 0.0001f) { // x
        if (std::fabs(data[1] + data[3] - x_size) <= recovDist) {
            blocked[1] = false;
            blocked[3] = false;
        }
    }
    if (y_size > 0.0001f) { // y
        if (std::fabs(data[0] + data[2] - y_size) <= recovDist) {
            blocked[0] = false;
            blocked[2] = false;
        }
    }

    std::array<float, 4> calc_data = data;
    for (int i = 0; i < 4; ++i) {
        if (blocked[i]) calc_data[i] = 0.0f;
    }

    float f  = calc_data[0]; // front
    float le = calc_data[1]; // left
    float ba = calc_data[2]; // back
    float ri = calc_data[3]; // right

    if (le <= 0 && ri > 0) le = x_size - ri;
    if (ri <= 0 && le > 0) ri = x_size - le;

    if (f <= 0 && ba > 0) f = y_size - ba;
    if (ba <= 0 && f > 0) ba = y_size - f;

    const float x = le - ri;
    const float y = f - ba;

    return { x / 2.0f, -y / 2.0f };
}

void US_Sensor::update() {
    const auto data = getData();
    const auto pos = calculatePosition(data);

    const float raw_x = pos[0];
    const float raw_y = pos[1];

    x_pos = kx.update(raw_x);
    y_pos = ky.update(raw_y);

    x_pos = pos[0];
    y_pos = pos[1];

    lastData = data; // store for next iter
}
