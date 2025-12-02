//
// Created by julius on 20.11.2025.
//

#ifndef US_MB_CONFIG_H
#define US_MB_CONFIG_H
#include <array>

constexpr std::array echoPins = {16, 19, 35, 25};
constexpr std::array triggerPins = {4, 18, 32, 26};

constexpr int sensorDelay = 10;
constexpr int calibRuns = 20;

constexpr int maxJump = 20.0f;
constexpr int recovDist = 10.0f;

#endif //US_MB_CONFIG_H
