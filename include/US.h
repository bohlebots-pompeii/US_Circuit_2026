//
// Created by julius on 20.11.2025.
//

#ifndef US_MB_US_H
#define US_MB_US_H
#include <array>

class US_Sensor {
public:
  static void initPins();

  static float x_size;
  static float y_size;

  static float x_pos, y_pos;

  static void calibrateSensor();

  static void update();

private:
  static std::array<float, 4> getData();

  static std::array<float, 2> calculatePosition(const std::array<float, 4>& data);
};

#endif //US_MB_US_H