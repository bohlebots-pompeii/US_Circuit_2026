//
// Created by julius on 20.11.2025.
//

#ifndef US_MB_US_H
#define US_MB_US_H
#include <array>

class US_Sensor {
public:
  static void initPins();

  static void update();

  static std::array<float, 4> getData();
};
#endif //US_MB_US_H