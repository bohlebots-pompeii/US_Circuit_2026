//
// Created by julius on 02.12.2025.
//
#pragma once

inline float median3(const float a, const float b, const float c) {
  if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
  if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
  return c;
}
