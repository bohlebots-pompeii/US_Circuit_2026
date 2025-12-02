//
// Created by julius on 02.12.2025.
//
#pragma once

struct Kalman1D {
  float x;    // estimate
  float P;    // estimate covariance
  float Q;    // process noise
  float R;    // measurement noise
  bool init=false;
  explicit Kalman1D(float q=0.01f, float r=0.1f): x(0), P(1), Q(q), R(r), init(false) {}
  float update(float z) {
    if(!init) { x = z; P = 1.0f; init = true; return x; }
    // Predict (here constant model: x_k = x_{k-1})
    P += Q;
    // Update
    const float K = P / (P + R);
    x = x + K * (z - x);
    P = (1.0f - K) * P;
    return x;
  }
};
