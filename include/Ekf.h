#ifndef US_EKF_H
#define US_EKF_H

#include <array>

class US_EKF {
public:
  US_EKF();
  ~US_EKF() = default;

  // Initialize filter with an initial position (px, py). Velocities set to zero.
  void init(float px, float py);

  // Predict step: dt in seconds
  void predict(float dt);

  // Update step: provide measurement px_meas, py_meas and booleans use_x/use_y to
  // indicate which axes are valid. If an axis is not used, pass false and its value is ignored.
  void update(float px_meas, float py_meas, bool use_x, bool use_y);

  // Convenience: one-shot predict+update
  void step(float dt, float px_meas, float py_meas, bool use_x, bool use_y);

  // Accessors
  std::array<float,2> getPosition() const; // {px, py}
  std::array<float,2> getVelocity() const; // {vx, vy}

  // Tuning (public for easy changes)
  float sigma_acc = 30.0f;        // assumed acceleration stddev (units/sec^2)
  float meas_std_default = 20.0f; // default measurement stddev (units)

private:
  // Internal state
  float x_[4];     // px, py, vx, vy
  float P_[4][4];  // covariance

  // helpers
  void resetCovariance();
};

#endif // US_EKF_H
