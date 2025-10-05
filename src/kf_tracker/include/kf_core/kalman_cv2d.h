#pragma once
#include <Eigen/Dense>

namespace kf_core {

// 2D Constant-Velocity Kalman Filter (stub for now)
class KalmanCV2D {
public:
  KalmanCV2D();

  // initialize from first measurement (x,y)
  void initFromMeasurement(const Eigen::Vector2d& z);

  // time update with timestep dt (seconds)
  void predict(double dt);

  // measurement update with (x,y)
  void update(const Eigen::Vector2d& z);

  // current position estimate (x,y)
  Eigen::Vector2d position() const;

private:
  // state: [px, py, vx, vy]^T
  Eigen::Vector4d x_;
  // covariance
  Eigen::Matrix4d P_;
};

} // namespace kf_core
