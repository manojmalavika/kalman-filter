#pragma once
#include <Eigen/Dense>

namespace kf_core {

// 2D Constant-Velocity Kalman Filter (stub for now)
class KalmanCV2D {
public:
  KalmanCV2D();

  void initFromMeasurement(const Eigen::Vector2d& z);
  void predict(double dt);
  void update(const Eigen::Vector2d& z);
  Eigen::Vector2d position() const;

private:
  // state: [px, py, vx, vy]^T
  Eigen::Vector4d x_;
  // covariance
  Eigen::Matrix4d P_;

  // measurement model (2x4)
  Eigen::Matrix<double, 2, 4> H_;
  // process covariance
  Eigen::Matrix4d Q_;
  // measurement covariance
  Eigen::Matrix2d R_;
  // state transition
  Eigen::Matrix4d F_;

};

} // namespace kf_core
