#include "kf_core/kalman_cv2d.h"

namespace kf_core {

KalmanCV2D::KalmanCV2D()
  : x_(Eigen::Vector4d::Zero()),
    P_(Eigen::Matrix4d::Identity()) {}

void KalmanCV2D::initFromMeasurement(const Eigen::Vector2d& z) {
  x_.head<2>() = z;            // set position
  x_.tail<2>().setZero();      // zero velocity
}

void KalmanCV2D::predict(double /*dt*/) {
  // no-op stub for now
}

void KalmanCV2D::update(const Eigen::Vector2d& /*z*/) {
  // no-op stub for now
}

Eigen::Vector2d KalmanCV2D::position() const {
  return x_.head<2>();
}

} // namespace kf_core
