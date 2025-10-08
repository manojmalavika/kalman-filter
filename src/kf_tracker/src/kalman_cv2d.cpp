#include "kf_core/kalman_cv2d.h"

namespace kf_core {

KalmanCV2D::KalmanCV2D()
  : x_(Eigen::Vector4d::Zero()),
    P_(Eigen::Matrix4d::Zero()),
    H_((Eigen::Matrix<double, 2, 4>() <<
        1, 0, 0, 0,
        0, 1, 0, 0).finished()),
    Q_(Eigen::Matrix4d::Zero()),
    R_(Eigen::Matrix2d::Zero()),
    F_(Eigen::Matrix4d::Identity()) 
    {
      Q_.diagonal() << 1e-4, 1e-4, 1e-4, 1e-4;
      R_.diagonal() << 1e-4, 1e-4;
    }

void KalmanCV2D::initFromMeasurement(const Eigen::Vector2d& z) {
  x_.head<2>() = z;            // measured position
  x_.tail<2>().setZero();      // unknown velocity

  // variance
  double pos_var = R_(0, 0) * 10;   // 
  double vel_var = 100;   // velocity is very unceratin in the beginning

  P_.setZero();
  P_(0, 0) = pos_var;
  P_(1, 1) = pos_var;
  P_(2, 2) = vel_var;
  P_(3, 3) = vel_var;

}

void KalmanCV2D::predict(double dt) {

  F_ << 1, 0, dt, 0,
       0, 1, 0, dt,
       0, 0, 1, 0,
       0, 0, 0, 1;
  // state
  x_ = F_ * x_;
  // covariance
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanCV2D::update(const Eigen::Vector2d& z) {
  // innovation
  Eigen::Vector2d y = z - H_ * x_;
  
  Eigen::Matrix2d S = H_ * P_ * H_.transpose() + R_;

  Eigen::Matrix<double, 4, 2> K = P_ * H_.transpose() * S.inverse();

  x_ = x_ + K * y;
  P_ = (Eigen::Matrix4d::Identity() - K * H_) * P_;

}

Eigen::Vector2d KalmanCV2D::position() const {
  return x_.head<2>();
}

} // namespace kf_core
