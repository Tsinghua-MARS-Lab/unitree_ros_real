#include <Eigen/Dense>
#include <iostream>
#pragma once
class KalmanFilter {

public:

  /**
  * Create a Kalman filter with the specified matrices.
  *   A - System dynamics matrix
  *   C - Output matrix
  *   Q - Process noise covariance
  *   R - Measurement noise covariance
  *   P - Estimate error covariance
  */
  KalmanFilter(
      int dim_x,
      int dim_z,
      int dim_u
  );

  void predict(const Eigen::VectorXd& u);

  void update(const Eigen::VectorXd& z);

  void update(const Eigen::VectorXd& z, const Eigen::MatrixXd& R, const Eigen::MatrixXd& H);

  Eigen::VectorXd get_x() {return x;};

  Eigen::MatrixXd P, Q, B, F, H, R;

  Eigen::VectorXd x;

  bool B_initialized;

private:

  // Matrices for computation
  Eigen::MatrixXd M, K, S, SI;

  // System dimensions
  int dim_x, dim_z, dim_u;

  // Is the filter initialized?
  bool z_initialized;

  // n-size identity
  Eigen::MatrixXd _I;

  // Estimated states
  Eigen::VectorXd y, z;

  double _alpha_sq;
};

KalmanFilter::KalmanFilter(int dim_x, int dim_z, int dim_u) {
  this->dim_x = dim_x;
  this->dim_z = dim_z;
  this->dim_u = dim_u;
  x.setZero(dim_x, 1);
  P.setIdentity(dim_x, dim_x);
  Q.setIdentity(dim_x, dim_x);
  B_initialized = false;
  F.setIdentity(dim_x, dim_x);
  H.setZero(dim_z, dim_x);
  R.setIdentity(dim_z, dim_z);
  z_initialized = false;
  K.setZero(dim_x, dim_z);
  y.setZero(dim_z, 1);
  S.setZero(dim_z, dim_z);
  SI.setZero(dim_z, dim_z);
  _I.setIdentity(dim_x, dim_x);
  _alpha_sq = 1.0;
}


void KalmanFilter::predict(const Eigen::VectorXd& u) {
    if (B_initialized) {
        x = F * x + B * u;
    } else {
        x = F * x;
    }
    P = _alpha_sq * (F * P * F.transpose()) + Q;
}

void KalmanFilter::update(const Eigen::VectorXd& zz) {
    y = zz - H * x;
    Eigen::MatrixXd PHT = P * H.transpose();
    S = H * PHT + R;
    SI = S.inverse();
    K = PHT * SI;
    x = x + K * y;
    Eigen::MatrixXd I_KH = _I - K * H;
    P = I_KH * P * I_KH.transpose() + K * R * K.transpose();
    this->z = zz;
}

