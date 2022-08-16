#include "kalman_filter.h"
#include <iostream>
#include <vector>

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

int main(int argc, char* argv) {
    KalmanFilter kalman_filter(3, 3, 3);
    kalman_filter.P = Eigen::MatrixXd::Identity(3, 3) * 0.1;
    kalman_filter.Q = Eigen::MatrixXd::Identity(3, 3) * 0.1;
    kalman_filter.R = Eigen::MatrixXd::Identity(3, 3) * 0.1;
    kalman_filter.H.setIdentity(3, 3);
    kalman_filter.F.setIdentity(3, 3);
    kalman_filter.B.setIdentity(3, 3);
    kalman_filter.B_initialized = true;
    std::vector<Eigen::Vector3d> test_data;
    for (int i=0; i<100; i++) {
        test_data.push_back(0.5 * Eigen::Vector3d::Random(3, 1) + 0.1 * Eigen::Vector3d::Ones(3, 1));
    }
    Eigen::Vector3d naive{0.0, 0.0, 0.0};
    for (int i=0; i<test_data.size(); i++) {
        kalman_filter.predict(test_data[i]);
        naive += test_data[i];
        if (i % 10 == 0){
            kalman_filter.update(0.1 * i * Eigen::Vector3d::Ones(3, 1) + 0.02 * Eigen::Vector3d::Random(3, 1));
        }
        std::cout << i << "\nkalman: " << kalman_filter.x << 
                          "\nnaive: " << naive << std::endl;
    }
    return 0;
}
