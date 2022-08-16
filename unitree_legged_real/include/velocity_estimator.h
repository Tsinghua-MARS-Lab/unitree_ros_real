#include <Eigen/Dense>
#include "kalman_filter.h"
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <array>
#include <math.h>
#include <iostream>
#include <fstream>

#define A1_LEG_UP 0.2
#define A1_LEG_LOW 0.2
#define A1_LEG_HIP 0.08505


class VelocityEstimator {
    public:
        VelocityEstimator(double accelerometer_variance, double sensor_variance, double initial_variance, double robot_timestep);
        ~VelocityEstimator(){outfile.close();};
        void reset();
        void update(UNITREE_LEGGED_SDK::LowState*);
        Eigen::VectorXd estimated_velocity;
        Eigen::Vector3d acc_bias;
        std::array<float, 4> foot_threshold;
    private:
        double _compute_delta_time(UNITREE_LEGGED_SDK::LowState*);
        KalmanFilter kalman_filter;
        double robot_timestep;
        double _initial_variance;
        double _last_timestamp;
        ofstream outfile;
};

VelocityEstimator::VelocityEstimator(double accelerometer_variance, double sensor_variance, double initial_variance, double robot_timestep): 
kalman_filter(3, 3, 3), _initial_variance(initial_variance), _last_timestamp(0.0), robot_timestep(robot_timestep)
{
    kalman_filter.P = Eigen::MatrixXd::Identity(3, 3) * _initial_variance;
    kalman_filter.Q = Eigen::MatrixXd::Identity(3, 3) * accelerometer_variance;
    kalman_filter.R = Eigen::MatrixXd::Identity(3, 3) * sensor_variance;
    kalman_filter.H.setIdentity(3, 3);
    kalman_filter.F.setIdentity(3, 3);
    kalman_filter.B.setIdentity(3, 3);
    kalman_filter.B_initialized = true;
    acc_bias = Eigen::Vector3d{0, 0, 9.8};
    outfile.open("acc_data.csv", ios::out);
    outfile << "acc_x" << ',' << "acc_y" << ',' << "acc_z" << "," << "vel_x" << ',' << "vel_y" << ',' << "vel_z" << ','
            << "obs_vel_x" << "," << "obs_vel_y" << "," << "obs_vel_z" << "," << "gyro_1" << ',' << "gyro_2" << ',' << "gyro_3" << std::endl;
}


Eigen::MatrixXd computeJacobian(const Eigen::Vector3d& leg_angles, int leg_id) {
    double l_up = 0.2;
    double l_low = 0.2;
    double l_hip = 0.08505 * pow(-1, leg_id + 1); // TODO: check this

    double t1 = leg_angles[0];
    double t2 = leg_angles[1];
    double t3 = leg_angles[2];

    // Modified !!!!!!
    double l_eff = sqrt(pow(l_up, 2) + pow(l_low, 2) + 2 * l_up * l_low * cos(t3));

    double t_eff = t2 + t3 / 2;
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(3, 3);
    J(0, 0) = 0;
    J(0, 1) = -l_eff * cos(t_eff);
    J(0, 2) = l_low * l_up * sin(t3) * sin(t_eff) / l_eff - l_eff * cos(t_eff) / 2;
    J(1, 0) = -l_hip * sin(t1) + l_eff * cos(t1) * cos(t_eff);
    J(1, 1) = -l_eff * sin(t1) * sin(t_eff);
    J(1, 2) = -l_low * l_up * sin(t1) * sin(t3) * cos(t_eff) / l_eff - l_eff * sin(t1) * sin(t_eff) / 2;
    J(2, 0) = l_hip * cos(t1) + l_eff * sin(t1) * cos(t_eff);
    J(2, 1) = l_eff * sin(t_eff) * cos(t1);
    J(2, 2) = l_low * l_up * sin(t3) * cos(t1) * cos(t_eff) / l_eff + l_eff * sin(t_eff) * cos(t1) / 2;

    // Modified !!!!!!
    return J; 
}

void VelocityEstimator::reset() {
    kalman_filter.x.setZero(3, 1);
    kalman_filter.P.setIdentity(3, 3);
    kalman_filter.P *= _initial_variance;
    _last_timestamp = 0.0;
    estimated_velocity = kalman_filter.x;
}

double VelocityEstimator::_compute_delta_time(UNITREE_LEGGED_SDK::LowState *state) {
    double delta_time_s;
    if (_last_timestamp == 0) {
        delta_time_s = robot_timestep;
    } else {
        std::cout << state->tick << std::endl; 
        delta_time_s = (state->tick - _last_timestamp) / 1000.0;
    }
    _last_timestamp = state->tick;
    // hack
    delta_time_s = 0.01;
    return delta_time_s;
}

void VelocityEstimator::update(UNITREE_LEGGED_SDK::LowState *state) {
    double delta_time_s = _compute_delta_time(state);

    // Modified !!!!!!
    Eigen::Vector3d sensor_acc{state->imu.accelerometer[0] - acc_bias[0], state->imu.accelerometer[1] - acc_bias[1], state->imu.accelerometer[2]}; // sensor_acc(state->imu.accelerometer.data());
    outfile << sensor_acc[0] << "," << sensor_acc[1] <<  "," << sensor_acc[2] << ",";
    std::array<float, 4> _q; // = state->imu.quaternion;
    for (int i = 0; i < 4; i ++){
        _q[i] = state->imu.quaternion[i]; 
    }
    Eigen::Quaterniond base_quaternion;
    base_quaternion.w() = _q[0];
    base_quaternion.x() = _q[1];
    base_quaternion.y() = _q[2];
    base_quaternion.z() = _q[3];
    Eigen::Matrix3d rot_mat = base_quaternion.normalized().toRotationMatrix();
    Eigen::Vector3d gravity{0.0, 0.0, -acc_bias[2]};
    
    // Modified !!!!!!
    Eigen::Vector3d calibrate_acc = rot_mat * sensor_acc + gravity; // rot_mat * sensor_acc.resize(3, 1) + gravity;
    std::cout << "delta time " << delta_time_s << std::endl;
    if (delta_time_s != 0) {
        kalman_filter.predict(calibrate_acc * delta_time_s);
    } else {
        std::cout << "No kalman predict" << std::endl;
    }

    std::vector<Eigen::Vector3d> observed_velocity;
    std::array<int16_t, 4> foot_force; // = state->footForce;
    for (int i; i < 4; i ++){
        foot_force[i] = state->footForce[i]; 
    }
    for (int i=0; i<4; i++) {
        if (foot_force[i] > foot_threshold[i]) {
            std::cout << "velocity corrected! " << std::endl; 
            Eigen::Vector3d motor_angles{state->motorState[3 * i].q, state->motorState[3 * i + 1].q, state->motorState[3 * i + 2].q};
            Eigen::MatrixXd jacobian = computeJacobian(motor_angles, i);
            Eigen::Vector3d joint_velocities{state->motorState[3 * i].dq, state->motorState[3 * i + 1].dq, state->motorState[3 * i + 2].dq};
            Eigen::Vector3d leg_v_in_base = jacobian * joint_velocities;
            observed_velocity.push_back(rot_mat * (-leg_v_in_base)); 
        }
    }

    Eigen::Vector3d mean_v = Eigen::Vector3d::Zero(3, 1);
    if (observed_velocity.size() > 0) {
        for (int i=0; i<observed_velocity.size(); i++) {
            mean_v += observed_velocity[i] / observed_velocity.size();
        }
        kalman_filter.update(mean_v);
    }
    estimated_velocity = kalman_filter.x;
    std::cout << "v in world:" << estimated_velocity << std::endl;
    estimated_velocity = rot_mat.inverse() * estimated_velocity;
    std::cout << "v in base:" << estimated_velocity << std::endl;
    outfile << estimated_velocity[0] << "," << estimated_velocity[1] << "," << estimated_velocity[2] << ",";
    outfile << mean_v[0] << "," << mean_v[1] << "," << mean_v[2] << ",";
    outfile << state->imu.gyroscope[0] << "," << state->imu.gyroscope[1] << "," << state->imu.gyroscope[2] << std::endl;
}
