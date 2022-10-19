/**
 * @file legged_twist.cpp
 * @author Ziwen Zhuang(zhuangzw@sqz.ac.cn)
 * @brief The ROS node that publish twist based on A1 legged motion.
 * @version 0.1
 * @date 2022-08-20
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "legged_twist.h"

void LeggedTwist::get_params()
{
    this->ros_handle.param<int>("frequency", this->frequency, 100);
    this->ros_handle.param<int>("foot_touch_threshold", this->foot_touch_threshold, 20);
}

Eigen::Matrix3d rotation_matrix_x(double q)
{
    Eigen::Matrix3d mat;
    mat(0, 0) = 1.;
    mat(1, 1) = cos(q);
    mat(1, 2) = sin(q);
    mat(2, 1) = -sin(q);
    mat(2, 2) = cos(q);
    
    return mat;
}

Eigen::Matrix3d rotation_matrix_y(double q)
{
    Eigen::Matrix3d mat;
    mat(0, 0) = cos(q);
    mat(0, 2) = -sin(q);
    mat(1, 1) = 1.;
    mat(2, 0) = sin(q);
    mat(2, 2) = cos(q);

    return mat;
}

Eigen::Matrix3d R_from_quaternion(double x, double y, double z, double w)
{
    // refering to https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation `Quaternion-derived rotation matrix`
    Eigen::Matrix3d mat;
    mat(0, 0) = 1 - 2 * (y*y + z*z);
    mat(0, 1) = 2 * (x*y - z*w);
    mat(0, 2) = 2 * (x*z + y*w);

    mat(1, 0) = 2 * (x*y + z*w);
    mat(1, 1) = 1 - 2 * (x*x + z*z);
    mat(1, 2) = 2 * (y*z - x*w);
    
    mat(2, 0) = 2 * (x*z - y*w);
    mat(2, 1) = 2 * (y*z + x*z);
    mat(2, 2) = 1 - 2 * (x*x + y*y);

    return mat;
}

Eigen::Matrix3d LeggedTwist::Jacobian(double q_calf, double q_thigh, double q_hip, float mirror_sign)
{
    Eigen::Matrix3d J_foot_calf;
    J_foot_calf(0, 2) = - this->calf_length;
    Eigen::Matrix3d R_calf_thigh = rotation_matrix_y(q_calf);
    Eigen::Matrix3d J_calf_thigh;
    J_calf_thigh(0, 1) = -this->thigh_length;
    Eigen::Matrix3d J_thigh_hip;
    J_thigh_hip(2, 2) = - mirror_sign * this->thigh_offset;
    Eigen::Matrix3d R_thigh_hip = rotation_matrix_y(q_thigh);
    Eigen::Matrix3d R_hip_trunk = rotation_matrix_x(q_hip);

    Eigen::Matrix3d jacobian = J_foot_calf;
    jacobian = R_calf_thigh * jacobian + J_calf_thigh;
    jacobian = R_thigh_hip * jacobian + J_thigh_hip;
    jacobian = R_hip_trunk * jacobian;

    return jacobian;
}

void LeggedTwist::low_state_callback(const unitree_legged_msgs::LowState::ConstPtr &msg)
{
    this->quaternion_buffer[0] = msg->imu.quaternion[1];
    this->quaternion_buffer[1] = msg->imu.quaternion[2];
    this->quaternion_buffer[2] = msg->imu.quaternion[3];
    this->quaternion_buffer[3] = msg->imu.quaternion[0];
    for (int i (0); i < 12; i++) this->joints_pos_buffer[i] = msg->motorState[i].q;
    for (int i (0); i < 12; i++) this->joints_vel_buffer[i] = msg->motorState[i].dq;
    for (int i (0); i < 4; i++) this->foot_forces_buffer[i] = msg->footForce[i];

}

void LeggedTwist::main_loop_callback(const ros::TimerEvent& event)
{
    int valid_legs_count = 0;
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();

    for (int i (0); i < 4; i++)
    {
        if (this->foot_forces_buffer[i] < this->foot_touch_threshold) continue;

        // continue Jacobian matrix of the given leg
        Eigen::Matrix3d jacobian = this->Jacobian(
            this->joints_pos_buffer[i*3 + 2],
            this->joints_pos_buffer[i*3 + 1],
            this->joints_pos_buffer[i*3 + 0],
            this->mirror_signs[i]
        );
        // compute foot velocity in base_link frame
        Eigen::Vector3d q_dot (
            this->joints_vel_buffer[i*3 + 0],
            this->joints_vel_buffer[i*3 + 1],
            this->joints_vel_buffer[i*3 + 2]
        );
        Eigen::Vector3d x_dot = jacobian * q_dot;
        // compute rotation matrix from base_link to odom (world frame)
        Eigen::Matrix3d R_base_odom = R_from_quaternion(
            this->quaternion_buffer[0],
            this->quaternion_buffer[1],
            this->quaternion_buffer[2],
            this->quaternion_buffer[3]
        );
        Eigen::Vector3d legged_velocity = - R_base_odom * x_dot;

        valid_legs_count += 1;
        velocity += legged_velocity;
    }
    
    if (valid_legs_count > 0)
    {
        geometry_msgs::TwistWithCovarianceStamped ros_msg;
        ros_msg.header.seq = this->twist_publisher_count++;
        ros_msg.header.stamp = ros::Time::now();
        ros_msg.header.frame_id = this->robot_namespace + "/base";
        ros_msg.twist.twist.linear.x = velocity[0] / valid_legs_count;
        ros_msg.twist.twist.linear.y = velocity[1] / valid_legs_count;
        ros_msg.twist.twist.linear.z = velocity[2] / valid_legs_count;
        ros_msg.twist.covariance[0 * 6 + 0] = 0.1;
        ros_msg.twist.covariance[1 * 6 + 1] = 0.1;
        ros_msg.twist.covariance[2 * 6 + 2] = 0.1;
        ros_msg.twist.covariance[3 * 6 + 3] = 20.;
        ros_msg.twist.covariance[4 * 6 + 4] = 20.;
        ros_msg.twist.covariance[5 * 6 + 5] = 20.;
        // NOTE: DO NOT use angular attribute in the twist. It is not implemented.
        this->twist_publisher.publish(ros_msg);
    }
    std_msgs::Int8 feet_count_msg; feet_count_msg.data = valid_legs_count;
    this->feet_touch_publisher.publish(feet_count_msg);
}

LeggedTwist::LeggedTwist(
    std::string robot_namespace,
    ros::NodeHandle nh
):
    robot_namespace(robot_namespace),
    ros_handle(nh)
{
    this->get_params();

    this->low_state_subscriber = this->ros_handle.subscribe(
        "low_state",
        10,
        &LeggedTwist::low_state_callback,
        this
    );
    this->feet_touch_publisher = this->ros_handle.advertise<std_msgs::Int8>(
        "num_feet_touches",
        1
    );
    this->twist_publisher = this->ros_handle.advertise<geometry_msgs::TwistWithCovarianceStamped>(
        "twist",
        10
    );

    this->main_loop_timer = this->ros_handle.createTimer(
        ros::Duration(1. / this->frequency),
        &LeggedTwist::main_loop_callback,
        this
    );
}

int main(int argc, char** argv)
{
    std::string robot_namespace (argv[1]);
    ros::init(argc, argv, robot_namespace);
    ros::NodeHandle nh ("~");

    // construct and initialize this ros node
    LeggedTwist legged_twist_estimator(
        robot_namespace,
        nh
    );
    ros::spin();
    return 0;
}