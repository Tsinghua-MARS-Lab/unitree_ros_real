/**
 * @file legged_twist.h
 * @author Ziwen Zhuang(zhuangzw@sqz.ac.cn)
 * @brief The ROS node that publish twist based on A1 legged motion.
 * @version 0.1
 * @date 2022-08-20
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef _UNTIREE_LEGGED_TWIST_H_
#define _UNTIREE_LEGGED_TWIST_H_

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_listener.h>
#include "std_msgs/Int8.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include <unitree_legged_msgs/LowState.h>
#include <eigen3/Eigen/Dense> // refering to https://stackoverflow.com/a/23299290
#include <math.h>


class LeggedTwist
{
public:
    std::string robot_namespace;
    int frequency; // main frequency to compute legged twist
    int foot_touch_threshold = 20;
    int twist_publisher_count = 0;

    ros::NodeHandle ros_handle;
    float quaternion_buffer [4] = {0.0, 0.0, 0.0, 1.0}; // x, y, z, w order
    float joints_pos_buffer [12] = {0};
    float joints_vel_buffer [12] = {0};
    int foot_forces_buffer [4] = {0};
    tf::TransformListener tf_listener;
    ros::Subscriber low_state_subscriber;
    ros::Subscriber imu_subscriber;
    ros::Publisher twist_publisher;
    ros::Publisher feet_touch_publisher;
    ros::Timer main_loop_timer;

protected:
    void get_params();

    // Some constants that are not designed to be updated
    const std::string imu_link = "/imu_link";
    const std::string calf_link_template = "/{}_calf";
    // refering to a1_description/xacro/const.xacro, robot.xacro, leg.xacro. TODO: read from the file.
    const float thigh_offset = 0.0838;
    const float thigh_length = 0.2;
    const float calf_length = 0.2;
    // The order of leg names should be the same as unitree_legged_sdk/include/quadruped.h and a1_description/xacro/robot.xacro
    const std::string leg_names [4] = {"FR", "FL", "RR", "RL"};
    const float mirror_signs [4] = {-1., 1., -1., 1.};

    Eigen::Matrix3d Jacobian(double q_calf, double q_thigh, double q_hip, float mirror_sign);

    void low_state_callback(const unitree_legged_msgs::LowState::ConstPtr &msg);
    void main_loop_callback(const ros::TimerEvent& event);
    
public:
    LeggedTwist(
        std::string robot_namespace,
        ros::NodeHandle nh
    );
};

#endif