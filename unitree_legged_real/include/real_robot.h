/**
 * @file real_robot.h
 * @author Ziwen Zhuang (zhuangzw@sqz.ac.cn)
 * @brief The full node for entire Unitree ROS handler and disentangled interface.
 * @version 0.1
 * @date 2022-07-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "ros_udp_node.h"

class UnitreeRos: RosUdpHandler
{
    // For simplicity, some states and commands must be processed and estimate
    void high_twist_callback(const geometry_msgs::Twist::ConstPtr &msg);
    void low_motor_callback(const unitree_legged_msgs::LegsCmd::ConstPtr &msg);
    void pose_estimate_and_publish();

    void set_gaitType_srv_callback(const);
    
};