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
#ifndef _UNITERE_ROS_REAL_ROBOT_H_
#define _UNITERE_ROS_REAL_ROBOT_H_

#include "ros_udp_node.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"
#include <unitree_legged_msgs/LegsCmd.h>
#include <unitree_legged_msgs/WirelessRemote.h>
#include <unitree_legged_srvs/SetGaitType.h>
#include <unitree_legged_srvs/SetHighMode.h>
#include <unitree_legged_srvs/SetSpeedLevel.h>
#include "unitree_legged_sdk/joystick.h"
#include "unitree_legged_sdk/go1_const.h"

class UnitreeRos: public RosUdpHandler
{
public:
    int state_check_times_max = 50; // When changing mode through UDP, the service handler checks the updated state at most this times
    int state_check_freq = 10; // The frequency of checking mode update.
    int timer_freq = 50; // default timer frequency for some default functions.
    bool publish_imu = false;
    int imu_publish_freq = 100;
    int imu_publish_seq = 0;
    bool publish_joint_state = false;
    int joint_state_publish_freq = 50;
    int joint_state_publish_seq = 0;
    bool publish_wirelessRemote = false;
    
    ros::Publisher imu_publisher;
    ros::Publisher wirelessRemote_publisher;
    ros::Publisher position_limit_publisher;
    ros::Publisher joint_state_publisher;

    ros::Subscriber low_motor_subscriber;

    ros::Timer imu_publish_timer;
    ros::Timer wirelessRemote_publish_timer;
    ros::Timer joint_state_publish_timer;

protected:
    void get_params();
    void set_params();

    void low_motor_callback(const unitree_legged_msgs::LegsCmd::ConstPtr &msg);
    
    void imu_publish_callback(const ros::TimerEvent& event);
    void wirelessRemote_publish_callback(const ros::TimerEvent& event);
    void joint_state_publish_callback(const ros::TimerEvent& event);

public:
    UnitreeRos(
            std::string robot_namespace,                    // Every topic from this node must have a namespace as prefix.
            const float udp_duration,                       // unit (sec), the duration to call udp related methods.
            uint8_t level,                                  // (UNTIREE_LEGGED_SDK::HIGHLEVEL, LOWLEVEL), do use the enum.
            ros::NodeHandle nh
        );
    ~UnitreeRos();
    void publisher_init();
    void server_init();
    void subscriber_init();
    void timer_init();
};

#endif
