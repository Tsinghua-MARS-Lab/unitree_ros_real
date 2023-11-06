/**
 * @file ros_udp_node.h
 * @author Ziwen Zhuang (zhuangzw@sqz.ac.cn)
 * @brief The ros node run on NX to transport all robot proprioception and ros command
 * @version 0.1
 * @date 2022-07-26
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef _UNITREE_ROS_UDP_NODE_H_
#define _UNITREE_ROS_UDP_NODE_H_

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
// #include <unitree_legged_msgs/HighCmd.h>
// #include <unitree_legged_msgs/HighState.h>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "unitree_legged_sdk/joystick.h"
#include "convert.h"

class RosUdpHandler
{
public:
    std::string robot_namespace;
    uint8_t ctrl_level;
    bool dryrun; // If true, does not send the udp message in udp_send() but do everything else.
    UNITREE_LEGGED_SDK::UDP udp;
    float udp_duration;
    bool cmd_check; // if set, /*cmd_checker publisher will publish the from buffer at each udpSend()
    ros::Time cmd_refresh_time;
    float cmd_lost_timelimit; // (in sec) If system does not recieve LegsCmd or LowCmd within this timelimit, robot will be in safety position.
    bool freeze_lost; // if true, the robot will freeze at current position. Otherwise, the motor will be in damping mode.
    float safety_guard_duration; // The frequency of safety_guard_callback function. (in sec)
    float torque_protect_limit; // if the estimated low_state torque exceed this limit, the program exits. The robot is probability to fall down.
    float torque_protect_limit_calf; // Go1's calf has higher torque limit, which is set individually.
    float pitch_protect_limit; // if greater than 0., the program exits when abs(pitch) of base is out of limit. The robot is probability to fall down.
    float roll_protect_limit; // if greater than 0., the program exits when abs(roll) of base is out of limit. The robot is probability to fall down.
    float R2_press_protect; // if true, the program exits when detecting the L2 button on the wirelessRemote is pressed.
    bool robot_safe = true; // if false, this program is shutting down. No code should be able to set this value to true.
    int udp_recv_result = -2;
    UNITREE_LEGGED_SDK::LoopFunc loop_udp_send;
    UNITREE_LEGGED_SDK::LoopFunc loop_udp_recv;
    UNITREE_LEGGED_SDK::LoopFunc loop_udp_translate;
    
    UNITREE_LEGGED_SDK::Safety safe;
    float position_protect_limit; // Please check safety.h, 0.0 is the least limit
    int power_protect_level; // Refer to unitree_legged_sdk/safety.h
    float safe_Kp = 15.;
    float safe_Kd = 0.5;
    float low_cmd_default_tau = 0.65f;
    bool start_stand; // if true, the motor will be initialized to mode 10, otherwise mode 0.

    // UNITREE_LEGGED_SDK::HighCmd high_cmd_buffer = {0};
    // UNITREE_LEGGED_SDK::HighState high_state_buffer = {0};
    // bool high_cmd_metadata_get = false;

    pthread_mutex_t low_cmd_mutex = PTHREAD_MUTEX_INITIALIZER;
    UNITREE_LEGGED_SDK::LowCmd low_cmd_buffer = {0};
    pthread_mutex_t low_state_mutex = PTHREAD_MUTEX_INITIALIZER;
    UNITREE_LEGGED_SDK::LowState low_state_buffer = {0};
    bool low_cmd_metadata_get = false;

    ros::NodeHandle ros_handle;
    ros::Subscriber cmd_subscriber;
    ros::Publisher cmd_checker;
    ros::Publisher state_publisher;
    ros::Timer cmd_lost_check_timer;
    ros::Timer safety_guard_timer;

protected:
    void get_params();
    void set_params();
    void udp_init(uint8_t level);
    void udp_start();
    // If dryrun, do everything except udp.send().
    void udp_send();
    void udp_recv();
    // Publish the low_state directly to ROS everytime in udp_translate().
    // Also publish every latest low_cmd to ROS (for double check).
    void udp_translate();
    // Initialize the buffer for the use of setting only part of the cmd.
    void set_default_high_cmd();
    void set_default_low_cmd();
    // void high_cmd_metadata_update();
    void low_cmd_metadata_update();
    // Expose udp communication directly through ROS topics
    // void high_cmd_callback(const unitree_legged_msgs::HighCmd::ConstPtr &msg);
    void low_cmd_callback(const unitree_legged_msgs::LowCmd::ConstPtr &msg);
    // Some helper functions
    void cmd_lost_check_callback(const ros::TimerEvent& event);
    void safety_guard_callback(const ros::TimerEvent& event);
    // void high_state_publish();
    void low_state_publish();
    void publisher_init();
    void subscriber_init();
    void timer_init();

public:
    RosUdpHandler(
            std::string robot_namespace,                    // Every topic from this node must have a namespace as prefix.
            const float udp_duration,                       // unit (sec), the duration to call udp related methods.
            uint8_t level,                                  // (UNTIREE_LEGGED_SDK::HIGHLEVEL, LOWLEVEL), do use the enum.
            ros::NodeHandle nh
        );
    ~RosUdpHandler();
};

#endif