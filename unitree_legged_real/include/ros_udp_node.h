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
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"

class RosUdpHandler
{
public:
    std::string robot_namespace_;
    uint8_t ctrl_level;
    bool dryrun_;
    UNITREE_LEGGED_SDK::UDP udp;
    float udp_duration_;
    bool cmd_check; // if set, /*_cmd_checker publisher will publish the from buffer at each udpSend()
    UNITREE_LEGGED_SDK::LoopFunc loop_udp_send;
    UNITREE_LEGGED_SDK::LoopFunc loop_udp_recv;
    
    UNITREE_LEGGED_SDK::Safety safe;
    float position_protect_limit;
    int low_power_protect_level;
    float low_cmd_default_tau = 0.65f;
    bool start_stand; // if true, the motor will be initialized to mode 10, otherwise mode 0.

    UNITREE_LEGGED_SDK::HighCmd high_cmd_buffer = {0};
    UNITREE_LEGGED_SDK::HighState high_state_buffer = {0};
    bool high_cmd_metadata_get = false;
    bool high_cmd_get = false; // set to true when a proper cmd is recieved from algorithm and being handled.

    UNITREE_LEGGED_SDK::LowCmd low_cmd_buffer = {0};
    UNITREE_LEGGED_SDK::LowState low_state_buffer = {0};
    bool low_cmd_metadata_get = false;
    bool low_cmd_get = false; // set to true when a proper cmd is recieved from algorithm and being handled.

    ros::NodeHandle ros_handle;
    ros::Subscriber cmd_subscriber;
    ros::Publisher cmd_checker;
    ros::Publisher state_publisher;

protected:
    void udp_init(uint8_t level);
    void udp_start();
    // If dryrun, do everything except udp.send(). Publish the message to send everytime in udp_send().
    void udp_send();
    // Publish the message directly to ROS everytime in udp_recv().
    void udp_recv();
    // Initialize the buffer for the use of setting only part of the cmd.
    void set_default_high_cmd();
    void set_default_low_cmd();
    void high_cmd_metadata_update();
    void low_cmd_metadata_update();
    // Expose udp communication directly through ROS topics
    void high_cmd_callback(const unitree_legged_msgs::HighCmd::ConstPtr &msg);
    void low_cmd_callback(const unitree_legged_msgs::LowCmd::ConstPtr &msg);
    // Some helper functions
    void high_state_publish();
    void low_state_publish();
    void publisher_init();
    void subscriber_init();

public:
    RosUdpHandler(
            std::string robot_namespace,                    // Every topic from this node must have a namespace as prefix.
            const float udp_duration,                       // unit (sec), the duration to call udp related methods.
            uint8_t level,                                  // (UNTIREE_LEGGED_SDK::HIGHLEVEL, LOWLEVEL), do use the enum.
            float position_protect_limit,                   // Please check safety.h, 0.0 is the least limit
            int power_protect_level,                        // Refer to unitree_legged_sdk/safety.h
            bool cmd_check,
            bool start_stand,                               // If true, defualt motor mode is 10, otherwise 0.
            bool dryrun                                     // If true, does not send the udp message in udp_send() but do everything else.
        );
};

#endif