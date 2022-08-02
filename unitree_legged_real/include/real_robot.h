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
#include "geometry_msgs/Twist.h"
#include <unitree_legged_msgs/LegsCmd.h>
#include <unitree_legged_srvs/SetGaitType.h>

class UnitreeRos: RosUdpHandler
{
public:
    ros::ServiceServer set_gaitType_service;
    int state_check_times_max = 50; // When changing mode through UDP, the service handler checks the updated state at most this times
    int state_check_freq = 10; // The frequency of checking mode update.

protected:
    // For simplicity, some states and commands must be processed and estimate
    void high_twist_callback(const geometry_msgs::Twist::ConstPtr &msg);
    void low_motor_callback(const unitree_legged_msgs::LegsCmd::ConstPtr &msg);
    void pose_estimate_and_publish();

    bool set_gaitType_srv_callback(
        unitree_legged_srvs::SetGaitType::Request &req,
        unitree_legged_srvs::SetGaitType::Response &res
    );
    
public:
    UnitreeRos(
            const char* robot_namespace,                    // Every topic from this node must have a namespace as prefix.
            const float udp_duration,                       // unit (sec), the duration to call udp related methods.
            uint8_t level,                                  // (UNTIREE_LEGGED_SDK::HIGHLEVEL, LOWLEVEL), do use the enum.
            UNITREE_LEGGED_SDK::HighLevelType highControl,
            int power_protect_level,
            bool &dryrun                                    // If true, does not send the udp message in udp_send() but do everything else.
        );
    void server_init();
};