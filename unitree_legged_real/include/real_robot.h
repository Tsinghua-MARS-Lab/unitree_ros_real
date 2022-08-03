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
#include "sensor_msgs/Imu.h"
#include "std_msgs/Float32.h"
#include <unitree_legged_msgs/LegsCmd.h>
#include <unitree_legged_srvs/SetGaitType.h>
#include <unitree_legged_srvs/SetHighMode.h>
#include <unitree_legged_srvs/SetSpeedLevel.h>

class UnitreeRos: public RosUdpHandler
{
public:
    int state_check_times_max = 50; // When changing mode through UDP, the service handler checks the updated state at most this times
    int state_check_freq = 10; // The frequency of checking mode update.
    
    ros::Publisher pose_estimation_publisher;

    ros::ServiceServer set_gaitType_service;
    ros::Subscriber high_twist_subscriber;
    ros::Subscriber foot_raise_height_subscriber;
    ros::Subscriber body_height_subscriber;

    ros::Subscriber low_motor_subscriber;

protected:
    // For simplicity, some states and commands must be processed and estimate
    void pose_estimate_and_publish();

    bool set_gaitType_srv_callback(
        unitree_legged_srvs::SetGaitType::Request &req,
        unitree_legged_srvs::SetGaitType::Response &res
    );
    bool set_high_mode_srv_callback(
        unitree_legged_srvs::SetHighMode::Request &req,
        unitree_legged_srvs::SetHighMode::Response &res
    );
    bool set_high_speedLevel_srv_callback(
        unitree_legged_srvs::SetSpeedLevel::Request &req,
        unitree_legged_srvs::SetSpeedLevel::Response &res
    );

    void high_twist_callback(const geometry_msgs::Twist::ConstPtr &msg);
    void foot_raise_height_callback(const std_msgs::Float32::ConstPtr &msg);
    void body_height_callback(const std_msgs::Float32::ConstPtr &msg);

    void low_motor_callback(const unitree_legged_msgs::LegsCmd::ConstPtr &msg);
    
public:
    UnitreeRos(
            const char* robot_namespace,                    // Every topic from this node must have a namespace as prefix.
            const float udp_duration,                       // unit (sec), the duration to call udp related methods.
            uint8_t level,                                  // (UNTIREE_LEGGED_SDK::HIGHLEVEL, LOWLEVEL), do use the enum.
            UNITREE_LEGGED_SDK::HighLevelType highControl,
            int power_protect_level,
            bool &dryrun                                    // If true, does not send the udp message in udp_send() but do everything else.
        );
    void publisher_init();
    void server_init();
    void subscriber_init();
};