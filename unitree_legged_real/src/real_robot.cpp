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

#include "real_robot.h"

void UnitreeRos::high_twist_callback(const geometry_msgs::Twist::ConstPtr &msg)
{
    this->high_cmd_buffer.velocity[0] = msg->linear.x;
    this->high_cmd_buffer.velocity[1] = msg->linear.y;
    this->high_cmd_buffer.yawSpeed = msg->angular.z;
}

bool UnitreeRos::set_gaitType_srv_callback(
    unitree_legged_srvs::SetGaitType::Request &req,
    unitree_legged_srvs::SetGaitType::Response &res
)
{
    if (this->ctrl_level != UNITREE_LEGGED_SDK::HIGHLEVEL)
    {
        ROS_ERROR("Your set_gaitType service is captured by a UnitreeRos high level handler.");
        exit(-1);
    }

    ros::Rate r(this->state_check_freq);
    int state_check_count = 0;

    this->high_cmd_buffer.gaitType = req.gait_type;
    while (this->high_state_buffer.gaitType != req.gait_type && state_check_count < this->state_check_times_max)
    {
        ROS_INFO(
            "Not getting the correct HighState update, gaitType is %i, waiting...",
            this->high_state_buffer.gaitType
        );
        r.sleep();
        state_check_count++;
    }

    res.success = (this->high_state_buffer.gaitType == req.gait_type);
    return true;
}

UnitreeRos::UnitreeRos(
    const char* robot_namespace,
    const float udp_duration,
    uint8_t level,
    UNITREE_LEGGED_SDK::HighLevelType highControl,
    int power_protect_level,
    bool &dryrun
):
    RosUdpHandler(robot_namespace, udp_duration, level, highControl, power_protect_level, dryrun)
{
    this->server_init();
}

void UnitreeRos::server_init()
{
    this->set_gaitType_service = this->ros_handle.advertiseService(
        this->robot_namespace_ + "/set_gaitType",
        &UnitreeRos::set_gaitType_srv_callback,
        this
    );
}

int main(int argc, char **argv)
{
    if (argc != 4)
    {
        std::cout << "You must provide exactly 3 keyword arguments, please use roslaunch rather than rosrun.";
        std::cout << std::endl;
        exit(-1);
    }

    // parse argument in a non-dynamic way, please use roslaunch!!!
    bool dryrun = strcasecmp(argv[1], "true");
    const char* robot_namespace = argv[2];
    std::string udp_duration_s (argv[3]);
    float udp_duration = std::stof(udp_duration_s);

    // construct and initialize this ros node
    ros::init(argc, argv, robot_namespace);
    UnitreeRos unitree_ros_node(
        robot_namespace,
        udp_duration,
        UNITREE_LEGGED_SDK::HIGHLEVEL,
        UNITREE_LEGGED_SDK::HighLevelType::Basic,
        1,
        dryrun
    );
    ros::spin();
    return 0;
}
