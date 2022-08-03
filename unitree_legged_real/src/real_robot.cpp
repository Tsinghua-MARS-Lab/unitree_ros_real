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

template<class Type_T>
bool check_until_timeout(
    int check_freq,
    int max_check_limit,
    Type_T *buffer,
    Type_T *target
)
{
    ros::Rate r (check_freq);
    int check_count = 0;
    while ((*buffer) != (*target) && check_count < max_check_limit)
    {
        r.sleep();
        check_count++;
    }

    return (*buffer) == (*target);
}

bool UnitreeRos::set_gaitType_srv_callback(
    unitree_legged_srvs::SetGaitType::Request &req,
    unitree_legged_srvs::SetGaitType::Response &res
)
{
    if (this->ctrl_level != UNITREE_LEGGED_SDK::HIGHLEVEL)
    {
        ROS_ERROR("Your set_gaitType service is captured by a UnitreeRos low level handler.");
        exit(-1);
    }

    this->high_cmd_buffer.gaitType = req.gait_type;
    res.success = check_until_timeout(
        this->state_check_freq,
        this->state_check_times_max,
        &this->high_state_buffer.gaitType,
        &req.gait_type
    );

    return true;
}

bool UnitreeRos::set_high_mode_srv_callback(
    unitree_legged_srvs::SetHighMode::Request &req,
    unitree_legged_srvs::SetHighMode::Response &res
)
{
    if (this->ctrl_level != UNITREE_LEGGED_SDK::HIGHLEVEL)
    {
        ROS_ERROR("Your set_high_mode service is captured by a UnitreeRos low level handler.");
        exit(-1);
    }

    this->high_cmd_buffer.mode = req.mode;
    res.success = check_until_timeout(
        this->state_check_freq,
        this->state_check_times_max,
        &this->high_state_buffer.mode,
        &req.mode
    );

    return true;
}

bool UnitreeRos::set_high_speedLevel_srv_callback(
    unitree_legged_srvs::SetSpeedLevel::Request &req,
    unitree_legged_srvs::SetSpeedLevel::Response &res
)
{
    if (this->ctrl_level != UNITREE_LEGGED_SDK::HIGHLEVEL)
    {
        ROS_ERROR("Your set_high_mode service is captured by a UnitreeRos low level handler.");
        exit(-1);
    }

    this->high_cmd_buffer.speedLevel = req.speed_level;
    res.success = (this->high_cmd_buffer.mode == 3); // refering to unitree_legged_sdk/include/unitree_legged_sdk/comm.h

    return true;
}

void UnitreeRos::high_twist_callback(const geometry_msgs::Twist::ConstPtr &msg)
{
    this->high_cmd_buffer.velocity[0] = msg->linear.x;
    this->high_cmd_buffer.velocity[1] = msg->linear.y;
    this->high_cmd_buffer.yawSpeed = msg->angular.z;
}

void UnitreeRos::foot_raise_height_callback(const std_msgs::Float32::ConstPtr &msg)
{
    this->high_cmd_buffer.footRaiseHeight = msg->data;
}

void UnitreeRos::body_height_callback(const std_msgs::Float32::ConstPtr &msg)
{
    this->high_cmd_buffer.bodyHeight = msg->data;
}

void UnitreeRos::low_motor_callback(const unitree_legged_msgs::LegsCmd::ConstPtr &msg)
{
    for (int i = 0; i < 12; i++)
    {
        this->low_cmd_buffer.motorCmd[i].mode = msg->cmd[i].mode;
        this->low_cmd_buffer.motorCmd[i].q = msg->cmd[i].q;
        this->low_cmd_buffer.motorCmd[i].dq = msg->cmd[i].dq;
        this->low_cmd_buffer.motorCmd[i].tau = msg->cmd[i].tau;
        this->low_cmd_buffer.motorCmd[i].Kp = msg->cmd[i].Kp;
        this->low_cmd_buffer.motorCmd[i].Kd = msg->cmd[i].Kd;
        for (int j = 0; j < 3; j++)
            this->low_cmd_buffer.motorCmd[i].reserve[j] = msg->cmd[i].reserve[i];
    }
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
    this->publisher_init();
    this->server_init();
    this->subscriber_init();
}

/* The publisher in the base class is called by its constructor. This publisher_init is not related
to the implementation of the base class
*/
void UnitreeRos::publisher_init()
{
    if (this->ctrl_level == UNITREE_LEGGED_SDK::LOWLEVEL)
    {
        this->pose_estimation_publisher = this->ros_handle.advertise<sensor_msgs::Imu>(
            this->robot_namespace_ + "/imu_estimated", 1
        );
    }
}

void UnitreeRos::server_init()
{
    if (this->ctrl_level == UNITREE_LEGGED_SDK::HIGHLEVEL)
    {
        this->set_gaitType_service = this->ros_handle.advertiseService(
            this->robot_namespace_ + "/set_gaitType",
            &UnitreeRos::set_gaitType_srv_callback,
            this
        );
    }
}

void UnitreeRos::subscriber_init()
{
    if (this->ctrl_level == UNITREE_LEGGED_SDK::HIGHLEVEL)
    {
        this->high_twist_subscriber = this->ros_handle.subscribe(
            this->robot_namespace_ + "/body_motion_cmd",
            10,
            &UnitreeRos::high_twist_callback,
            this
        );
        this->foot_raise_height_subscriber = this->ros_handle.subscribe(
            this->robot_namespace_ + "/foot_raise_heigh_cmd",
            10,
            &UnitreeRos::foot_raise_height_callback,
            this
        );
        this->body_height_subscriber = this->ros_handle.subscribe(
            this->robot_namespace_ + "/body_height_cmd",
            10,
            &UnitreeRos::body_height_callback,
            this
        );
    } else if (this->ctrl_level == UNITREE_LEGGED_SDK::LOWLEVEL)
    {
        this->low_motor_subscriber = this->ros_handle.subscribe(
            this->robot_namespace_ + "/motor_cmd",
            10,
            &UnitreeRos::low_motor_callback,
            this
        );
    }
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
