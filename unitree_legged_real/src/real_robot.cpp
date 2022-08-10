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

void UnitreeRos::wirelessRemote_publish_callback(const ros::WallTimerEvent& event)
{
    unitree_legged_msgs::WirelessRemote ros_msg;
    xRockerBtnDataStruct _keyData;
    if (this->ctrl_level == UNITREE_LEGGED_SDK::HIGHLEVEL)
        memcpy(&_keyData, this->high_cmd_buffer.wirelessRemote, 40);
    else
        memcpy(&_keyData, this->low_cmd_buffer.wirelessRemote, 40);
    
    // transfer data from the struct to ros message
    for (int i (0); i < 2; i++) ros_msg.head[i] = _keyData.head[i];
    ros_msg.btn.components.R1 = _keyData.btn.components.R1;
    ros_msg.btn.components.L1 = _keyData.btn.components.L1;
    ros_msg.btn.components.start = _keyData.btn.components.start;
    ros_msg.btn.components.select = _keyData.btn.components.select;
    ros_msg.btn.components.R2 = _keyData.btn.components.R2;
    ros_msg.btn.components.L2 = _keyData.btn.components.L2;
    ros_msg.btn.components.F1 = _keyData.btn.components.F1;
    ros_msg.btn.components.F2 = _keyData.btn.components.F2;
    ros_msg.btn.components.A = _keyData.btn.components.A;
    ros_msg.btn.components.B = _keyData.btn.components.B;
    ros_msg.btn.components.X = _keyData.btn.components.X;
    ros_msg.btn.components.Y = _keyData.btn.components.Y;
    ros_msg.btn.components.up = _keyData.btn.components.up;
    ros_msg.btn.components.right = _keyData.btn.components.right;
    ros_msg.btn.components.down = _keyData.btn.components.down;
    ros_msg.btn.components.left = _keyData.btn.components.left;
    ros_msg.btn.value = _keyData.btn.value;
    ros_msg.lx = _keyData.lx;
    ros_msg.rx = _keyData.rx;
    ros_msg.ry = _keyData.ry;
    ros_msg.L2 = _keyData.L2;
    ros_msg.ly = _keyData.ly;
    for (int i (0); i < 16; i++) ros_msg.idle[i] = _keyData.idle[i];

    // publish the message
    this->wirelessRemote_publisher.publish(ros_msg);
}

UnitreeRos::UnitreeRos(
    const char* robot_namespace,
    const float udp_duration,
    uint8_t level,
    int power_protect_level,
    bool cmd_check,
    bool dryrun
):
    RosUdpHandler(robot_namespace, udp_duration, level, power_protect_level, cmd_check, dryrun)
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
    this->wirelessRemote_publisher = this->ros_handle.advertise<unitree_legged_msgs::WirelessRemote>(
        this->robot_namespace_ + "/wireless_remote", 1
    );
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

void UnitreeRos::timer_init()
{
    this->wirelessRemote_publish_timer = this->ros_handle.createWallTimer(
        ros::WallDuration(1. / this->timer_freq),
        &UnitreeRos::wirelessRemote_publish_callback,
        this
    );
}

int main(int argc, char **argv)
{
    if (argc != (5 + 3))
    {
        std::cout << "You must provide exactly 5 keyword arguments rather than " << argc - 3 << ", please use roslaunch rather than rosrun.";
        std::cout << std::endl;
        for (int i = 0; i < argc; i++) std::cout << argv[i] << std::endl;
        exit(-1);
    }

    // parse argument in a non-dynamic way, please use roslaunch!!!
    bool dryrun = (strcasecmp(argv[1], "true") == 0);
    const char* robot_namespace = argv[2];
    std::string udp_duration_s (argv[3]);
    float udp_duration = std::stof(udp_duration_s);
    bool cmd_check = (strcasecmp(argv[4], "true") == 0);
    bool use_low_level = (strcasecmp(argv[5], "low") == 0);
    uint8_t level = UNITREE_LEGGED_SDK::HIGHLEVEL;
    if (use_low_level) level = UNITREE_LEGGED_SDK::LOWLEVEL;

    // construct and initialize this ros node
    ros::init(argc, argv, robot_namespace);
    UnitreeRos unitree_ros_node(
        robot_namespace,
        udp_duration,
        level,
        1,
        cmd_check,
        dryrun
    );
    ros::spin();
    return 0;
}
