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
#include "unitree_legged_sdk/a1_const.h" // NOTE: this make the file deeply coupled to A1 robot, not other types

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

void UnitreeRos::wirelessRemote_publish_callback(const ros::TimerEvent& event)
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

void UnitreeRos::protect_limit_publish_callback(const ros::TimerEvent& event)
{
    std_msgs::Float32MultiArray position_limit_msg;
    position_limit_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    position_limit_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    position_limit_msg.layout.dim[0].size = 2; // (high, level)
    position_limit_msg.layout.dim[0].stride = 2 * 12;
    position_limit_msg.layout.dim[1].size = 12; // (hip0, thigh0, calf, hip1, thigh1, calf1, ...) 12 in total
    position_limit_msg.layout.dim[1].stride = 12;
    
    // NOTE: the following code is only workable on A1 robot, not other model
    for (int i (0); i < 12; i+=3)
    {
        position_limit_msg.data.push_back(UNITREE_LEGGED_SDK::a1_Hip_min + this->position_protect_limit);
        position_limit_msg.data.push_back(UNITREE_LEGGED_SDK::a1_Thigh_min + this->position_protect_limit);
        position_limit_msg.data.push_back(UNITREE_LEGGED_SDK::a1_Calf_min + this->position_protect_limit);
    }
    for (int i (0); i < 12; i+=3)
    {
        position_limit_msg.data.push_back(UNITREE_LEGGED_SDK::a1_Hip_max - this->position_protect_limit);
        position_limit_msg.data.push_back(UNITREE_LEGGED_SDK::a1_Thigh_max - this->position_protect_limit);
        position_limit_msg.data.push_back(UNITREE_LEGGED_SDK::a1_Calf_max - this->position_protect_limit);
    }
    
    // publish the limit data. NOTE: the data unit is radian
    this->position_limit_publisher.publish(position_limit_msg);
}

UnitreeRos::UnitreeRos(
    std::string robot_namespace,
    const float udp_duration,
    uint8_t level,
    float position_protect_limit,
    int power_protect_level,
    bool cmd_check,
    bool dryrun
):
    RosUdpHandler(robot_namespace, udp_duration, level, position_protect_limit, power_protect_level, cmd_check, dryrun)
{
    this->publisher_init();
    this->server_init();
    this->subscriber_init();
    this->timer_init();
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
        this->position_limit_publisher = this->ros_handle.advertise<std_msgs::Float32MultiArray>(
            this->robot_namespace_ + "/position_limit", 1
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
    this->wirelessRemote_publish_timer = this->ros_handle.createTimer(
        ros::Duration(1. / this->timer_freq),
        &UnitreeRos::wirelessRemote_publish_callback,
        this
    );
    this->protect_limit_publish_timer = this->ros_handle.createTimer(
        ros::Duration(1. / this->timer_freq),
        &UnitreeRos::protect_limit_publish_callback,
        this
    );
}

int main(int argc, char **argv)
{
    std::string robot_namespace (argv[1]);
    ros::init(argc, argv, robot_namespace);
    ros::NodeHandle nh ("~");

    // get configuration using rosparam, use ros launch to start this node!!!
    bool dryrun; nh.param<bool>("dryrun", dryrun, true);
    float udp_duration; nh.param<float>("udp_duration", udp_duration, 0.01);
    bool cmd_check; nh.param<bool>("cmd_check", cmd_check, true);
    std::string ctrl_level_s; nh.getParam("ctrl_level", ctrl_level_s);
    bool use_low_level = (ctrl_level_s.compare("low") == 0);
    uint8_t level = UNITREE_LEGGED_SDK::HIGHLEVEL;
    if (use_low_level) level = UNITREE_LEGGED_SDK::LOWLEVEL;
    float position_protect_limit; nh.param<float>("position_protect_limit", position_protect_limit, 0.087);
    int power_protect_level; nh.param<int>("power_protect_level", power_protect_level, 1);

    // construct and initialize this ros node
    UnitreeRos unitree_ros_node(
        robot_namespace,
        udp_duration,
        level,
        position_protect_limit,
        power_protect_level,
        cmd_check,
        dryrun
    );
    ros::spin();
    return 0;
}
