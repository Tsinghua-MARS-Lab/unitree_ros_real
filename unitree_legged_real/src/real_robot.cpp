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

void UnitreeRos::get_params()
{
    
    this->ros_handle.param<bool>("publish_imu", this->publish_imu, false);
    this->ros_handle.param<bool>("publish_joint_state", this->publish_joint_state, false);
    this->ros_handle.param<bool>("publish_wirelessRemote", this->publish_wirelessRemote, false);
}

void UnitreeRos::set_params()
{
    this->ros_handle.setParam("joint_limits/hip_max", UNITREE_LEGGED_SDK::go1_Hip_max - this->position_protect_limit);
    this->ros_handle.setParam("joint_limits/hip_min", UNITREE_LEGGED_SDK::go1_Hip_min + this->position_protect_limit);
    this->ros_handle.setParam("joint_limits/thigh_max", UNITREE_LEGGED_SDK::go1_Thigh_max - this->position_protect_limit);
    this->ros_handle.setParam("joint_limits/thigh_min", UNITREE_LEGGED_SDK::go1_Thigh_min + this->position_protect_limit);
    this->ros_handle.setParam("joint_limits/calf_max", UNITREE_LEGGED_SDK::go1_Calf_max - this->position_protect_limit);
    this->ros_handle.setParam("joint_limits/calf_min", UNITREE_LEGGED_SDK::go1_Calf_min + this->position_protect_limit);
}

void UnitreeRos::low_motor_callback(const unitree_legged_msgs::LegsCmd::ConstPtr &msg)
{
    this->cmd_refresh_time = ros::Time::now();
    pthread_mutex_lock(&this->low_cmd_mutex);
    for (int i = 0; i < 12; i++)
    {
        this->low_cmd_buffer.motorCmd[i].mode = msg->cmd[i].mode;
        this->low_cmd_buffer.motorCmd[i].q = msg->cmd[i].q;
        this->low_cmd_buffer.motorCmd[i].dq = msg->cmd[i].dq;
        this->low_cmd_buffer.motorCmd[i].tau = msg->cmd[i].tau;
        this->low_cmd_buffer.motorCmd[i].Kp = msg->cmd[i].Kp;
        this->low_cmd_buffer.motorCmd[i].Kd = msg->cmd[i].Kd;
        for (int j = 0; j < 3; j++)
            this->low_cmd_buffer.motorCmd[i].reserve[j] = msg->cmd[i].reserve[j];
    }
    pthread_mutex_unlock(&this->low_cmd_mutex);
}

void UnitreeRos::imu_publish_callback(const ros::TimerEvent& event){
    sensor_msgs::Imu ros_msg;
    UNITREE_LEGGED_SDK::IMU *imu_ptr;
    if (this->ctrl_level == UNITREE_LEGGED_SDK::LOWLEVEL)
        imu_ptr = &this->low_state_buffer.imu;
    
    // fill ros message
    ros_msg.header.seq = this->imu_publish_seq++;
    ros_msg.header.stamp = ros::Time::now();
    ros_msg.header.frame_id = "imu_link";
    ros_msg.orientation.x = (double)imu_ptr->quaternion[1];
    ros_msg.orientation.y = (double)imu_ptr->quaternion[2];
    ros_msg.orientation.z = (double)imu_ptr->quaternion[3];
    ros_msg.orientation.w = (double)imu_ptr->quaternion[0];
    // ros_msg.orientation_covariance unknown
    ros_msg.angular_velocity.x = (double)imu_ptr->gyroscope[0];
    ros_msg.angular_velocity.y = (double)imu_ptr->gyroscope[1];
    ros_msg.angular_velocity.z = (double)imu_ptr->gyroscope[2];
    // ros_msg.angular_velocity_covariance unknown
    ros_msg.linear_acceleration.x = (double)imu_ptr->accelerometer[0];
    ros_msg.linear_acceleration.y = (double)imu_ptr->accelerometer[1];
    ros_msg.linear_acceleration.z = (double)imu_ptr->accelerometer[2];
    // ros_mag.linear_acceleration_covariance unknown

    this->imu_publisher.publish(ros_msg);
}

void UnitreeRos::wirelessRemote_publish_callback(const ros::TimerEvent& event)
{
    unitree_legged_msgs::WirelessRemote ros_msg;
    unitree_legged_msgs::WirelessRemote _keyData = Bytes2rosMsg(this->low_state_buffer.wirelessRemote);
    
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

void UnitreeRos::joint_state_publish_callback(const ros::TimerEvent& event)
{
    sensor_msgs::JointState ros_msg;
    ros_msg.header.seq = this->joint_state_publish_seq++;
    ros_msg.header.stamp = ros::Time::now();
    ros_msg.header.frame_id = "base";
    // 12 joint names
    ros_msg.name.push_back(std::string("FR_hip_joint"));
    ros_msg.name.push_back(std::string("FR_thigh_joint"));
    ros_msg.name.push_back(std::string("FR_calf_joint"));
    ros_msg.name.push_back(std::string("FL_hip_joint"));
    ros_msg.name.push_back(std::string("FL_thigh_joint"));
    ros_msg.name.push_back(std::string("FL_calf_joint"));
    ros_msg.name.push_back(std::string("RR_hip_joint"));
    ros_msg.name.push_back(std::string("RR_thigh_joint"));
    ros_msg.name.push_back(std::string("RR_calf_joint"));
    ros_msg.name.push_back(std::string("RL_hip_joint"));
    ros_msg.name.push_back(std::string("RL_thigh_joint"));
    ros_msg.name.push_back(std::string("RL_calf_joint"));
    // 12 joint positions
    for (int i (0); i < 12; i++) ros_msg.position.push_back(this->low_state_buffer.motorState[i].q);
    // 12 joint velocites
    for (int i (0); i < 12; i++) ros_msg.velocity.push_back(this->low_state_buffer.motorState[i].dq);
    // 12 joint effort
    for (int i (0); i < 12; i++) ros_msg.effort.push_back(this->low_state_buffer.motorState[i].tauEst);

    this->joint_state_publisher.publish(ros_msg);
}

UnitreeRos::UnitreeRos(
    std::string robot_namespace,
    const float udp_duration,
    uint8_t level,
    ros::NodeHandle nh
):
    RosUdpHandler(
        robot_namespace,
        udp_duration,
        level,
        nh
    )
{
    this->get_params();
    this->set_params();
    this->publisher_init();
    this->server_init();
    this->subscriber_init();
    this->timer_init();
}

UnitreeRos::~UnitreeRos()
{
    for (int i (0); i < 12; i++) this->low_cmd_buffer.motorCmd[i].mode = 0;
    this->udp.SetSend(this->low_cmd_buffer);
    this->udp.Send();
    std::cout << "UnitreeRos desctructed" << std::endl;
}

/* The publisher in the base class is called by its constructor. This publisher_init is not related
to the implementation of the base class
*/
void UnitreeRos::publisher_init()
{
    if (this->ctrl_level == UNITREE_LEGGED_SDK::LOWLEVEL)
    {
        this->position_limit_publisher = this->ros_handle.advertise<std_msgs::Float32MultiArray>(
            "position_limit", 1
        );
        if (this->publish_joint_state)
            this->joint_state_publisher = this->ros_handle.advertise<sensor_msgs::JointState>(
                "/joint_states", 1
            );
    }
    if (this->publish_imu)
        this->imu_publisher = this->ros_handle.advertise<sensor_msgs::Imu>(
            "imu", 1
        );
    if (this->publish_wirelessRemote)
        this->wirelessRemote_publisher = this->ros_handle.advertise<unitree_legged_msgs::WirelessRemote>(
            "wireless_remote", 1
        );
}

void UnitreeRos::server_init()
{
}

void UnitreeRos::subscriber_init()
{
    if (this->ctrl_level == UNITREE_LEGGED_SDK::LOWLEVEL)
    {
        this->low_motor_subscriber = this->ros_handle.subscribe(
            "legs_cmd",
            10,
            &UnitreeRos::low_motor_callback,
            this
        );
    }
}

void UnitreeRos::timer_init()
{
    if (this->ctrl_level == UNITREE_LEGGED_SDK::LOWLEVEL)
    {
        if (this->publish_joint_state)
            this->joint_state_publish_timer = this->ros_handle.createTimer(
                ros::Duration(1. / this->joint_state_publish_freq),
                &UnitreeRos::joint_state_publish_callback,
                this
            );
    }
    if (this->publish_imu)
        this->imu_publish_timer = this->ros_handle.createTimer(
            ros::Duration(1. / this->imu_publish_freq),
            &UnitreeRos::imu_publish_callback,
            this
        );
    if (this->publish_wirelessRemote)
        this->wirelessRemote_publish_timer = this->ros_handle.createTimer(
            ros::Duration(1. / this->timer_freq),
            &UnitreeRos::wirelessRemote_publish_callback,
            this
        );
}

int main(int argc, char **argv)
{
    std::string robot_namespace (argv[1]);
    ros::init(argc, argv, robot_namespace);
    ros::NodeHandle nh ("~");

    // get configuration using rosparam, use ros launch to start this node!!!
    float udp_duration; nh.param<float>("udp_duration", udp_duration, 0.01);
    std::string ctrl_level_s; nh.getParam("ctrl_level", ctrl_level_s);
    bool use_low_level = (ctrl_level_s.compare("low") == 0);
    if (!use_low_level) ROS_DEBUG("You set the ctrl_level to low, which is not supported by this package");
    uint8_t level = UNITREE_LEGGED_SDK::LOWLEVEL;

    // construct and initialize this ros node
    UnitreeRos unitree_ros_node(
        robot_namespace,
        udp_duration,
        level,
        nh
    );
    ros::spin();
    return 0;
}
