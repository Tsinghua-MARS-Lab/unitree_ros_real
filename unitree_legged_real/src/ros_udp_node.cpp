/**
 * @file ros_udp_node.cpp
 * @author Ziwen Zhuang (zhuangzw@sqz.ac.cn)
 * @brief The ros node run on NX to transport all robot proprioception and ros command
 * @version 0.1
 * @date 2022-07-26
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "ros_udp_node.h"

void RosUdpHandler::udp_init(uint8_t level)
{
    if (level == UNITREE_LEGGED_SDK::HIGHLEVEL)
    {
        this->udp.InitCmdData(this->high_cmd_buffer);
    }
    else if (level == UNITREE_LEGGED_SDK::LOWLEVEL)
    {
        this->udp.InitCmdData(this->low_cmd_buffer);
    }
}

void RosUdpHandler::udp_start()
{
    this->loop_udp_recv.start();
    this->loop_udp_send.start();
}

void RosUdpHandler::udp_send()
{
    if (this->ctrl_level == UNITREE_LEGGED_SDK::HIGHLEVEL)
        this->udp.SetSend(this->high_cmd_buffer);
    else if (this->ctrl_level == UNITREE_LEGGED_SDK::LOWLEVEL)
    {
        // Set to udp buffer after protection
        this->safe.PositionLimit(this->low_cmd_buffer);
        this->safe.PositionProtect(this->low_cmd_buffer, this->low_state_buffer);
        this->safe.PowerProtect(this->low_cmd_buffer, this->low_state_buffer, 1);
        this->udp.SetSend(this->low_cmd_buffer);
    }
    if (!this->dryrun_)
        this->udp.Send();
}

void RosUdpHandler::udp_recv()
{
    this->udp.Recv();
    if (this->ctrl_level == UNITREE_LEGGED_SDK::HIGHLEVEL)
    {
        this->udp.GetRecv(this->high_state_buffer);
        this->high_state_publish();
    }
    else if (this->ctrl_level == UNITREE_LEGGED_SDK::LOWLEVEL)
    {
        this->udp.GetRecv(this->low_state_buffer);
        this->low_state_publish();
    }
}

void RosUdpHandler::high_cmd_callback(const unitree_legged_msgs::HighCmd::ConstPtr &msg)
{
    this->high_cmd_buffer = rosMsg2Cmd(msg);
}

void RosUdpHandler::low_cmd_callback(const unitree_legged_msgs::LowCmd::ConstPtr &msg)
{
    this->low_cmd_buffer = rosMsg2Cmd(msg);
}

void RosUdpHandler::high_state_publish()
{
    unitree_legged_msgs::HighState ros_msg = state2rosMsg(this->high_state_buffer);
    this->state_publisher.publish(ros_msg);
}

void RosUdpHandler::low_state_publish()
{
    unitree_legged_msgs::LowState ros_msg = state2rosMsg(this->low_state_buffer);
    this->state_publisher.publish(ros_msg);
}

void RosUdpHandler::publisher_init()
{
    if (this->ctrl_level == UNITREE_LEGGED_SDK::HIGHLEVEL)
    {
        this->state_publisher = this->ros_handle.advertise<unitree_legged_msgs::HighState>(
            this->robot_namespace_ + "/high_state", 1
        );
        this->cmd_checker = this->ros_handle.advertise<unitree_legged_msgs::HighState>(
            this->robot_namespace_ + "/high_cmd_check", 1
        );
    }
    else if (this->ctrl_level == UNITREE_LEGGED_SDK::LOWLEVEL)
    {
        this->state_publisher = this->ros_handle.advertise<unitree_legged_msgs::LowState>(
            this->robot_namespace_ + "/low_state", 1
        );
        this->cmd_checker = this->ros_handle.advertise<unitree_legged_msgs::HighState>(
            this->robot_namespace_ + "/low_cmd_check", 1
        );
    }
}

void RosUdpHandler::subscriber_init()
{
    if (this->ctrl_level == UNITREE_LEGGED_SDK::HIGHLEVEL)
        this->cmd_subscriber = this->ros_handle.subscribe(
            this->robot_namespace_ + "/high_cmd",
            10,
            &RosUdpHandler::high_cmd_callback,
            this
        );
    else if (this->ctrl_level == UNITREE_LEGGED_SDK::LOWLEVEL)
        this->cmd_subscriber = this->ros_handle.subscribe(
            this->robot_namespace_ + "/low_cmd",
            10,
            &RosUdpHandler::low_cmd_callback,
            this
        );
}

RosUdpHandler::RosUdpHandler(
    const char* robot_namespace,
    const float udp_duration,
    uint8_t level,
    UNITREE_LEGGED_SDK::HighLevelType highControl,
    int power_protect_level,
    bool &dryrun
):
    robot_namespace_(robot_namespace),
    udp_duration_(udp_duration),
    ctrl_level(level),
    dryrun_(dryrun),
    udp(level, highControl),
    safe(UNITREE_LEGGED_SDK::LeggedType::A1),
    low_power_protect_level(power_protect_level),
    loop_udp_send("udp_send", udp_duration, 3, boost::bind(&RosUdpHandler::udp_send, this)),
    loop_udp_recv("udp_recv", udp_duration, 3, boost::bind(&RosUdpHandler::udp_recv, this))
{
    this->udp_init(level);
    this->publisher_init();
    this->subscriber_init();
    this->udp_start();
    ROS_DEBUG("RosUpdHandler constructed and initialized");
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
    RosUdpHandler ros_udp_handler(
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
