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
    {
        if (!this->high_cmd_get && !this->high_cmd_initialized) return;
        this->udp.SetSend(this->high_cmd_buffer);
        if (this->cmd_check)
        {
            unitree_legged_msgs::HighCmd ros_msg = Cmd2rosMsg(&this->high_cmd_buffer);
            this->cmd_checker.publish(ros_msg);
        }
    }
    else if (this->ctrl_level == UNITREE_LEGGED_SDK::LOWLEVEL)
    {
        if (!this->low_cmd_get && !this->low_cmd_initialized) return;
        // Set to udp buffer after protection
        this->safe.PositionLimit(this->low_cmd_buffer);
        this->safe.PositionProtect(this->low_cmd_buffer, this->low_state_buffer);
        this->safe.PowerProtect(this->low_cmd_buffer, this->low_state_buffer, this->low_power_protect_level);
        // Publish cmd_check if needed
        if (this->cmd_check)
        {
            unitree_legged_msgs::LowCmd ros_msg = Cmd2rosMsg(&this->low_cmd_buffer);
            this->cmd_checker.publish(ros_msg);
        }
        this->udp.SetSend(this->low_cmd_buffer);
    }
    if (!this->dryrun_)
        this->udp.Send();
}

void RosUdpHandler::udp_recv()
{
    int recv_result = this->udp.Recv();
    if (recv_result < 0)
    {
        ROS_ERROR_DELAYED_THROTTLE(0.1, "udp_recv error");
        return;
    }
    if (this->ctrl_level == UNITREE_LEGGED_SDK::HIGHLEVEL)
    {
        this->udp.GetRecv(this->high_state_buffer);
        this->high_state_publish();
        if (!this->high_cmd_initialized) this->high_cmd_init();
    }
    else if (this->ctrl_level == UNITREE_LEGGED_SDK::LOWLEVEL)
    {
        this->udp.GetRecv(this->low_state_buffer);
        this->low_state_publish();
        if (!this->low_cmd_initialized) this->low_cmd_init();
    }
}

void RosUdpHandler::high_cmd_init()
{
    this->high_cmd_buffer.levelFlag = this->high_state_buffer.levelFlag;
    this->high_cmd_buffer.commVersion = this->high_state_buffer.commVersion;
    this->high_cmd_buffer.robotID = this->high_state_buffer.robotID;
    this->high_cmd_buffer.SN = this->high_state_buffer.SN;
    this->high_cmd_buffer.bandWidth = this->high_state_buffer.bandWidth;
    this->high_cmd_buffer.mode = this->high_state_buffer.mode;
    this->high_cmd_buffer.gaitType = this->high_state_buffer.gaitType;
    this->high_cmd_buffer.speedLevel = 0;
    this->high_cmd_buffer.footRaiseHeight = this->high_state_buffer.footRaiseHeight;
    this->high_cmd_buffer.bodyHeight = this->high_state_buffer.bodyHeight;

    this->high_cmd_initialized = true;
}

void RosUdpHandler::low_cmd_init()
{
    this->low_cmd_buffer.levelFlag = this->low_state_buffer.levelFlag;
    this->low_cmd_buffer.commVersion = this->low_state_buffer.commVersion;
    this->low_cmd_buffer.robotID = this->low_state_buffer.robotID;
    this->low_cmd_buffer.SN = this->low_state_buffer.SN;
    this->low_cmd_buffer.bandWidth = this->low_state_buffer.bandWidth;
    for (int i (0); i < 12; i++)
    {
        this->low_cmd_buffer.motorCmd[i].mode = this->low_state_buffer.motorState[i].mode;
        this->low_cmd_buffer.motorCmd[i].q = this->low_state_buffer.motorState[i].q;
    }

    this->low_cmd_initialized = true;
}

void RosUdpHandler::high_cmd_callback(const unitree_legged_msgs::HighCmd::ConstPtr &msg)
{
    this->high_cmd_buffer = rosMsg2Cmd(msg);
    this->high_cmd_get = true;
}

void RosUdpHandler::low_cmd_callback(const unitree_legged_msgs::LowCmd::ConstPtr &msg)
{
    this->low_cmd_buffer = rosMsg2Cmd(msg);
    this->low_cmd_get = true;
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
        if (this->cmd_check)
            this->cmd_checker = this->ros_handle.advertise<unitree_legged_msgs::HighCmd>(
                this->robot_namespace_ + "/high_cmd_check", 1
            );
    }
    else if (this->ctrl_level == UNITREE_LEGGED_SDK::LOWLEVEL)
    {
        this->state_publisher = this->ros_handle.advertise<unitree_legged_msgs::LowState>(
            this->robot_namespace_ + "/low_state", 1
        );
        if (this->cmd_check)
            this->cmd_checker = this->ros_handle.advertise<unitree_legged_msgs::HighCmd>(
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
    int power_protect_level,
    bool cmd_check,
    bool dryrun
):
    robot_namespace_(robot_namespace),
    udp_duration_(udp_duration),
    ctrl_level(level),
    cmd_check(cmd_check),
    dryrun_(dryrun),
    // udp(level, highControl),
    safe(UNITREE_LEGGED_SDK::LeggedType::A1),
    low_power_protect_level(power_protect_level),
    loop_udp_send("udp_send", udp_duration, 3, boost::bind(&RosUdpHandler::udp_send, this)),
    loop_udp_recv("udp_recv", udp_duration, 3, boost::bind(&RosUdpHandler::udp_recv, this))
{
    if (level == UNITREE_LEGGED_SDK::HIGHLEVEL)
    {
        this->udp = UNITREE_LEGGED_SDK::UDP(
            8090,
            "192.168.123.161",
            8082,
            sizeof(UNITREE_LEGGED_SDK::HighCmd),
            sizeof(UNITREE_LEGGED_SDK::HighState)
        );
    } else if (level == UNITREE_LEGGED_SDK::LOWLEVEL)
    {
        this->udp = UNITREE_LEGGED_SDK::UDP(level);
    }
    this->udp_init(level);
    this->publisher_init();
    this->subscriber_init();
    this->udp_start();
    ROS_DEBUG("RosUpdHandler constructed and initialized");
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
    RosUdpHandler ros_udp_handler(
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
