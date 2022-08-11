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
        this->low_cmd_motorCmd_init();
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
        this->udp.SetSend(this->high_cmd_buffer);
        if (this->cmd_check)
        {
            unitree_legged_msgs::HighCmd ros_msg = Cmd2rosMsg(&this->high_cmd_buffer);
            this->cmd_checker.publish(ros_msg);
        }
    }
    else if (this->ctrl_level == UNITREE_LEGGED_SDK::LOWLEVEL)
    {
        // Set to udp buffer after protection
        this->safe.PositionLimit(this->low_cmd_buffer);
        this->safe.PositionProtect(this->low_cmd_buffer, this->low_state_buffer, this->position_protect_limit);
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
        ROS_ERROR_DELAYED_THROTTLE(1, "udp_recv error: %i", recv_result);
        return;
    }
    if (this->ctrl_level == UNITREE_LEGGED_SDK::HIGHLEVEL)
    {
        this->udp.GetRecv(this->high_state_buffer);
        this->high_state_publish();
        if (!this->high_cmd_metadata_get) this->high_cmd_metadata_update();
    }
    else if (this->ctrl_level == UNITREE_LEGGED_SDK::LOWLEVEL)
    {
        this->udp.GetRecv(this->low_state_buffer);
        this->low_state_publish();
        if (!this->low_cmd_metadata_get) this->low_cmd_metadata_update();
    }
}

// NOTE: this function is one of the reason that this node can only work on Unitree A1 robot.
void RosUdpHandler::low_cmd_motorCmd_init()
{
    // set mode
    for (int i(0); i < 12; i++) this->low_cmd_buffer.motorCmd[i].mode = 10;
    // set q (position)
    this->low_cmd_buffer.motorCmd[UNITREE_LEGGED_SDK::FR_0].q = -0.3;
    this->low_cmd_buffer.motorCmd[UNITREE_LEGGED_SDK::FR_1].q = 0.9;
    this->low_cmd_buffer.motorCmd[UNITREE_LEGGED_SDK::FR_2].q = -1.6;
    this->low_cmd_buffer.motorCmd[UNITREE_LEGGED_SDK::FL_0].q = 0.3;
    this->low_cmd_buffer.motorCmd[UNITREE_LEGGED_SDK::FL_1].q = 0.9;
    this->low_cmd_buffer.motorCmd[UNITREE_LEGGED_SDK::FL_2].q = -1.6;
    this->low_cmd_buffer.motorCmd[UNITREE_LEGGED_SDK::RR_0].q = -0.3;
    this->low_cmd_buffer.motorCmd[UNITREE_LEGGED_SDK::RR_1].q = 0.9;
    this->low_cmd_buffer.motorCmd[UNITREE_LEGGED_SDK::RR_2].q = -1.6;
    this->low_cmd_buffer.motorCmd[UNITREE_LEGGED_SDK::RL_0].q = 0.3;
    this->low_cmd_buffer.motorCmd[UNITREE_LEGGED_SDK::RL_1].q = 0.9;
    this->low_cmd_buffer.motorCmd[UNITREE_LEGGED_SDK::RL_2].q = -1.6;
    // set dq (velocity)
    for (int i(0); i < 12; i++) this->low_cmd_buffer.motorCmd[i].dq = 0.;
    // set tau (torque)
    this->low_cmd_buffer.motorCmd[UNITREE_LEGGED_SDK::FR_0].tau = +this->low_cmd_default_tau; //
    this->low_cmd_buffer.motorCmd[UNITREE_LEGGED_SDK::FR_1].tau = -this->low_cmd_default_tau;
    this->low_cmd_buffer.motorCmd[UNITREE_LEGGED_SDK::FR_2].tau = +this->low_cmd_default_tau;
    this->low_cmd_buffer.motorCmd[UNITREE_LEGGED_SDK::FL_0].tau = -this->low_cmd_default_tau; //
    this->low_cmd_buffer.motorCmd[UNITREE_LEGGED_SDK::FL_1].tau = -this->low_cmd_default_tau;
    this->low_cmd_buffer.motorCmd[UNITREE_LEGGED_SDK::FL_2].tau = -this->low_cmd_default_tau;
    this->low_cmd_buffer.motorCmd[UNITREE_LEGGED_SDK::RR_0].tau = +this->low_cmd_default_tau; //
    this->low_cmd_buffer.motorCmd[UNITREE_LEGGED_SDK::RR_1].tau = -this->low_cmd_default_tau;
    this->low_cmd_buffer.motorCmd[UNITREE_LEGGED_SDK::RR_2].tau = +this->low_cmd_default_tau;
    this->low_cmd_buffer.motorCmd[UNITREE_LEGGED_SDK::RL_0].tau = -this->low_cmd_default_tau; //
    this->low_cmd_buffer.motorCmd[UNITREE_LEGGED_SDK::RL_1].tau = -this->low_cmd_default_tau;
    this->low_cmd_buffer.motorCmd[UNITREE_LEGGED_SDK::RL_2].tau = -this->low_cmd_default_tau;
    // set Kp
    for (int i(0); i < 12; i++) this->low_cmd_buffer.motorCmd[i].Kp = 40.0;
    // set Kd
    for (int i(0); i < 12; i++) this->low_cmd_buffer.motorCmd[i].Kd = 1.0;
}

void RosUdpHandler::high_cmd_metadata_update()
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

    this->high_cmd_metadata_get = true;
}

void RosUdpHandler::low_cmd_metadata_update()
{
    this->low_cmd_buffer.levelFlag = this->low_state_buffer.levelFlag;
    this->low_cmd_buffer.commVersion = this->low_state_buffer.commVersion;
    this->low_cmd_buffer.robotID = this->low_state_buffer.robotID;
    this->low_cmd_buffer.SN = this->low_state_buffer.SN;
    this->low_cmd_buffer.bandWidth = this->low_state_buffer.bandWidth;

    this->low_cmd_metadata_get = true;
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
            this->cmd_checker = this->ros_handle.advertise<unitree_legged_msgs::LowCmd>(
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
    std::string robot_namespace,
    const float udp_duration,
    uint8_t level,
    float position_protect_limit,
    int power_protect_level,
    bool cmd_check,
    bool dryrun
):
    safe(UNITREE_LEGGED_SDK::LeggedType::A1),
    // udp(8091, "192.168.123.161", 8082, sizeof(UNITREE_LEGGED_SDK::HighCmd), sizeof(UNITREE_LEGGED_SDK::HighState)),
    udp(level),
    robot_namespace_(robot_namespace),
    udp_duration_(udp_duration),
    ctrl_level(level),
    position_protect_limit(position_protect_limit),
    low_power_protect_level(power_protect_level),
    cmd_check(cmd_check),
    dryrun_(dryrun),
    loop_udp_send("udp_send", udp_duration, 3, boost::bind(&RosUdpHandler::udp_send, this)),
    loop_udp_recv("udp_recv", udp_duration, 3, boost::bind(&RosUdpHandler::udp_recv, this))
{
    // if (level == UNITREE_LEGGED_SDK::HIGHLEVEL)
    // {
    //     ROS_INFO_ONCE("Setting up UDP with HIGHLEVEL control");
    //     this->udp = UNITREE_LEGGED_SDK::UDP(
    //         8080,
    //         "192.168.123.161",
    //         8082,
    //         sizeof(UNITREE_LEGGED_SDK::HighCmd),
    //         sizeof(UNITREE_LEGGED_SDK::HighState)
    //     );
    // } else if (level == UNITREE_LEGGED_SDK::LOWLEVEL)
    // {
    //     ROS_INFO_ONCE("Setting up UDP with LOWLEVEL control");
    //     this->udp = UNITREE_LEGGED_SDK::UDP(level);
    // }
    this->udp_init(level);
    ROS_INFO("Udp initialized");
    this->publisher_init();
    this->subscriber_init();
    this->udp_start();
    ROS_INFO("RosUdpHandler constructed and started");
}

int main(int argc, char **argv)
{
    std::string robot_namespace (argv[1]);
    ros::init(argc, argv, robot_namespace);
    ros::NodeHandle nh ("~");

    // get configuration using rosparam, use ros launch to start this node!!!
    bool dryrun; nh.param<bool>("dryrun", dryrun, true);
    float udp_duration; nh.param<float>("udp_duration", udp_duration, 0.01);
    bool cmd_check; nh.getParam("cmd_check", cmd_check);
    std::string ctrl_level_s; nh.getParam("ctrl_level", ctrl_level_s);
    bool use_low_level = (ctrl_level_s.compare("low") == 0);
    uint8_t level = UNITREE_LEGGED_SDK::HIGHLEVEL;
    if (use_low_level) level = UNITREE_LEGGED_SDK::LOWLEVEL;
    float position_protect_limit; nh.param<float>("position_protect_limit", position_protect_limit, 0.087);
    int power_protect_level; nh.param<int>("power_protect_level", power_protect_level, 1);

    // construct and initialize this ros node
    RosUdpHandler ros_udp_handler(
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
