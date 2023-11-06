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

void RosUdpHandler::get_params()
{
    
    this->ros_handle.getParam("cmd_check", this->cmd_check);
    this->ros_handle.param<float>("position_protect_limit", this->position_protect_limit, 0.087);
    this->ros_handle.param<int>("power_protect_level", this->power_protect_level, 1);
    this->ros_handle.param<bool>("dryrun", this->dryrun, true);
    this->ros_handle.param<bool>("start_stand", this->start_stand, true);
    this->ros_handle.param<float>("cmd_lost_timelimit", this->cmd_lost_timelimit, 0.02);
    this->ros_handle.param<bool>("freeze_lost", this->freeze_lost, false);
    this->ros_handle.param<float>("safety_guard_duration", this->safety_guard_duration, 0.02);
    this->ros_handle.param<float>("torque_protect_limit", this->torque_protect_limit, 33.5);
    this->ros_handle.param<float>("torque_protect_limit_calf", this->torque_protect_limit_calf, this->torque_protect_limit);
    this->ros_handle.param<float>("pitch_protect_limit", this->pitch_protect_limit, -1.);
    this->ros_handle.param<float>("roll_protect_limit", this->roll_protect_limit, -1.);
    this->ros_handle.param<float>("R2_press_protect", this->R2_press_protect, true);

    if (this->start_stand) ROS_INFO("Motor will be initialized to mode 10, please put the leg in stand positions.");
    else ROS_INFO("Motor will be initialized to mode 0, please put the robot on the ground or hang up.");
}

void RosUdpHandler::set_params()
{
    this->ros_handle.setParam("cmd/PosStopF", UNITREE_LEGGED_SDK::PosStopF);
    this->ros_handle.setParam("cmd/VelStopF", UNITREE_LEGGED_SDK::VelStopF);
}

void RosUdpHandler::udp_init(uint8_t level)
{
    if (level == UNITREE_LEGGED_SDK::LOWLEVEL)
    {
        this->udp.InitCmdData(this->low_cmd_buffer);
    }
}

void RosUdpHandler::udp_start()
{
    if (this->ctrl_level == UNITREE_LEGGED_SDK::LOWLEVEL)
    {
        this->set_default_low_cmd();
        this->udp.SetSend(this->low_cmd_buffer);
        this->udp.Send();
    }
    this->loop_udp_recv.start();
    this->loop_udp_send.start();
    this->cmd_refresh_time = ros::Time::now();
    this->loop_udp_translate.start();
}

void RosUdpHandler::udp_send()
{
    if (!this->dryrun && this->robot_safe)
        this->udp.Send();
}

void RosUdpHandler::udp_recv()
{
    this->udp_recv_result = this->udp.Recv();
}

void RosUdpHandler::udp_translate()
{
    // Receive part
    if (this->udp_recv_result < 0)
    {
        ROS_ERROR_DELAYED_THROTTLE(0.1, "udp_recv error: %i, message translate might lost once.", this->udp_recv_result);
    }
    else if (this->ctrl_level == UNITREE_LEGGED_SDK::LOWLEVEL)
    {
        pthread_mutex_lock(&this->low_state_mutex);
        this->udp.GetRecv(this->low_state_buffer);
        pthread_mutex_unlock(&this->low_state_mutex);
        this->low_state_publish();
        if (!this->low_cmd_metadata_get) this->low_cmd_metadata_update();
    }

    // Send part
    if (this->ctrl_level == UNITREE_LEGGED_SDK::LOWLEVEL)
    {
        // Set to udp buffer after protection
        this->safe.PositionLimit(this->low_cmd_buffer);
        if (this->low_cmd_metadata_get)
        {   // means the los_state_buffer is valid, the following protection should be running.
            if (this->position_protect_limit > 0.)
                this->safe.PositionProtect(this->low_cmd_buffer, this->low_state_buffer, this->position_protect_limit);
            this->safe.PowerProtect(this->low_cmd_buffer, this->low_state_buffer, this->power_protect_level);
        }
        // Publish cmd_check if needed
        if (this->cmd_check)
        {
            unitree_legged_msgs::LowCmd ros_msg = Cmd2rosMsg(&this->low_cmd_buffer);
            this->cmd_checker.publish(ros_msg);
        }
        pthread_mutex_lock(&this->low_cmd_mutex);
        if (!this->robot_safe)
        {
            for (int i (0); i < 12; i++) this->low_cmd_buffer.motorCmd[i].mode = 0;
        }
        this->udp.SetSend(this->low_cmd_buffer);
        pthread_mutex_unlock(&this->low_cmd_mutex);
        if (!this->robot_safe)
        {
            ROS_FATAL("Robot unsafe, exit the program.");
            if (!this->dryrun) this->udp.Send();
            ros::shutdown();
        }
    }
}

// NOTE: this function is one of the reason that this node can only work on Unitree A1 robot.
void RosUdpHandler::set_default_low_cmd()
{
    // set mode
    if (this->start_stand)
        for (int i(0); i < 12; i++) this->low_cmd_buffer.motorCmd[i].mode = 10;
    else
        for (int i(0); i < 12; i++) this->low_cmd_buffer.motorCmd[i].mode = 0;
    // set q (position)
    this->low_cmd_buffer.motorCmd[UNITREE_LEGGED_SDK::FR_0].q = -0.1;
    this->low_cmd_buffer.motorCmd[UNITREE_LEGGED_SDK::FR_1].q = 0.8;
    this->low_cmd_buffer.motorCmd[UNITREE_LEGGED_SDK::FR_2].q = -1.5;
    this->low_cmd_buffer.motorCmd[UNITREE_LEGGED_SDK::FL_0].q = 0.1;
    this->low_cmd_buffer.motorCmd[UNITREE_LEGGED_SDK::FL_1].q = 0.8;
    this->low_cmd_buffer.motorCmd[UNITREE_LEGGED_SDK::FL_2].q = -1.5;
    this->low_cmd_buffer.motorCmd[UNITREE_LEGGED_SDK::RR_0].q = -0.1;
    this->low_cmd_buffer.motorCmd[UNITREE_LEGGED_SDK::RR_1].q = 1.0;
    this->low_cmd_buffer.motorCmd[UNITREE_LEGGED_SDK::RR_2].q = -1.5;
    this->low_cmd_buffer.motorCmd[UNITREE_LEGGED_SDK::RL_0].q = 0.1;
    this->low_cmd_buffer.motorCmd[UNITREE_LEGGED_SDK::RL_1].q = 1.0;
    this->low_cmd_buffer.motorCmd[UNITREE_LEGGED_SDK::RL_2].q = -1.5;
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
    for (int i(0); i < 12; i++) this->low_cmd_buffer.motorCmd[i].Kp = 20.0;
    // set Kd
    for (int i(0); i < 12; i++) this->low_cmd_buffer.motorCmd[i].Kd = 0.5;
}

void RosUdpHandler::low_cmd_metadata_update()
{
    this->low_cmd_buffer.head[0] = this->low_state_buffer.head[0];
    this->low_cmd_buffer.head[1] = this->low_state_buffer.head[1];
    this->low_cmd_buffer.levelFlag = this->low_state_buffer.levelFlag;
    this->low_cmd_buffer.frameReserve = this->low_state_buffer.frameReserve;

    this->low_cmd_buffer.SN[0] = this->low_state_buffer.SN[0];
    this->low_cmd_buffer.SN[1] = this->low_state_buffer.SN[1];
    this->low_cmd_buffer.version[0] = this->low_state_buffer.version[0];
    this->low_cmd_buffer.version[1] = this->low_state_buffer.version[1];
    this->low_cmd_buffer.bandWidth = this->low_state_buffer.bandWidth;

    this->low_cmd_metadata_get = true;
}

void RosUdpHandler::low_cmd_callback(const unitree_legged_msgs::LowCmd::ConstPtr &msg)
{
    pthread_mutex_lock(&this->low_cmd_mutex);
    this->low_cmd_buffer = rosMsg2Cmd(msg);
    pthread_mutex_unlock(&this->low_cmd_mutex);
    this->cmd_refresh_time = ros::Time::now();
}

void RosUdpHandler::cmd_lost_check_callback(const ros::TimerEvent& event)
{
    ros::Duration time_elapsed = ros::Time::now() - this->cmd_refresh_time;
    if (this->cmd_lost_timelimit > 0
        && time_elapsed.toSec() > this->cmd_lost_timelimit
        && !this->cmd_refresh_time.is_zero()
    )
    { // assuming command lost, set low_cmd_buffer
        if (this->ctrl_level == UNITREE_LEGGED_SDK::LOWLEVEL && this->freeze_lost)
        {
            // Send command to freeze the robot
            pthread_mutex_lock(&this->low_cmd_mutex);
            for (int i(0); i < 12; i++)
            {
                this->low_cmd_buffer.motorCmd[i].q = this->low_state_buffer.motorState[i].q;
                this->low_cmd_buffer.motorCmd[i].dq = 0.;
                this->low_cmd_buffer.motorCmd[i].tau = 0.;
                this->low_cmd_buffer.motorCmd[i].Kp = this->safe_Kp;
                this->low_cmd_buffer.motorCmd[i].Kd = this->safe_Kd;
            }
            pthread_mutex_unlock(&this->low_cmd_mutex);
            // overwrite refresh time as a marker
            ROS_WARN_THROTTLE(30, "Cmd lost, freeze the robot. DO NOT use this as a normal operation.");
            this->cmd_refresh_time.nsec = 0;
            this->cmd_refresh_time.sec = 0;
        } else if (this->ctrl_level == UNITREE_LEGGED_SDK::LOWLEVEL && (!this->freeze_lost)) {
            // keep sending current position as position control to perform motor damping.
            pthread_mutex_lock(&this->low_cmd_mutex);
            for (int i(0); i < 12; i++)
            {
                this->low_cmd_buffer.motorCmd[i].q = this->low_state_buffer.motorState[i].q;
                this->low_cmd_buffer.motorCmd[i].dq = 0.;
                this->low_cmd_buffer.motorCmd[i].tau = 0.;
                this->low_cmd_buffer.motorCmd[i].Kp = this->safe_Kp;
                this->low_cmd_buffer.motorCmd[i].Kd = this->safe_Kd;
            }
            pthread_mutex_unlock(&this->low_cmd_mutex);
            ROS_WARN_THROTTLE(30., "Cmd lost, position control as motor damping. DO NOT use this as a normal operation.");
        }
    }
}

void RosUdpHandler::safety_guard_callback(const ros::TimerEvent& event)
{
    /* Shutdown the process if the robot is not safe to prevent the motor from burning, even
    it is not the best way to protect the robot.
    */
    if (this->ctrl_level == UNITREE_LEGGED_SDK::LOWLEVEL)
    {
        if (this->pitch_protect_limit > 0. && (abs(this->low_state_buffer.imu.rpy[1]) > this->pitch_protect_limit))
        {
            this->robot_safe = false;
            ROS_ERROR("Pitch out of limit to %f, robot unsafe.", this->low_state_buffer.imu.rpy[1]);
            return;
        }
        if (this->roll_protect_limit > 0. && (abs(this->low_state_buffer.imu.rpy[0]) > this->roll_protect_limit))
        {
            this->robot_safe = false;
            ROS_ERROR("Roll out of limit to %f, robot unsafe.", this->low_state_buffer.imu.rpy[0]);
            return;
        }
        if (this->torque_protect_limit > 0.)
        {
            for (int i(0); i < 4; i++)
            {
                for (int j(0); j < 2; j++)
                    if (abs(this->low_state_buffer.motorState[i*3 + j].tauEst) > this->torque_protect_limit)
                    {
                        this->robot_safe = false;
                        ROS_ERROR("Torque out of limit to %f at joint %d, robot unsafe.", this->low_state_buffer.motorState[i*3 + j].tauEst, i);
                        return;
                    }
                if (abs(this->low_state_buffer.motorState[i*3 + 2].tauEst) > this->torque_protect_limit_calf)
                {
                    this->robot_safe = false;
                    ROS_ERROR("Torque out of limit to %f at joint %d, robot unsafe.", this->low_state_buffer.motorState[i*3 + 2].tauEst, i);
                    return;
                }
            }
        }
        if (this->low_state_buffer.bms.SOC < 20.)
        {
            ROS_WARN_THROTTLE(10, "Battery lower than 20%, please charge the battery as soon as possible!");
        }
        if (this->R2_press_protect)
        {
            unitree_legged_msgs::WirelessRemote remote_data = Bytes2rosMsg(this->low_state_buffer.wirelessRemote);
            if ((int)remote_data.btn.components.R2 == 1)
            {
                this->robot_safe = false;
                ROS_ERROR("R2 button pressed, robot unsafe.");
                return;
            }
        }
    }
}

void RosUdpHandler::low_state_publish()
{
    pthread_mutex_lock(&this->low_state_mutex);
    unitree_legged_msgs::LowState ros_msg = state2rosMsg(this->low_state_buffer);
    pthread_mutex_unlock(&this->low_state_mutex);
    this->state_publisher.publish(ros_msg);
}

void RosUdpHandler::publisher_init()
{
    if (this->ctrl_level == UNITREE_LEGGED_SDK::LOWLEVEL)
    {
        this->state_publisher = this->ros_handle.advertise<unitree_legged_msgs::LowState>(
            "low_state", 10
        );
        if (this->cmd_check)
            this->cmd_checker = this->ros_handle.advertise<unitree_legged_msgs::LowCmd>(
                "low_cmd_check", 10
            );
    }
}

void RosUdpHandler::subscriber_init()
{
    if (this->ctrl_level == UNITREE_LEGGED_SDK::LOWLEVEL)
        this->cmd_subscriber = this->ros_handle.subscribe(
            "low_cmd",
            10,
            &RosUdpHandler::low_cmd_callback,
            this
        );
}

void RosUdpHandler::timer_init()
{
    if (this->ctrl_level == UNITREE_LEGGED_SDK::LOWLEVEL)
    {
        this->cmd_lost_check_timer = this->ros_handle.createTimer(
            ros::Duration(this->cmd_lost_timelimit),
            &RosUdpHandler::cmd_lost_check_callback,
            this
        );
        this->safety_guard_timer = this->ros_handle.createTimer(
            ros::Duration(this->safety_guard_duration),
            &RosUdpHandler::safety_guard_callback,
            this
        );
    }
}

RosUdpHandler::RosUdpHandler(
    std::string robot_namespace,
    const float udp_duration,
    uint8_t level,
    ros::NodeHandle nh
):
    safe(UNITREE_LEGGED_SDK::LeggedType::Go1),
    // udp(8091, "192.168.123.161", 8082, sizeof(UNITREE_LEGGED_SDK::HighCmd), sizeof(UNITREE_LEGGED_SDK::HighState)),
    udp(level, 8080, "192.168.123.10", 8007),
    robot_namespace(robot_namespace),
    udp_duration(udp_duration),
    ctrl_level(level),
    ros_handle(nh),
    loop_udp_send("udp_send", udp_duration, 3, boost::bind(&RosUdpHandler::udp_send, this)),
    loop_udp_recv("udp_recv", udp_duration, 3, boost::bind(&RosUdpHandler::udp_recv, this)),
    loop_udp_translate("udp_translate", udp_duration, boost::bind(&RosUdpHandler::udp_translate, this))
{
    // if (level == UNITREE_LEGGED_SDK::LOWLEVEL)
    // {
    //     ROS_INFO_ONCE("Setting up UDP with LOWLEVEL control");
    //     this->udp = UNITREE_LEGGED_SDK::UDP(level);
    // }
    this->get_params();
    this->udp_init(level);
    ROS_INFO("Udp initialized");
    // set ros parameters and timers
    this->set_params();
    this->publisher_init();
    this->subscriber_init();
    this->timer_init();
    this->udp_start();
    ROS_INFO("RosUdpHandler constructed and started");
}

RosUdpHandler::~RosUdpHandler()
{
    for (int i (0); i < 12; i++) this->low_cmd_buffer.motorCmd[i].mode = 0;
    this->udp.SetSend(this->low_cmd_buffer);
    this->udp.Send();
    std::cout << "RosUdpHandler destructed" << std::endl;
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
    uint8_t level = UNITREE_LEGGED_SDK::HIGHLEVEL;
    if (use_low_level) level = UNITREE_LEGGED_SDK::LOWLEVEL;

    // construct and initialize this ros node
    RosUdpHandler ros_udp_handler(
        robot_namespace,
        udp_duration,
        level,
        nh
    );
    ros::spin();
    return 0;
}
