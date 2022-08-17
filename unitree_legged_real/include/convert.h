
#ifndef _UNITREE_CONVERT_H_
#define _UNITREE_CONVERT_H_

#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include <unitree_legged_msgs/MotorCmd.h>
#include <unitree_legged_msgs/MotorState.h>
#include <unitree_legged_msgs/IMU.h>

#include "unitree_legged_sdk/unitree_legged_sdk.h"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

unitree_legged_msgs::Cartesian state2rosMsg(UNITREE_LEGGED_SDK::Cartesian &state)
{
    unitree_legged_msgs::Cartesian ros_msg;

    ros_msg.x = state.x;
    ros_msg.y = state.y;
    ros_msg.z = state.z;

    return ros_msg;
}

unitree_legged_msgs::IMU state2rosMsg(UNITREE_LEGGED_SDK::IMU &state)
{
    unitree_legged_msgs::IMU ros_msg;

    // make sure the ros_msg quaternion is in (x, y, z, w) order
    ros_msg.quaternion[0] = state.quaternion[1];
    ros_msg.quaternion[1] = state.quaternion[2];
    ros_msg.quaternion[2] = state.quaternion[3];
    ros_msg.quaternion[3] = state.quaternion[0];

    for (int i(0); i < 3; i++)
    {
        ros_msg.gyroscope[i] = state.gyroscope[i];
        ros_msg.accelerometer[i] = state.accelerometer[i];
        ros_msg.rpy[i] = state.rpy[i];
    }

    ros_msg.temperature = state.temperature;

    return ros_msg;
}
unitree_legged_msgs::MotorState state2rosMsg(UNITREE_LEGGED_SDK::MotorState &state)
{
    unitree_legged_msgs::MotorState ros_msg;

    ros_msg.mode = state.mode;
    ros_msg.q = state.q;
    ros_msg.dq = state.dq;
    ros_msg.ddq = state.ddq;
    ros_msg.tauEst = state.tauEst;
    ros_msg.q_raw = state.q_raw;
    ros_msg.dq_raw = state.dq_raw;
    ros_msg.ddq_raw = state.ddq_raw;
    ros_msg.temperature = state.temperature;

    ros_msg.reserve[0] = state.reserve[0];
    ros_msg.reserve[1] = state.reserve[1];

    return ros_msg;
}

UNITREE_LEGGED_SDK::MotorCmd rosMsg2Cmd(const unitree_legged_msgs::MotorCmd &msg)
{
    UNITREE_LEGGED_SDK::MotorCmd cmd;

    cmd.mode = msg.mode;
    cmd.q = msg.q;
    cmd.dq = msg.dq;
    cmd.tau = msg.tau;
    cmd.Kp = msg.Kp;
    cmd.Kd = msg.Kd;

    for (int i(0); i < 3; i++) cmd.reserve[i] = msg.reserve[i];
    return cmd;
}

unitree_legged_msgs::MotorCmd Cmd2rosMsg(const UNITREE_LEGGED_SDK::MotorCmd* cmd)
{
    unitree_legged_msgs::MotorCmd ros_msg;

    ros_msg.mode = cmd->mode;
    ros_msg.q = cmd->q;
    ros_msg.dq = cmd->dq;
    ros_msg.tau = cmd->tau;
    ros_msg.Kp = cmd->Kp;
    ros_msg.Kd = cmd->Kd;

    for (int i(0); i < 3; i++) ros_msg.reserve[i] = cmd->reserve[i];

    return ros_msg;
}

unitree_legged_msgs::LowState state2rosMsg(UNITREE_LEGGED_SDK::LowState &state)
{
    unitree_legged_msgs::LowState ros_msg;

    ros_msg.levelFlag = state.levelFlag;
    ros_msg.commVersion = state.commVersion;
    
    ros_msg.robotID = state.robotID;
    ros_msg.SN = state.SN;
    ros_msg.bandWidth = state.bandWidth;
    ros_msg.imu = state2rosMsg(state.imu);
    for (int i(0); i < 20; i++) ros_msg.motorState[i] = state2rosMsg(state.motorState[i]);
    for (int i(0); i < 4; i++) ros_msg.footForce[i] = state.footForce[i];
    for (int i(0); i < 4; i++) ros_msg.footForceEst[i] = state.footForceEst[i];
    ros_msg.tick = state.tick;
    for (int i(0); i < 40; i++) ros_msg.wirelessRemote[i] = state.wirelessRemote[i]; 
    ros_msg.reserve = state.reserve;

    ros_msg.crc = state.crc;

    return ros_msg;
}

UNITREE_LEGGED_SDK::LowCmd rosMsg2Cmd(const unitree_legged_msgs::LowCmd::ConstPtr &msg)
{
    UNITREE_LEGGED_SDK::LowCmd cmd;

    cmd.levelFlag = msg->levelFlag;
    cmd.commVersion = msg->commVersion;
    
    cmd.robotID = msg->robotID;
    cmd.SN = msg->SN;
    cmd.bandWidth = msg->bandWidth;
    for (int i(0); i < 20; i++) cmd.motorCmd[i] = rosMsg2Cmd(msg->motorCmd[i]);
    for (int i(0); i < 4; i++)
    {
        cmd.led[i].r = msg->led[i].r;
        cmd.led[i].g = msg->led[i].g;
        cmd.led[i].b = msg->led[i].b;
    }
    for (int i(0); i < 40; i++) cmd.wirelessRemote[i] = msg->wirelessRemote[i];
    cmd.reserve = msg->reserve;

    cmd.crc = msg->crc;

    return cmd;
}

unitree_legged_msgs::LowCmd Cmd2rosMsg(const UNITREE_LEGGED_SDK::LowCmd *cmd)
{
    unitree_legged_msgs::LowCmd ros_msg;

    ros_msg.levelFlag = cmd->levelFlag;
    ros_msg.commVersion = cmd->commVersion;
    
    ros_msg.robotID = cmd->robotID;
    ros_msg.SN = cmd->SN;
    ros_msg.bandWidth = cmd->bandWidth;
    for (int i(0); i < 20; i++) ros_msg.motorCmd[i] = Cmd2rosMsg(&cmd->motorCmd[i]);
    for (int i(0); i < 4; i++)
    {
        ros_msg.led[i].r = cmd->led[i].r;
        ros_msg.led[i].g = cmd->led[i].g;
        ros_msg.led[i].b = cmd->led[i].b;
    }
    for (int i(0); i < 40; i++) ros_msg.wirelessRemote[i] = cmd->wirelessRemote[i];
    ros_msg.reserve = cmd->reserve;

    ros_msg.crc = cmd->crc;

    return ros_msg;
}

unitree_legged_msgs::HighState state2rosMsg(UNITREE_LEGGED_SDK::HighState &state)
{
    unitree_legged_msgs::HighState ros_msg;

    ros_msg.levelFlag = state.levelFlag;
    ros_msg.commVersion = state.commVersion;
    
    ros_msg.robotID = state.robotID;
    ros_msg.SN = state.SN;
    ros_msg.bandWidth = state.bandWidth;
    ros_msg.mode = state.mode;
    ros_msg.progress = state.progress;

    ros_msg.imu = state2rosMsg(state.imu);
    ros_msg.gaitType = state.gaitType;
    ros_msg.footRaiseHeight = state.footRaiseHeight;
    for (int i(0); i < 3; i++) ros_msg.position[i] = state.position[i];
    ros_msg.bodyHeight = state.bodyHeight;
    for (int i(0); i < 3; i++) ros_msg.velocity[i] = state.velocity[i];
    ros_msg.yawSpeed = state.yawSpeed;

    for (int i(0); i < 4; i++) ros_msg.footPosition2Body[i] = state2rosMsg(state.footPosition2Body[i]);
    for (int i(0); i < 4; i++) ros_msg.footSpeed2Body[i] = state2rosMsg(state.footSpeed2Body[i]);
    for (int i(0); i < 4; i++) ros_msg.footForce[i] = state.footForce[i];
    for (int i(0); i < 4; i++) ros_msg.footForceEst[i] = state.footForceEst[i];
    for (int i(0); i < 40; i++) ros_msg.wirelessRemote[i] = state.wirelessRemote[i];
    ros_msg.reserve = state.reserve;

    ros_msg.crc = state.crc;

    return ros_msg;
}

UNITREE_LEGGED_SDK::HighCmd rosMsg2Cmd(const unitree_legged_msgs::HighCmd::ConstPtr &msg)
{
    UNITREE_LEGGED_SDK::HighCmd cmd;

    cmd.levelFlag = msg->levelFlag;
    cmd.commVersion = msg->commVersion;
    cmd.robotID = msg->robotID;

    cmd.SN = msg->SN;
    cmd.bandWidth = msg->bandWidth;
    cmd.mode = msg->mode;

    cmd.gaitType = msg->gaitType;
    cmd.speedLevel = msg->speedLevel;
    cmd.footRaiseHeight = msg->footRaiseHeight;
    cmd.bodyHeight = msg->bodyHeight;
    for (int i(0); i < 2; i++) cmd.postion[i] = msg->position[i];
    for (int i(0); i < 3; i++) cmd.euler[i] = msg->euler[i];
    cmd.yawSpeed = msg->yawSpeed;
    for (int i(0); i < 4; i++)
    {
        cmd.led[i].r = msg->led[i].r;
        cmd.led[i].g = msg->led[i].g;
        cmd.led[i].b = msg->led[i].b;
    }
    for (int i(0); i < 40; i++) cmd.wirelessRemote[i] = msg->wirelessRemote[i];
    cmd.reserve = msg->reserve;

    cmd.crc = msg->crc;

    return cmd;
}

unitree_legged_msgs::HighCmd Cmd2rosMsg(const UNITREE_LEGGED_SDK::HighCmd* cmd)
{
    unitree_legged_msgs::HighCmd ros_msg;

    ros_msg.levelFlag = cmd->levelFlag;
    ros_msg.commVersion = cmd->commVersion;
    ros_msg.robotID = cmd->robotID;

    ros_msg.SN = cmd->SN;
    ros_msg.bandWidth = cmd->bandWidth;
    ros_msg.mode = cmd->mode;

    ros_msg.gaitType = cmd->gaitType;
    ros_msg.speedLevel = cmd->speedLevel;
    ros_msg.footRaiseHeight = cmd->footRaiseHeight;
    ros_msg.bodyHeight = cmd->bodyHeight;
    for (int i(0); i < 2; i++) ros_msg.position[i] = cmd->postion[i];
    for (int i(0); i < 3; i++) ros_msg.euler[i] = cmd->euler[i];
    ros_msg.yawSpeed = cmd->yawSpeed;
    for (int i(0); i < 4; i++)
    {
        ros_msg.led[i].r = cmd->led[i].r;
        ros_msg.led[i].g = cmd->led[i].g;
        ros_msg.led[i].b = cmd->led[i].b;
    }
    for (int i(0); i < 40; i++) ros_msg.wirelessRemote[i] = cmd->wirelessRemote[i];
    ros_msg.reserve = cmd->reserve;

    ros_msg.crc = cmd->crc;

    return ros_msg;
}

#endif