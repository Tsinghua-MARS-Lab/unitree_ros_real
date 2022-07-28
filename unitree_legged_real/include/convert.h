
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

/** ROS message to UDP cmd **/

template<class Cmd_T, class Msg_T>
void rosMsg2Cmd(Cmd_T *dst, Msg_T *src)
{
    memcpy(dst, src, sizeof(Msg_T));
}

// UNITREE_LEGGED_SDK::HighCmd rosMsg2Cmd(const unitree_legged_msgs::HighCmd::ConstPtr &msg)
// {
//     UNITREE_LEGGED_SDK::HighCmd cmd;

//     cmd.levelFlag = msg->levelFlag;
//     cmd.commVersion = msg->commVersion;
//     cmd.robotID = msg->robotID;
//     cmd.SN = msg->SN;
//     cmd.bandWidth = msg->bandWidth;
//     cmd.mode = msg->mode;
//     cmd.gaitType = msg->gaitType;
//     cmd.speedLevel = msg->speedLevel;
//     cmd.footRaiseHeight = msg->footRaiseHeight;
//     cmd.bodyHeight = msg->bodyHeight;
//     for (int i = 0; i < 2; i++) cmd.postion[i] = msg->postion[i];
//     for (int i = 0; i < 3; i++) cmd.euler[i] = msg->euler[i];
//     cmd.yawSpeed = msg->yawSpeed;
//     for (int i = 0; i < 4; i++) cmd.led[i] = msg->led[i];
//     memcpy(cmd.wirelessRemote, msg->wirelessRemote, sizeof(uint8_t) * 40);
//     cmd.reserve = msg->reserve;
//     cmc.crc = msg->crc;

//     return cmd;
// }

/** UDP states to ROS message **/

template<class Msg_T, class State_T>
void state2rosMsg(Msg_T *dst, State_T *src)
{
    memcpy(dst, src, sizeof(State_T));
}

#endif