
#ifndef _UNITREE_CONVERT_H_
#define _UNITREE_CONVERT_H_

#include <unitree_legged_msgs/Cartesian.h>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
// #include <unitree_legged_msgs/HighCmd.h>
// #include <unitree_legged_msgs/HighState.h>
#include <unitree_legged_msgs/MotorCmd.h>
#include <unitree_legged_msgs/MotorState.h>
#include <unitree_legged_msgs/IMU.h>

#include "unitree_legged_sdk/unitree_legged_sdk.h"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

UNITREE_LEGGED_SDK::BmsCmd rosMsg2Cmd(const unitree_legged_msgs::BmsCmd &msg)
{
    UNITREE_LEGGED_SDK::BmsCmd cmd;
    
    // cmd.off = msg.off; // TODO: check if this is correct
    for (int i = 0; i < 3; i++) cmd.reserve[i] = msg.reserve[i];

    return cmd;
}

unitree_legged_msgs::BmsCmd Cmd2rosMsg(const UNITREE_LEGGED_SDK::BmsCmd *cmd)
{
    unitree_legged_msgs::BmsCmd ros_msg;

    ros_msg.off = cmd->off;
    for (int i = 0; i < 3; i++) ros_msg.reserve[i] = cmd->reserve[i];

    return ros_msg;
}

unitree_legged_msgs::BmsState state2rosMsg(UNITREE_LEGGED_SDK::BmsState &state)
{
    unitree_legged_msgs::BmsState ros_msg;

    ros_msg.version_h = state.version_h;
    ros_msg.version_l = state.version_l;
    ros_msg.bms_status = state.bms_status;
    ros_msg.SOC = state.SOC;
    ros_msg.current = state.current;
    ros_msg.cycle = state.cycle;
    ros_msg.BQ_NTC[0] = state.BQ_NTC[0];
    ros_msg.BQ_NTC[1] = state.BQ_NTC[1];
    ros_msg.MCU_NTC[0] = state.MCU_NTC[0];
    ros_msg.MCU_NTC[1] = state.MCU_NTC[1];
    for (int i = 0; i < 10; i++) ros_msg.cell_vol[i] = state.cell_vol[i];

    return ros_msg;
}

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

unitree_legged_msgs::WirelessRemote Bytes2rosMsg(const std::array<uint8_t, 40> &bytes)
{
    unitree_legged_msgs::WirelessRemote ros_msg;

    // 0, 1 Byte
    ros_msg.head[0] = bytes[0];
    ros_msg.head[1] = bytes[1];

    // 2, 3 Byte, 16 bits
    ros_msg.btn.components.R1 =     ((bytes[2] >> 0) & 0x01);
    ros_msg.btn.components.L1 =     ((bytes[2] >> 1) & 0x01);
    ros_msg.btn.components.start =  ((bytes[2] >> 2) & 0x01);
    ros_msg.btn.components.select = ((bytes[2] >> 3) & 0x01);
    ros_msg.btn.components.R2 =     ((bytes[2] >> 4) & 0x01);
    ros_msg.btn.components.L2 =     ((bytes[2] >> 5) & 0x01);
    ros_msg.btn.components.F1 =     ((bytes[2] >> 6) & 0x01);
    ros_msg.btn.components.F2 =     ((bytes[2] >> 7) & 0x01);
    ros_msg.btn.components.A =      ((bytes[3] >> 0) & 0x01);
    ros_msg.btn.components.B =      ((bytes[3] >> 1) & 0x01);
    ros_msg.btn.components.X =      ((bytes[3] >> 2) & 0x01);
    ros_msg.btn.components.Y =      ((bytes[3] >> 3) & 0x01);
    ros_msg.btn.components.up =     ((bytes[3] >> 4) & 0x01);
    ros_msg.btn.components.right =  ((bytes[3] >> 5) & 0x01);
    ros_msg.btn.components.down =   ((bytes[3] >> 6) & 0x01);
    ros_msg.btn.components.left =   ((bytes[3] >> 7) & 0x01);
    ros_msg.btn.value = (bytes[3] << 8) | bytes[2];

    // 4 ~ 24 Bytes
    // little endian
    union {
        float output;   // assumes sizeof(float) == sizeof(int)
        uint32_t input;
    } data;
    data.input = (
        (uint32_t)bytes[4] << 0 |
        (uint32_t)bytes[5] << 8 |
        (uint32_t)bytes[6] << 16 |
        (uint32_t)bytes[7] << 24
    );
    ros_msg.lx = data.output;
    data.input = (
        (uint32_t)bytes[8] << 0 |
        (uint32_t)bytes[9] << 8 |
        (uint32_t)bytes[10] << 16 |
        (uint32_t)bytes[11] << 24
    );
    ros_msg.rx = data.output;
    data.input = (
        (uint32_t)bytes[12] << 0 |
        (uint32_t)bytes[13] << 8 |
        (uint32_t)bytes[14] << 16 |
        (uint32_t)bytes[15] << 24
    );
    ros_msg.ry = data.output;
    data.input = (
        (uint32_t)bytes[16] << 0 |
        (uint32_t)bytes[17] << 8 |
        (uint32_t)bytes[18] << 16 |
        (uint32_t)bytes[19] << 24
    );
    ros_msg.L2 = data.output;
    data.input = (
        (uint32_t)bytes[20] << 0 |
        (uint32_t)bytes[21] << 8 |
        (uint32_t)bytes[22] << 16 |
        (uint32_t)bytes[23] << 24
    );
    ros_msg.ly = data.output;

    return ros_msg;
}

void rosMsg2Bytes(std::array<uint8_t, 40> &dst, const unitree_legged_msgs::WirelessRemote &wireless_remote)
{
    // 0, 1 Byte
    dst[0] = wireless_remote.head[0];
    dst[1] = wireless_remote.head[2];

    // 2, 3 Byte, 16 bits
    dst[2] |= ((uint8_t)wireless_remote.btn.components.R1 << 0);
    dst[2] |= ((uint8_t)wireless_remote.btn.components.L1 << 1);
    dst[2] |= ((uint8_t)wireless_remote.btn.components.start << 2);
    dst[2] |= ((uint8_t)wireless_remote.btn.components.select << 3);
    dst[2] |= ((uint8_t)wireless_remote.btn.components.R2 << 4);
    dst[2] |= ((uint8_t)wireless_remote.btn.components.L2 << 5);
    dst[2] |= ((uint8_t)wireless_remote.btn.components.F1 << 6);
    dst[2] |= ((uint8_t)wireless_remote.btn.components.F2 << 7);
    dst[3] |= ((uint8_t)wireless_remote.btn.components.A << 0);
    dst[3] |= ((uint8_t)wireless_remote.btn.components.B << 1);
    dst[3] |= ((uint8_t)wireless_remote.btn.components.X << 2);
    dst[3] |= ((uint8_t)wireless_remote.btn.components.Y << 3);
    dst[3] |= ((uint8_t)wireless_remote.btn.components.up << 4);
    dst[3] |= ((uint8_t)wireless_remote.btn.components.right << 5);
    dst[3] |= ((uint8_t)wireless_remote.btn.components.down << 6);
    dst[3] |= ((uint8_t)wireless_remote.btn.components.left << 7);

    // 4 ~ 24 Bytes
    // little endian
    union {
        float input;   // assumes sizeof(float) == sizeof(int)
        uint32_t output;
    } data;
    data.input = wireless_remote.lx;
    dst[4] = ((uint8_t)data.output >> 0);
    dst[5] = ((uint8_t)data.output >> 8);
    dst[6] = ((uint8_t)data.output >> 16);
    dst[7] = ((uint8_t)data.output >> 24);
    data.input = wireless_remote.rx;
    dst[8] = ((uint8_t)data.output >> 0);
    dst[9] = ((uint8_t)data.output >> 8);
    dst[10] = ((uint8_t)data.output >> 16);
    dst[11] = ((uint8_t)data.output >> 24);
    data.input = wireless_remote.ry;
    dst[12] = ((uint8_t)data.output >> 0);
    dst[13] = ((uint8_t)data.output >> 8);
    dst[14] = ((uint8_t)data.output >> 16);
    dst[15] = ((uint8_t)data.output >> 24);
    data.input = wireless_remote.L2;
    dst[16] = ((uint8_t)data.output >> 0);
    dst[17] = ((uint8_t)data.output >> 8);
    dst[18] = ((uint8_t)data.output >> 16);
    dst[19] = ((uint8_t)data.output >> 24);
    data.input = wireless_remote.ly;
    dst[20] = ((uint8_t)data.output >> 0);
    dst[21] = ((uint8_t)data.output >> 8);
    dst[22] = ((uint8_t)data.output >> 16);
    dst[23] = ((uint8_t)data.output >> 24);
}

unitree_legged_msgs::LowState state2rosMsg(UNITREE_LEGGED_SDK::LowState &state)
{
    unitree_legged_msgs::LowState ros_msg;

    ros_msg.head[0] = state.head[0];
    ros_msg.head[1] = state.head[1];
    ros_msg.levelFlag = state.levelFlag;
    ros_msg.frameReserve = state.frameReserve;

    ros_msg.SN[0] = state.SN[0];
    ros_msg.SN[1] = state.SN[1];
    ros_msg.version[0] = state.version[0];
    ros_msg.version[1] = state.version[1];
    ros_msg.bandWidth = state.bandWidth;
    ros_msg.imu = state2rosMsg(state.imu);
    for (int i(0); i < 20; i++) ros_msg.motorState[i] = state2rosMsg(state.motorState[i]);
    ros_msg.bms = state2rosMsg(state.bms);    
    for (int i(0); i < 4; i++) ros_msg.footForce[i] = state.footForce[i];
    for (int i(0); i < 4; i++) ros_msg.footForceEst[i] = state.footForceEst[i];
    ros_msg.tick = state.tick;
    ros_msg.wirelessRemote = Bytes2rosMsg(state.wirelessRemote); 
    ros_msg.reserve = state.reserve;

    ros_msg.crc = state.crc;

    return ros_msg;
}

UNITREE_LEGGED_SDK::LowCmd rosMsg2Cmd(const unitree_legged_msgs::LowCmd::ConstPtr &msg)
{
    UNITREE_LEGGED_SDK::LowCmd cmd;

    cmd.head[0] = msg->head[0];
    cmd.head[1] = msg->head[1];
    cmd.levelFlag = msg->levelFlag;
    cmd.frameReserve = msg->frameReserve;

    cmd.SN[0] = msg->SN[0];
    cmd.SN[1] = msg->SN[1];
    cmd.version[0] = msg->version[0];
    cmd.version[1] = msg->version[1];    
    cmd.bandWidth = msg->bandWidth;
    for (int i(0); i < 20; i++) cmd.motorCmd[i] = rosMsg2Cmd(msg->motorCmd[i]);
    cmd.bms = rosMsg2Cmd(msg->bms);
    rosMsg2Bytes(cmd.wirelessRemote, msg->wirelessRemote);
    cmd.reserve = msg->reserve;

    cmd.crc = msg->crc;

    return cmd;
}

unitree_legged_msgs::LowCmd Cmd2rosMsg(const UNITREE_LEGGED_SDK::LowCmd *cmd)
{
    unitree_legged_msgs::LowCmd ros_msg;

    ros_msg.head[0] = cmd->head[0];
    ros_msg.head[1] = cmd->head[1];
    ros_msg.levelFlag = cmd->levelFlag;
    ros_msg.frameReserve = cmd->frameReserve;

    ros_msg.SN[0] = cmd->SN[0];
    ros_msg.SN[1] = cmd->SN[1];
    ros_msg.version[0] = cmd->version[0];
    ros_msg.version[1] = cmd->version[1];    
    ros_msg.bandWidth = cmd->bandWidth;
    for (int i(0); i < 20; i++) ros_msg.motorCmd[i] = Cmd2rosMsg(&cmd->motorCmd[i]);
    ros_msg.bms = Cmd2rosMsg(&cmd->bms);
    ros_msg.wirelessRemote = Bytes2rosMsg(cmd->wirelessRemote);
    ros_msg.reserve = cmd->reserve;

    ros_msg.crc = cmd->crc;

    return ros_msg;
}

/* Disabling all HighLevel thingy...

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
    ros_msg.wirelessRemote = Bytes2rosMsg(state.wirelessRemote);
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
    rosMsg2Bytes(cmd.wirelessRemote, msg->wirelessRemote);
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
    ros_msg.wirelessRemote = Bytes2rosMsg(cmd->wirelessRemote);
    ros_msg.reserve = cmd->reserve;

    ros_msg.crc = cmd->crc;

    return ros_msg;
}

*/

#endif