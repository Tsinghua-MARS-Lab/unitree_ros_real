# Unitree ROS real (package)

## Description
The ROS package that runs on Unitree A1 NX and expose all functionality of Unitree A1 to ROS network.

## Installation
1. Make sure you can access your ROS system. or `source /opt/ros/kinetic/setup.bash`

2. Make sure your system has `lcm` installed and optionally `realsense-ros` installed.

3. Setup your ROS workspace on your robot (A1) where you want to run the neural network.

    ``` bash
    mkdir -p unitree_ws/src
    cd unitree_ws
    catkin_make
    ```

4. Download [unitree_legged_sdk v3.8.6](https://github.com/unitreerobotics/unitree_legged_sdk/releases/tag/v3.8.6) and extract it under `unitree_ws/src/`

5. Clone this branch of the repository to `unitree_ws/src/`

6. Run `catkin_make` under the workspace folder

    ``` bash
    cd ../
    catkin_make
    ```

    If error occurrs related to message file, you can run `catkin_make` again or temporarily stop the package `unitree_legged_real` from compiling, then compile `unitree_legged_real` after the message files are generated.

## Usage

***Important NOTE: Keep the emergency stop button in your hand or connect your robot using a stoppable power source if you run this package for the first time.***

### Launch the ROS robot node

1. Make sure your ROS system is accessable and `unitree_ws` is sourced (e.g. `source unitree_ws/devel/setup.bash`)

2. Make a detail check on the options in `unitree_legged_real/launch/robot.launch`.

3. Make sure your robot has all motor powered off, or pressing `R2 + B` on your remote control for A1.

4. Make sure all joints on the robot are not at the mechanical limit. (Or the safety guard will stop the node)

5. Launch the ROS node by

    - If you want only see the robot states but not reacting any motor command.
    ``` bash
    roslaunch unitree_legged_real robot.launch
    ```

    - If you want to start the robot node and responding to motor command
    ``` bash
    roslaunch unitree_legged_real robot.launch dryrun:=false
    ```

### Control the robot using ROS network

1. Make sure you have another terminal with ROS access and `unitree_ws` workspace access. Make sure the ROS robot node is running.

2. Checkout robot state topic and command topic. Just publish and subscribe through these two topics.

    If you did not change anything in `robot.launch`, you will see `/a112138/low_state` as robot proprioception state and `/a112138/legs_cmd` as where the robot recieve the motor commands (defined in the same order as the SDK)

### Check Robot state in rviz

1. Launch the robot node by

    ``` bash
    roslaunch unitree_legged_real robot.launch publish_joint_state:=true publish_imu:=true
    ```

2. Launch the odom node by

    ``` bash
    roslaunch unitree_legged_real odom.launch
    ```

3. Visualize in another terminal in the same ROS network that have access to a compiled `unitree_ws` workspace and GUI running. (a.k.a a laptop connected to A1's ROS network)

    ``` bash
    roslaunch a1_description a1_rviz.launch
    ```

## Interfaces and functions
- Only one process to handle robot proprioceptive command and states.
- A configurable rosnode namespace, just like xiaomi Cyberdog (/mi3065353)
- publish HighState / LowState on a given frequency
- publish HighCmd / LowCmd publish every time upd.send()
- Mandatory safe protection just before upd.send()

### In HIGHLEVEL
- Currently removed.

### In LOWEVEL
- subscribe customized motor message for each joint

### Perception system
- publish Twist message filtered IMU rpy euler angle, linear speed (kalman filter, etc)
- publish Transform message for each joint position of the robot
- publish Transform message for Robot base odometry