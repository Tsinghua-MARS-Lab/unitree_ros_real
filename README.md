# Unitree ROS real (package)

## Description
The ROS package that runs on Unitree A1 NX and expose all functionality of Unitree A1 to ROS network.

## Interfaces and functions
- Only one process to handle robot proprioceptive command and states.
- A configurable rosnode namespace, just like xiaomi Cyberdog (/mi3065353)
- publish HighState / LowState on a given frequency
- publish HighCmd / LowCmd publish every time upd.send()
- Mandatory safe protection just before upd.send()

### In HIGHLEVEL
- service for change mode (use ROS constant in service file)
- service for change gaitType (use ROS constant in service file)
- subscribe Twist message for x, y, yaw moving
- subscribe Twist message for position, euler angle positioning
- subscribe Float32 message for set body height

### In LOWEVEL
- subscribe customized motor message for each joint

### Perception system
- publish Twist message filtered IMU rpy euler angle, linear speed (kalman filter, etc)
- publish Transform message for each joint position of the robot
- publish Transform message for Robot base odometry
- publish aligned realsense RGB-D camera image and camera Transform (intrinsic parameters)

    (Usually it is two topics: image_raw and camera_info for RGB and for Depth)