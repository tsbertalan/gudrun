#!/bin/bash
set -e
cd ~/gudrun/data/
rosbag record --bz2 --split --duration=5m  \
    \
    /camera/rgb/image_color /camera/depth_registered/image\
    \
    /imu/data_raw /imu/data \
    /vo /odometry/filtered \
    \
    /tf /tf_static \
    \
    /scan \
    \
    /motor_encoder/count \
    /motor_encoder/velocity \
    /ackermann_cmd \
    /speed_control/setpoint \
    /speed_control/proportional \
    /speed_control/integral \
    /speed_control/derivative \
    /speed_control/measured \
    /speed_control/control \
    /speed_control/throttle \
    /key_vel /nav_vel /twist_mux/cmd_vel \
    \
    /move_base/TebLocalPlannerROS/global_plan\
    /move_base/TebLocalPlannerROS/local_plan \
