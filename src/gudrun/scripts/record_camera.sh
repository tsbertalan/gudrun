#!/bin/bash
set -e
folder=$HOME/data/gudrun/bagfiles/altdiff
echo "Saving bagfiles in folder: $folder"
mkdir -p "$folder"
cd "$folder"
rosbag record --output-prefix=fisheye --split --duration=3m --lz4 \
    /tf /speed_control/measured /steering_smoothed /cmd_vel \
    /controller_odom/twist_actual /ackermann_cmd \
    /d400_camera_throttled /d400_depth_throttled /d400/depth/camera_info
    # /t265/fisheye1/image_raw /t265/fisheye2/image_raw \
#/d400/depth/image_rect_raw 

