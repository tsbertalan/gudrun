#!/bin/bash
set -e
prefix_extra=$1
folder="$HOME/data/gudrun/bagfiles/$prefix_extra/"
echo "Saving bagfiles in folder: $folder"
mkdir -p "$folder"
cd "$folder"


rosbag record --output-prefix=camera_"$prefix_extra" --split --duration=3m --lz4 \
    /tf /speed_control/measured /steering_smoothed /cmd_vel \
    /controller_odom/twist_actual /ackermann_cmd \
    /d400/color/image_raw/throttled /d400/depth/image_rect_raw/throttled /segmentation/rgbd/throttled\
    /d400/depth/camera_info
    # /t265/fisheye1/image_raw /t265/fisheye2/image_raw \
#/d400/depth/image_rect_raw 

