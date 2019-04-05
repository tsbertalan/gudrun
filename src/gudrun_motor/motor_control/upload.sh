#!/bin/bash
port=$( rosrun  gudrun_sensors get_usb_device_by_ID.py motor_control )
# arduino --port "$port" --board arduino:avr:leonardo --upload encoder.ino 
rosrun gudrun_sensors upload_with_specified_vid_pid.py  --product=8037 --vendor=2341 --port=$port