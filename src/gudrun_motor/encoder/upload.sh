#!/bin/bash
port=$( rosrun  gudrun_sensors get_usb_device_by_ID.py encoder_micro )
# arduino --port "$port" --board arduino:avr:leonardo --upload encoder.ino 
rosrun gudrun_sensors upload_with_specified_vid_pid.py --product=8036 --vendor=2341 --port=$port