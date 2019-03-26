#!/bin/bash
port=$( rosrun  gudrun_sensors get_usb_device_by_ID.py encoder_mega )
screen $port 115200
