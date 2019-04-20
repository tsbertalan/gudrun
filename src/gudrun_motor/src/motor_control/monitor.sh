#!/bin/bash
port=$( rosrun  gudrun_sensors get_usb_device_by_ID.py motor_control )
screen $port 115200
