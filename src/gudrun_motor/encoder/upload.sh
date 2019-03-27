#!/bin/bash
port=$( rosrun  gudrun_sensors get_usb_device_by_ID.py encoder_micro )
arduino --port "$port" --board arduino:avr:leonardo --upload encoder.ino 
