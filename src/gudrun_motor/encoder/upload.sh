#!/bin/bash
port=$( rosrun  gudrun_sensors get_usb_device_by_ID.py encoder_mega )
arduino --port "$port" --board arduino:avr:mega:cpu=atmega2560 --upload encoder.ino 
