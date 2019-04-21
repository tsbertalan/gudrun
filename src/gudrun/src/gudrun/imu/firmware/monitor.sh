#!/bin/bash
SEARCH_STRING=$( cat device_search_string.txt )
port=$( rosrun  gudrun_sensors get_usb_device_by_ID.py $SEARCH_STRING )
screen $port 115200
