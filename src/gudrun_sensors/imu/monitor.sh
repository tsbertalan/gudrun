#!/bin/bash
SEARCH_STRING=$( cat device_search_string.txt )
# trim whitespace
# SEARCH_STRING=${SEARCH_STRING##*( )}
port=$( rosrun  gudrun_sensors get_usb_device_by_ID.py "$SEARCH_STRING" )
screen $port 
