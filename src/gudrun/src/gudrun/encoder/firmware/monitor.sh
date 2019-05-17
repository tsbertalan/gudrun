#!/bin/bash
ss=$( cat ../device_search_string.txt )
port=$( rosrun  gudrun get_usb_device_by_ID $ss )
screen $port 115200
