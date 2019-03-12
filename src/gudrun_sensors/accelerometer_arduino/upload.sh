#!/bin/bash
DIR=$( dirname "$0" )
PORT=$( bash "$DIR/get_port.sh" )
echo "Uploading to $PORT ..."
arduino --port "$PORT" --board arduino:avr:mega:cpu=atmega2560 --upload *.ino 
