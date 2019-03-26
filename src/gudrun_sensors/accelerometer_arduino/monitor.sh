#!/bin/bash
DIR=`dirname "$0"`
echo "DIR=$DIR"
screen `bash $DIR/get_port.sh` 115200
