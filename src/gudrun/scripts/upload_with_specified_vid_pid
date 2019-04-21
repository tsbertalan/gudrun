#!/usr/bin/env python
from __future__ import print_function
import argparse

from gudrun.usb_device.usb_firmware import upload

parser = argparse.ArgumentParser(description='Upload ino file with specified product and vendor ID.')
parser.add_argument('--product', default='8036')
parser.add_argument('--vendor', default='2341')
parser.add_argument('--port', default=None)
args = parser.parse_args()

if args.port is None:
    args.port = cmd('rosrun', 'gudrun', 'get_usb_device_by_ID.py', 'Arduino_LLC_Arduino_Leonardo_8036:2341').strip()
    print('No port was specified, so getting default as', args.port)
upload(pid=args.product, vid=args.vendor, port=args.port)
