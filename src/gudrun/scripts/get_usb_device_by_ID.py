#!/usr/bin/env python
from __future__ import print_function
from gudrun.usb_device.get_devices import all_device_paths, device_path

from sys import argv

if len(argv) < 2:
    full_devices_map = all_device_paths()
    from pprint import pprint
    pprint(full_devices_map)

else:
    print(device_path(argv[1]))
