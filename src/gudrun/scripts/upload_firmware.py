#!/usr/bin/env python
from sys import argv

if argv[1] == 'encoder':
    from gudrun.encoder import Encoder as Device
    
if argv[1] == 'imu':
    from gudrun.imu import IMU as Device

device = Device(allow_default_ss=True, connect=False)
device.flash()