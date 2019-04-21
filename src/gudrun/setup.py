#!/usr/bin/env python

from distutils.core import setup#, find_packages
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
     packages=[
        'motor_control', 'motor_control.motor_control',
        'gudrun',
        'gudrun.ros',
        'gudrun.encoder',
        'gudrun.imu',
        ],
     package_dir={'': 'src'}
)

setup(**setup_args)
