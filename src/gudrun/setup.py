#!/usr/bin/env python

from distutils.core import setup#, find_packages
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
     packages=[
        'gudrun',
        'gudrun.ros',
        'gudrun.encoder',
        'gudrun.imu',
        'gudrun.motor',
        'gudrun.ultrasound',
        ],
     package_dir={'': 'src'}
)

setup(**setup_args)
