#!/usr/bin/env python

from distutils.core import setup#, find_packages
from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import find_packages

fp = find_packages()
print('Found packages:', fp)

setup_args = generate_distutils_setup(
     packages=['motor_control', 'motor_control.motor_control'] + fp,
     package_dir={'': 'src'}
)

setup(**setup_args)
