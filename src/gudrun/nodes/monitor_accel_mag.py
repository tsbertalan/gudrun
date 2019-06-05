#!/usr/bin/env python
from __future__ import print_function
import rospy
from sensor_msgs.msg import Imu
from collections import deque

from numpy import sqrt, mean

rospy.init_node('monitor_accel_mag')

history = deque(maxlen=100)

def print_mag(imu_msg):
    a = imu_msg.linear_acceleration
    magnitude = sqrt(a.x**2 + a.y**2 + a.z**2)

    history.append(magnitude)

    print('Magnitude:', magnitude)
    print('Running mean of', len(history), 'is', mean(history), '.')

rospy.Subscriber('/imu/data', Imu, print_mag, queue_size=1)

rospy.spin()
