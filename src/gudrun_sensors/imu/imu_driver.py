#!/usr/bin/env python

"""Thanks to Jean Rabault:
https://folk.uio.no/jeanra/Microelectronics/TransmitStructArduinoPython.html"""

from __future__ import print_function
from serial import Serial
import struct


def get_port_address(verbose=False):
    import os.path
    from subprocess import check_output
    
    ss_config_path = os.path.join(os.path.dirname(__file__), 'device_search_string.txt')

    if os.path.isfile(ss_config_path):
        with open(ss_config_path, 'r') as f:
            search_string = f.read().strip()
    else:
        search_string = 'Arduino_LLC_Arduino_Leonardo_8037:2341'
        from warnings import warn
        warn('No %s; using default search string of %s.' % (ss_config_path, search_string))

    if verbose: print('Searching for device "%s" ...' % search_string)
    
    cmd = ['rosrun', 'gudrun_sensors', 'get_usb_device_by_ID.py', search_string]
    if verbose: print('$ ' + ' '.join(cmd))
    addr = check_output(cmd).strip()

    if addr == 'device_not_found':
        raise IOError("Device '%s' wasn't found." % search_string)
    
    return addr


class IMU(object):
    """A class to stream the serial messages from Arduino."""

    def __init__(self, SIZE_STRUCT=9*4, verbose=0):
        self.port = Serial(get_port_address(), 115200)
        
        self.SIZE_STRUCT = SIZE_STRUCT
        self.verbose = verbose
        
        self.port.flushInput()

    def read_one_value(self):
        """Wait for next serial message from the Arduino,
        and read the whole message as a structure."""
        read = False

        while not read:
            myByte = self.port.read(1)
            if myByte == 'S':
                data = self.port.read(self.SIZE_STRUCT)
                myByte = self.port.read(1)
                if myByte == 'E':

                    # is  a valid message struct
                    new_values = list(struct.unpack('<fffffffff', data))

                    read = True

                    # convert measurements to rad/s
                    for i in range(3, 6):
                        new_values[i] = 3.1415 / 180.0 * new_values[i]

                    return new_values

        return None

    def stream(self):
        while True:
            yield self.read_one_value()


def ros_publish(rate=None):
    import rospy
    from sensor_msgs.msg import Imu as Imu_Message

    FRAME_NAME = 'imu'

    rospy.init_node('imu')

    publisher = rospy.Publisher('imu', Imu_Message, queue_size=10)

    msg = Imu_Message()

    imu = IMU()


    if rate is not None:
        dt = 1. / rate
        t_last = rospy.get_time() - dt - 1

    while not rospy.is_shutdown():

        data = imu.read_one_value()

        if rate is not None: t = rospy.get_time()

        if rate is None or t >= t_last + dt:
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = data[0:3]
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = data[3:6]
            msg.orientation.x, msg.orientation.y, msg.orientation.z = data[6:9]

            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = FRAME_NAME

            publisher.publish(msg)

            t_last = rospy.get_time()


if __name__ == '__main__':
    ros_publish()
