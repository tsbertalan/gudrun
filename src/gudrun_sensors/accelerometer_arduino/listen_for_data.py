#!/usr/bin/env python

"""Thanks to Jean Rabault:
https://folk.uio.no/jeanra/Microelectronics/TransmitStructArduinoPython.html"""

from __future__ import print_function
from serial import Serial
import struct
import time, datetime
import numpy as np


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




def main():

    imu = IMU()

    def get_time_micros():
        return int((datetime.datetime.utcnow() - datetime.datetime(1970, 1, 1)).total_seconds() * 1000000)
        return(int(round(time.time() * 1000000)))
        
    micros = get_time_micros()

    for values in imu.stream():

        values = tuple(values)

        current_time = get_time_micros()
        time_elapsed = current_time - micros
        micros = current_time

        print("Time elapsed since last (us): " + str(time_elapsed))
        print("------")
        print("Accelerations: %.2f %.2f %.2f" % values[0:3])
        print("Angular rates: %.2f %.2f %.2f" % values[3:6])
        print("Orientation:   %.2f %.2f %.2f" % values[6:9])
        print (u"{}[2J{}[;H".format(chr(27), chr(27)), end="")  # clear screen https://stackoverflow.com/a/2084521/1224886


if __name__ == '__main__':
    main()
