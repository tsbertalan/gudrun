#!/usr/bin/env python

"""Thanks to Jean Rabault:
https://folk.uio.no/jeanra/Microelectronics/TransmitStructArduinoPython.html"""

from __future__ import print_function
import glob
import struct
import time, datetime
import numpy as np


def look_for_available_ports():
    """
    find available serial ports to Arduino
    """
    available_ports = glob.glob('/dev/ttyACM*')
    print("Available porst: ")
    print(available_ports)

    return available_ports


def get_time_micros():
    return int((datetime.datetime.utcnow() - datetime.datetime(1970, 1, 1)).total_seconds() * 1000000)
    return(int(round(time.time() * 1000000)))


def get_time_millis():
    return(int(round(time.time() * 1000)))


# This looks wrong to me:
# def get_time_seconds():
#     return(int(round(time.time() * 1000000)))


def print_values(values):
    print("------")
    print("Accelerations: ")
    print(values[0:3])
    print("Angular rates: ")
    print(values[3:6])
    print("Orientation: ")
    print(values[6:9])


class ReadFromArduino(object):
    """A class to read the serial messages from Arduino. The code running on Arduino
    can for example be the ArduinoSide_LSM9DS0 sketch."""

    def __init__(self, port, SIZE_STRUCT=9*4, verbose=0):
        self.port = port
        # self.millis = get_time_millis()
        self.micros = get_time_micros()
        self.SIZE_STRUCT = SIZE_STRUCT
        self.verbose = verbose
        self.latest_values = -1
        # self.t_init = get_time_millis()
        # self.t = 0

        self.port.flushInput()

    def read_one_value(self):
        """Wait for next serial message from the Arduino, and read the whole
        message as a structure."""
        read = False

        while not read:
            myByte = self.port.read(1)
            if myByte == 'S':
                data = self.port.read(self.SIZE_STRUCT)
                myByte = self.port.read(1)
                if myByte == 'E':
                    # self.t = (get_time_millis() - self.t_init) / 1000.0

                    # is  a valid message struct
                    new_values = struct.unpack('<fffffffff', data)

                    # current_time = get_time_millis()
                    # time_elapsed = current_time - self.millis
                    # self.millis = current_time
                    current_time = get_time_micros()
                    time_elapsed = current_time - self.micros
                    self.micros = current_time

                    read = True

                    self.latest_values = np.array(new_values)

                    # convert measurements to rad/s
                    self.latest_values[3: 6] = 3.1415 / 180.0 * self.latest_values[3: 6]

                    if self.verbose > 1:
                        # print("Time elapsed since last (ms): " + str(time_elapsed))
                        print('time [us]:', get_time_micros())
                        print("Time elapsed since last (us): " + str(time_elapsed))
                        print_values(new_values)

                    return(True)

        return(False)

if __name__ == '__main__':
    from serial import Serial
    available_ports = look_for_available_ports()
    if len(available_ports) > 0:
        port = Serial(available_ports[0], 115200)

        listener = ReadFromArduino(port, verbose=2)

        while True:
            listener.read_one_value()

    else:
        print('No serial ports found.')