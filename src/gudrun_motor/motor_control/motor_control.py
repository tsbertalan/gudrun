from __future__ import print_function
import serial, time
from subprocess import check_output

class MotorControl(object):

    def __init__(self, PORT=None, BAUDRATE=115200):

        if PORT is None: PORT = check_output(['rosrun', 'gudrun_sensors', 'get_usb_device_by_ID.py', 'motor_control']).strip()

        self.ser = serial.Serial(PORT, BAUDRATE)

        self.HEADER = 0x7E

    @staticmethod
    def checksum(msg):
        t = 0
        for c in msg:
            # if isinstance(msg, str):
            #     t += ord(c)
            # else:
            t += c
        return t & 0xff

    def send_packet(self, byte_a, byte_b):
        c = self.checksum([self.HEADER, byte_a, byte_b])
        print('Sending a=%s, b=%s; checksum=%s' % (byte_a, byte_b, c))
        self.ser.write(chr(self.HEADER))
        self.ser.write(chr(byte_a))
        self.ser.write(chr(byte_b))
        self.ser.write(chr(c))
        time.sleep(.01)

        print('Response:')
        response = []
        while self.ser.in_waiting > 0:
            data = self.ser.readline()
            response.append(data)
        print(''.join(['    %s' % r for r in response]))
        print('%d lines' % (len(response),))


mc = MotorControl()
mc.send_packet(0, 90)

