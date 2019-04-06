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
            assert isinstance(c, int)
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

if False:
	help = lambda : print('Type two numbers separated by a space, each in [0, 180].')
	help()

	while True:
	    print('>>> ', end='')
	    inp = raw_input()
	    try:
	        a, b = inp.split()
	        mc.send_packet(int(a), int(b))
	    except ValueError:
	        help()

else:
	help = lambda : print('Type one number in [0, 180].')
	help()

	while True:
	    print('>>> ', end='')
	    inp = raw_input()
	    try:
	        mc.send_packet(90, int(inp))
	    except ValueError:
	        help()
