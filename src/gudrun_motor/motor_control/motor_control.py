from __future__ import print_function
import serial, time
from subprocess import check_output

class MotorControl(object):

    def __init__(self, PORT=None, BAUDRATE=115200, verbose=False):

        if PORT is None: PORT = check_output(['rosrun', 'gudrun_sensors', 'get_usb_device_by_ID.py', 'motor_control']).strip()
        self.ser = serial.Serial(PORT, BAUDRATE)
        self.HEADER = 0x7E
        self.verbose = verbose

    @staticmethod
    def checksum(msg):
        t = 0
        for c in msg:
            assert isinstance(c, int)
            t += c
        return t & 0xff

    def send_packet(self, byte_a, byte_b):
        c = self.checksum([self.HEADER, byte_a, byte_b])
        if self.verbose: print('Sending steering=%s, throttle=%s -> checksum=%s' % (byte_a, byte_b, c))
        self.ser.write(chr(self.HEADER))
        self.ser.write(chr(byte_a))
        self.ser.write(chr(byte_b))
        self.ser.write(chr(c))
        time.sleep(.01)

        response = []
        while self.ser.in_waiting > 0:
            data = self.ser.readline()
            response.append(data.strip())
        if self.verbose:
            if len(response) > 0:
                print('Response (%d line%s):' % (len(response), '' if len(response) == 1 else 's'))
                print('\n'.join(['> %s' % r for r in response]))
            else:
                print('Response is empty.')

        if 'csbad' in response:
            from warnings import warn, simplefilter, catch_warnings
            with catch_warnings():
                simplefilter('always', UserWarning)
                warn('Motor control firmware reported bad checksum.')

def commandline():
    mc = MotorControl(verbose=1)
    
    help = lambda : print('Type two numbers both in [0, 180], separated by space. 90 is neutral. Enter ctrl+d (EOF) to exit.')
    help()

    while True:

        try:
            print('>>> ', end='')
            inp = raw_input().split()
            if len(inp) != 2:
                print('Need two values.')
                raise ValueError

            a, b = inp

            a = int(a)
            b = int(b)

            for x in (a, b):
                if not (x >= 0 and x <= 180):
                    print('Out-of-range.')
                    raise ValueError

            mc.send_packet(a, b)
        except (AssertionError, ValueError):
            help()
        except EOFError:
            print('ctrl+d')
            break

if __name__ == '__main__':
    commandline()
