
from __future__ import print_function
import sys, termios, tty, serial, time
from os import system, getpid
from subprocess import check_output

from gudrun.usb_device import USBDevice


def rospy_log(kind, *a, **k):
    from rospy import logwarn, logerr, loginfo
    if kind == 'WARN':
        logwarn(*a, **k)
    elif kind == 'ERR':
        logwarn(*a, **k)
    elif kind == 'INFO':
        logwarn(*a, **k)
    else:
        raise ValueError('Argument "kind"=%s should be one of "WARN", "ERR", or "INFO".')


class Motor(USBDevice):
    # TODO: The level of abstraction for this thing is a little too low, given its new name.

    product = 8037
    vendor = 2341

    def __init__(self, verbose=False, **kw):

        self.HEADER = 0x7E
        self.verbose = verbose
        self.last_bytes = 90, 90
        super(Motor, self).__init__(**kw)

    @staticmethod
    def checksum(msg):
        t = 0
        for c in msg:
            assert isinstance(c, int)
            t += c
        return t & 0xff

    def send_packet(self, byte_a=None, byte_b=None):
        if byte_a is None:
            byte_a = self.last_bytes[0]
        if byte_b is None:
            byte_b = self.last_bytes[1]
        self.last_bytes = byte_a, byte_b

        c = self.checksum([self.HEADER, byte_a, byte_b])
        if self.verbose: print('Sending steering=%s, throttle=%s -> checksum=%s' % (byte_a, byte_b, c))
        try:
            self.ser.write(chr(self.HEADER))
            self.ser.write(chr(byte_a))
            self.ser.write(chr(byte_b))
            self.ser.write(chr(c))
        except ValueError:
            rospy_log('ERR', 'Failed to send packet: a=%s (%s) b=%s (%s) chk=%s (%s)' % (
                byte_a, type(byte_a), byte_b, type(byte_b), c, type(c)
            ))
        time.sleep(.01)

        response = []
        while self.ser.in_waiting > 0:
            try:
                data = self.ser.readline()
                response.append(data.strip())   
            except serial.serialutil.SerialException as e:
                rospy_log('ERR', str(e))
        if self.verbose:
            if len(response) > 0:
                print('Response (%d line%s):' % (len(response), '' if len(response) == 1 else 's'))
                print('\n'.join(['> %s' % r for r in response]))

        if 'csbad' in response:
            rospy_log('ERR', 'Motor control firmware reported bad checksum.')


def commandline():
    motor = Motor(verbose=1)
    
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

            motor.send_packet(a, b)
        except (AssertionError, ValueError):
            help()
        except EOFError:
            print('ctrl+d')
            break


DEBUG_DUMMY = False


class Smoother(object):

    def __init__(self, N=10):
        import numpy as np
        from collections import deque
        self.d = deque(maxlen=N)

    def __call__(self, x):
        self.d.append(x)
        m = 0.0
        for x in self.d:
            m += x
        return 0.0 if len(self.d) == 0 else m / len(self.d)

    def clear(self):
        self.d.clear()

    @property
    def maxlen(self):
        return self.d.maxlen
    

def _set_servo(pin, mc, angle):
    angle = int(angle)
    if pin == 0:
        mc.send_packet(byte_a=angle)
    else:
        mc.send_packet(byte_b=angle)


class Axis(object):

    def __init__(self, pin, motor_control_connection, zero_point=90, low_point=0, high_point=180, dummy=False):
        self._pin = pin
        self.motor = motor_control_connection
        self._ms_points = low_point, zero_point, high_point
        self._dummy = dummy
        self.fraction = 0

    @property
    def fraction(self):
        return self._fraction
    
    @fraction.setter
    def fraction(self, fraction):
        old_fraction = fraction
        if fraction < -1:
            fraction = -1
        if fraction > 1:
            fraction = 1
        if fraction != old_fraction and not hasattr(self, '_input_fraction_warned'):
           rospy_log('WARN', 'Capped input fraction from %s to %s.' % (old_fraction, fraction))
           self._input_fraction_warned = True
        self._fraction = fraction
        l, m, h = self._ms_points
        if fraction >= 0:
            angle = fraction * (h - m) + m
        else:
            angle = fraction * (m - l) + m
        if not self._dummy:
            _set_servo(self._pin, self.motor, angle)
        else:
            print('(Set servo %d to %s ms)' % (self._pin, ms))


class Car(object):

    def __init__(self, steering_pin=0, throttle_pin=1, dummy=DEBUG_DUMMY, MAX_THROTTLE_ABS=1):
        self.motor = Motor()

        self.MAX_THROTTLE_ABS = MAX_THROTTLE_ABS
        self._steering_axis = Axis(steering_pin, self.motor, dummy=dummy)
        self._throttle_axis = Axis(throttle_pin, self.motor, dummy=dummy)
        self._reversing = False
        self.stop()
        self.center()

        self.initialized = True

    def switch_to_reverse(self):
        if not self._reversing:
            self._throttle_axis.fraction = -.1
            time.sleep(.05)
            self._throttle_axis.fraction = 0
            time.sleep(.05)
            self._reversing = True

    @property
    def throttle(self):
        return self._throttle

    @throttle.setter
    def throttle(self, fraction):

        if not getattr(self, 'initialized', False):
            return

        fraction = min(max(fraction, -self.MAX_THROTTLE_ABS), self.MAX_THROTTLE_ABS)

        self._throttle = fraction
        if fraction < 0:
            self.switch_to_reverse()
        else:
            self._reversing = False
        self._throttle_axis.fraction = fraction

    def stop(self):
        self.throttle = 0

    def center(self):
        if getattr(self, 'initialized', False):
            self.steering = 0

    @property
    def steering(self):
        return self._steering_axis.fraction

    @steering.setter
    def steering(self, fraction):

        if not getattr(self, 'initialized', False):
            return

        self._steering_axis.fraction = fraction

    def __del__(self):
        print('Resetting steering and throttle on deletion of %s.' % self)
        self.stop()
        self.center()
    

if __name__ == '__main__':
    commandline()
