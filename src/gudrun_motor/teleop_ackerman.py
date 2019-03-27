#!/usr/bin/env python
from __future__ import print_function
import sys, termios, tty
import rospy
from ackermann_msgs.msg import AckermannDriveStamped


class Axis(object):

    def __init__(self, callback, zero_point=0, low_point=-1, high_point=1, dummy=False):
        self.callback = callback
        self._ms_points = low_point, zero_point, high_point
        self._dummy = dummy
        self.fraction = 0

    @property
    def fraction(self):
        return self._fraction
    
    @fraction.setter
    def fraction(self, fraction):
        self._fraction = fraction
        l, m, h = self._ms_points
        if fraction >= 0:
            ms = fraction * (h - m) + m
        else:
            ms = fraction * (m - l) + m
        if not self._dummy:
            self.callback(ms)
        else:
            print('(Set axis %d to %s ms)' % (self._pin, ms))

class Teleop(object):

    def __init__(self):
        rospy.init_node('teleop_ackermann')
        self.pub = rospy.Publisher('ackermann_cmd', AckermannDriveStamped, queue_size=1)
        self.message = AckermannDriveStamped()
        self._steering_angle = self._speed = 0.

    def set_steering(self, angle):
        self.message.drive.steering_angle = angle
        self._publish()

    def set_speed(self, speed):
        self.message.drive.speed = speed
        self._publish()

    def _publish(self):
        self.pub.publish(self.message)


def keyboard_teleop():

    teleop = Teleop()

    steering_axis = Axis(teleop.set_steering)
    speed_axis = Axis(teleop.set_speed)

    def getch():
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
     
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    BUTTON_DELAY = 0.2
    THROTTLE_INCREMENT = .01
    STEERING_INCREMENT = .25

    def inc(steering=True, direction=+1):
        if steering:
            steering_axis.fraction += STEERING_INCREMENT * direction
            print('to', steering_axis.fraction)
        else:
            speed_axis.fraction += THROTTLE_INCREMENT * direction
            print('to', speed_axis.fraction)

    def center():
        steering_axis.fraction = 0

    def stop():
        speed_axis.fraction = 0

    keys = dict(
        # bizzarely, arrow keys give capital ABCD.
        A='UP',
        B='DOWN',
        C='RIGHT',
        D='LEFT',
    )
    keys['\t'] = 'TAB'
    keys[' '] = 'SPACE'
    def display_help():
        for key, (label, unused_action) in sorted(actions.items()):
            key = keys.get(key, key)
            print('"%s": %s' % (key, label))
        print('============================')

    actions = dict(
        q=('Quit.', None),
        w=('Accelerate', lambda : inc(0)),
        A=('Accelerate', lambda : inc(0)),
        s=('Decelerate', lambda : inc(0, -1)),
        B=('Decelerate', lambda : inc(0, -1)),
        d=('Steer right', lambda : inc(1, -1)),
        C=('Steer right', lambda : inc(1, -1)),
        a=('Steer left', lambda : inc(1, 1)),
        D=('Steer left', lambda : inc(1, 1)),
        c=('Center steering.', center),
        h=('Display key listing.', display_help),
        )    
    actions[' '] = ('Cut throttle.', stop)
    actions['\t'] = ('Cut throttle.', stop)

    display_help()

    while not rospy.is_shutdown():

        char = getch()

        if char in actions:
            label, action = actions[char]
            print(label, end='\n' if action in [None, display_help, center, stop] else ' ')
            if action is None:
                break
            action()
        # else:
        #     print('Got unknown char "%s".' % char)

if __name__ == '__main__':
    keyboard_teleop()
