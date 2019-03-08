#!/usr/bin/env python

import numpy as np, rospy, time
from sensor_msgs.msg import Range
from teleop import Car, Smoother

class SpeedRecorder(object):

    def __init__(self):
        rospy.init_node('speed_record')
       
        filename = rospy.get_param('speed_record/filename', None)
        self.f = open(filename, 'a') if filename else None
        if not self.f:
            print("Given filename was %s; not saving distance data." % filename)

        self.v = rospy.get_param('speed_record/throttle')
        self.w = rospy.get_param('speed_record/steering')
        self.timeout = rospy.get_param('speed_record/timeout')
        if isinstance(self.timeout, str) and self.timeout == 'inf':
            self.timeout = np.inf

        self.EMERGENCY_STOP = False
        self.ESTOP_THRESH = 30
        
        rospy.Subscriber('sensors/ultrasound_0', Range, self.callback_left)
        rospy.Subscriber('sensors/ultrasound_1', Range, self.callback_right)

        self.car = Car()
        self.car.MAX_THROTTLE_ABS = .5

        self.smoother = Smoother(10)

        self.loop()

    def loop(self):
        self.car.throttle = self.v
        self.car.steering = self.w

        end_time = time.time() + self.timeout
        while not rospy.is_shutdown() and time.time() < end_time and not self.EMERGENCY_STOP:
            rospy.rostime.wallsleep(0.5)
        
        self.car.throttle = 0
        time.sleep(.1)
        self.car.steering = 0

    @staticmethod
    def _get_range(msg):
        return min(msg.max_range, max(msg.min_range, msg.range))

    def callback_left(self, r):
        self.write_range('l', r)

    def callback_right(self, r):
        self.write_range('r', r)

    @property
    def EMERGENCY_STOP(self):
        if hasattr(self, '_estop'):
            return self._estop

    @EMERGENCY_STOP.setter
    def EMERGENCY_STOP(self, value):
        self._estop = value
        if value:
            self.car.throttle = 0

    def write_range(self, which, r):

        if not self.f:
            return None

        d = self._get_range(r)

        dsmooth = self.smoother(d)
        print('dsmooth_%s = %s' % (which, dsmooth))
        if dsmooth > 10 and dsmooth < self.ESTOP_THRESH:
            print('smoothed distance is %s! Emergency stop.' % dsmooth)
            self.EMERGENCY_STOP = True

        if d == r.min_range: d = 'MIN'
        if d == r.max_range: d = 'MAX'

        self.write('%s,%s,%s,%s,%s' % (
            time.time(), 
            self.v, self.w,
            which,
            d
        ))

    def write(self, line):
        if self.f:
            self.f.write(line + '\n')
            self.f.flush()


if __name__ == '__main__':
    SpeedRecorder()
