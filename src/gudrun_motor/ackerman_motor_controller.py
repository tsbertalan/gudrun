#!/usr/bin/env python
from __future__ import print_function
import rospy
from ackermann_msgs.msg import AckermannDriveStamped


from teleop import Car

class AckermanMotorController(object):

    def __init__(self, run=True):
        rospy.init_node('ackerman_motor_controller')

        self.car = Car()

        rospy.Subscriber('/ackermann_cmd', AckermannDriveStamped, self.callback)

        if run: rospy.spin()

    def set_throttle(self, speed):

        # Calibration curve for speed=y from throttle=x
        # is given as two lines,
        # one for forward-driving, and one for reverse.
        #y - 0 = m * (x - xb)
        #x = y / m + xb

        m_reverse = 5.204
        xb_reverse = .216
        m_forward = 6.462
        xb_forward = -.215

        if speed == 0:
            self.car.throttle = 0

        elif speed > 0:
            self.car.throttle = speed / m_reverse + xb_reverse

        else:
            self.car.throttle = speed / m_forward + xb_forward

    def set_steering(self, angle):
        # Calibration "curve" is a simple multiplier.
        # Since "car" takes positive to be right turn, we flip the sign.
        self.car.steering = -angle * 1.65849761

    def callback(self, message):
        self.set_steering(message.drive.steering_angle)
        self.set_throttle(message.drive.speed)

        # Ignoring, for now, all the velocity/acceleration/jerk parts of the message.


if __name__ == '__main__':
    AckermanMotorController()
