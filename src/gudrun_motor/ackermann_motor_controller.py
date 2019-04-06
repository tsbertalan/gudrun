#!/usr/bin/env python
from __future__ import print_function
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32

from simple_pid import PID

from teleop import Car

class AckermannMotorController(object):

    def __init__(self, PID_update_rate=20):
        rospy.init_node('ackermann_motor_controller')
        rospy.loginfo('Started ackermann_motor_controller')

        self.car = Car()
        self._command = 0

        rospy.Subscriber('ackermann_cmd', AckermannDriveStamped, self.callback)
        rospy.Subscriber('motor_encoder_speed', Float32, self._son_do_you_know_how_fast_you_were_going)
        self.cp_publisher = rospy.Publisher('speed_control/proportional', Float32, queue_size=10)
        self.ci_publisher = rospy.Publisher('speed_control/integral', Float32, queue_size=10)
        self.cd_publisher = rospy.Publisher('speed_control/derivative', Float32, queue_size=10)
        self.sp_publisher = rospy.Publisher('speed_control/setpoint', Float32, queue_size=10)
        self.pv_publisher = rospy.Publisher('speed_control/measured', Float32, queue_size=10)
        self.cv_publisher = rospy.Publisher('speed_control/control', Float32, queue_size=10)
        self.ce_publisher = rospy.Publisher('speed_control/error', Float32, queue_size=10)
        self._speed = 0

        # Set the PID's "sample_time" to smaller than our own likely update speed,
        # so we will compute a new control value every call.
        self.pid = PID(.5, 0.2, 0, setpoint=0, sample_time=1e-6, output_limits=(0, None), proportional_on_measurement=True)

        self.PID_update_timer = rospy.Rate(PID_update_rate)
        self._cps = self._speed = 0

        self.loop()

    def callback(self, message):
        self._command = message.drive.speed
        steering = message.drive.steering_angle
        if not self._forward:
            steering *= -1
        self.set_steering(steering)
        self.set_throttle(abs(message.drive.speed))

        # I'm ignoring, for now, all the velocity/acceleration/jerk parts of the message.
        # It should be noted that the Mini Maestro servo controller does have options for
        # acceleration (and jerk?) limiting that should be considered, though they might
        # need to be translated from [m/s/s...] to [motor encoder count/s] terms.

    @property
    def _forward(self):
        return self._command >= 0

    def set_throttle(self, speed_target):
        self.pid.setpoint = speed_target

    def set_steering(self, angle):
        # Calibration "curve" is a simple multiplier.
        # Since "car" takes positive to be right turn, we flip the sign.
        self.car.steering = -angle * 1.65849761

    def _son_do_you_know_how_fast_you_were_going(self, msg):
        # TODO: Fix the encoder so that we can get direction as well as speed.
        # This approach will work ok, I guess, for incremental changes.
        # But there will certainly be weirdly buggy edge cases.
        self._speed = msg.data

    def loop(self):
        while not rospy.is_shutdown():
            self.update_pid()
            try:
                self.PID_update_timer.sleep()
            except rospy.ROSInterruptException:
                print('Caught interrupt in ackermann_motor_controller!')
                break

        if rospy.is_shutdown():
            print('Caught shutdown signal in ackermann_motor_controller!')

    def update_pid(self):
        command = self._command
        control = self.pid(self._speed)

        # Ensure the sign of the command always matches the sign of the requested direction.
        # Our controller is like a gas pedal; always positive. But the lower-level Car
        # interface expects negative numbers for reverse drive.
        sign = lambda k: (-1. if k < 0 else 1.)
        if sign(command) != sign(control):
            control *= -1.

        if command == 0:
            control = 0

        cp, ci, cd = self.pid.components
        self.cp_publisher.publish(cp)
        self.ci_publisher.publish(ci)
        self.cd_publisher.publish(cd)

        self.sp_publisher.publish(abs(command))
        self.pv_publisher.publish(self._speed)
        self.cv_publisher.publish(abs(control))
        self.ce_publisher.publish(self._speed - abs(command))

        throttle = self.pseudospeed_to_throttle(control)
        self.car.throttle = throttle

    @staticmethod
    def pseudospeed_to_throttle(pseudospeed):
        # Calibration curve for speed=y from throttle=x
        # is given as two lines,
        # one for forward-driving, and one for reverse.
        #y - 0 = m * (x - xb)
        #x = y / m + xb

        m_reverse = 5.204
        xb_reverse = -.216
        m_forward = 6.462
        xb_forward = .215
        if pseudospeed == 0:
            return 0

        elif pseudospeed < 0:
            return pseudospeed / m_reverse + xb_reverse

        else:
            return pseudospeed / m_forward + xb_forward


if __name__ == '__main__':
    AckermannMotorController()
