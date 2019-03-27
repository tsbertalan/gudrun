#!/usr/bin/env python
from __future__ import print_function
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32

from simple_pid import PID

from teleop import Car

class AckermanMotorController(object):

    def __init__(self, run=True, PID_update_rate=20):
        rospy.init_node('ackerman_motor_controller')

        self.car = Car()

        rospy.Subscriber('ackermann_cmd', AckermannDriveStamped, self.callback)
        rospy.Subscriber('motor_encoder_speed', Float32, self._son_do_you_know_how_fast_you_were_going)
        self._speed = 0

        # Set the PID's "sample_time" to smaller than our own likely update speed,
        # so we will compute a new control value every call.
        self.pid = PID(1, 0, 0, setpoint=0, sample_time=1e-6)

        GEAR_RATIO = 90.0 / 12.0 / 1.5 # 90 is spur; 12 is pinion; 1.5 quotient is emperical (differential maybe?)
        ENCODER_CPR = 48.0 # [count/rev]
        WHEEL_CIRCUMFRENCE = (
            3.14159 # circumfrence [diameters]
            * 2.0   # circumfrence [in]
            * 2.54  # circumfrence [cm]
            / 100.  # circumfrence [m]
        )
        self.CPS_TO_SPEED = (
            1.0                   # motor [count/s]
            / ENCODER_CPR         # motor [rev/s]
            / GEAR_RATIO          # wheel [rev/s]
            * WHEEL_CIRCUMFRENCE  # wheel [m/s]
        )

        self.PID_update_timer = rospy.Rate(PID_update_rate)
        self._cps = self._speed = 0

        if run: self.spin()

    def callback(self, message):
        print('Got ackeramn message:', message)
        self.set_steering(message.drive.steering_angle)
        self.set_throttle(message.drive.speed)

        # I'm ignoring, for now, all the velocity/acceleration/jerk parts of the message.
        # It should be noted that the Mini Maestro servo controller does have options for
        # acceleration (and jerk?) limiting that should be considered, though they might
        # need to be translated from [m/s/s...] to [motor encoder count/s] terms.

    def set_throttle(self, speed_target):
        self.pid.setpoint = speed_target

    def set_steering(self, angle):
        # Calibration "curve" is a simple multiplier.
        # Since "car" takes positive to be right turn, we flip the sign.
        self.car.steering = -angle * 1.65849761

    def _son_do_you_know_how_fast_you_were_going(self, msg):
        print('Noted:', msg.data, '[ticks/s]')
        # TODO: Fix the encoder so that we can get direction as well as speed.
        # This approach will work ok, I guess, for incremental changes.
        # But there will certainly be weirdly buggy edge cases.
        self._cps = msg.data * (1. if self.pid.setpoint >= 0 else -1.)
        self._speed = self._cps * self.CPS_TO_SPEED

    def spin(self):
        while not rospy.is_shutdown():
            self.update_pid()
            print('is_shutdown:', rospy.is_shutdown())
            self.PID_update_timer.sleep()

    def update_pid(self):
        command = self.pid.setpoint
        if command != 0:
            control = self.pid(self._speed)
            # Ensure the sign of the command always matches the sign of the requested direction.
            # This is a bad hack.
            sign = lambda k: (-1. if k < 0 else 1.)
            if sign(command) != sign(control):
                control *= -1.
        else:
            control = 0

        throttle = self.pseudospeed_to_throttle(control)
        print('speed=', self._speed, '[m/s]; setpoint=', command, '[m/s]; throttle=', throttle, '[a.u.]')
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
    AckermanMotorController()
