#!/usr/bin/env python
from __future__ import print_function
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32

from simple_pid import PID

from teleop import Car, Smoother

class AckermannMotorController(object):

    def __init__(self, PID_update_rate=20):
        rospy.init_node('ackermann_motor_controller')
        rospy.loginfo('Started ackermann_motor_controller')

        self.car = Car()
        self._command = 0

        rospy.Subscriber('ackermann_cmd', AckermannDriveStamped, self.callback)
        rospy.Subscriber('motor_encoder/velocity', Float32, self._son_do_you_know_how_fast_you_were_going)
        self.cp_publisher = rospy.Publisher('speed_control/proportional', Float32, queue_size=10)
        self.ci_publisher = rospy.Publisher('speed_control/integral', Float32, queue_size=10)
        self.cd_publisher = rospy.Publisher('speed_control/derivative', Float32, queue_size=10)
        self.sp_publisher = rospy.Publisher('speed_control/setpoint', Float32, queue_size=10)
        self.pv_publisher = rospy.Publisher('speed_control/measured', Float32, queue_size=10)
        self.cv_publisher = rospy.Publisher('speed_control/control', Float32, queue_size=10)
        self.ce_publisher = rospy.Publisher('speed_control/error', Float32, queue_size=10)
        self._velocity = 0

        # Set the PID's "sample_time" to smaller than our own likely update speed,
        # so we will compute a new control value every call.
        self.pid = PID(1, 0.5, 0.25, setpoint=0, sample_time=1e-6, proportional_on_measurement=False)

        self.input_velocity_smoother = Smoother(1)
        if self.input_velocity_smoother.maxlen < 8:
        	# dangerous to  use derivative term with unsmoothed data.
        	self.pid.Kd = 0

        self.PID_update_timer = rospy.Rate(PID_update_rate)
        self._cps = self._velocity = 0

        self.loop()

    def callback(self, message):
        self._command = message.drive.speed
        steering = message.drive.steering_angle
        if not self._forward:
            # TODO: Handle this in from the path planning side, if possible, maybe.
            steering *= -1
        self.set_steering(steering)
        self.set_throttle(message.drive.speed)

        # I'm ignoring, for now, all the velocity/acceleration/jerk parts of the message.
        # It should be noted that the Mini Maestro servo controller does have options for
        # acceleration (and jerk?) limiting that should be considered, though they might
        # need to be translated from [m/s/s...] to [motor encoder count/s] terms.

    @property
    def _forward(self):
        return self._command >= 0

    def set_throttle(self, velocity_target):
        self.pid.setpoint = velocity_target

    def set_steering(self, angle):
        # Calibration "curve" is a simple multiplier.
        # Since "car" takes positive to be right turn, we flip the sign.
        self.car.steering = -angle * 1.65849761

    def _son_do_you_know_how_fast_you_were_going(self, msg):
        # TODO: Fix the encoder so that we can get direction as well as speed.
        # This approach will work ok, I guess, for incremental changes.
        # But there will certainly be weirdly buggy edge cases.
        if self.input_velocity_smoother.maxlen == 1:
        	self._velocity = msg.data
    	else:
        	self._velocity = self.input_velocity_smoother(msg.data)

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

    def update_pid(self, MIN_MOTOR=.01):
        command = self._command
        control = self.pid(self._velocity)

        if command == 0:
            control = 0
            # Reset the integral term when stopping.
            self.pid._integral = 0

        # Ensure the controller doesn't fall to zero (and brake the motor)
        # just because we're overspeeding.
        elif command > 0:
            control = max(control, MIN_MOTOR)
        else:
            control = min(control, -MIN_MOTOR)

        cp, ci, cd = self.pid.components
        self.cp_publisher.publish(cp)
        self.ci_publisher.publish(ci)
        self.cd_publisher.publish(cd)

        self.sp_publisher.publish(command)
        self.pv_publisher.publish(self._velocity)
        self.cv_publisher.publish(control)
        self.ce_publisher.publish(self._velocity - command)

        throttle = self.pseudovelocity_to_throttle(control)
        self.car.throttle = throttle

    @staticmethod
    def pseudovelocity_to_throttle(pseudovelocity):
        # Calibration curve for speed=y from throttle=x
        # is given as two lines,
        # one for forward-driving, and one for reverse.
        #y - 0 = m * (x - xb)
        #x = y / m + xb

        m_reverse = 5.204 * 1
        xb_reverse = -.216
        m_forward = 6.462 * 1
        xb_forward = .215
        if pseudovelocity == 0:
            return 0

        elif pseudovelocity < 0:
            return pseudovelocity / m_reverse + xb_reverse

        else:
            return pseudovelocity / m_forward + xb_forward


if __name__ == '__main__':
    AckermannMotorController()
