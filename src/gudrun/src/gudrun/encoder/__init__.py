#!/usr/bin/env python
from __future__ import print_function
from subprocess import check_output
import serial, time, rospy
from collections import deque
from numpy import polyfit

from std_msgs.msg import Header, Int32, Float32

from gudrun.usb_device import USBDevice


class Encoder(USBDevice):

    product = 8036
    vendor = 2341

    def __init__(self, **kw):


        super(Encoder, self).__init__(**kw)


class EncoderNode(USBDevice):

    def __init__(self, READ_RATE = 20):
        rospy.init_node('listen_to_encoder')
        
        self.device = Encoder()

        self.READ_RATE = READ_RATE

        self.ser = self.device.ser

        self.position_publisher = rospy.Publisher('motor_encoder/count', Int32, queue_size=1)
        self.speed_publisher = rospy.Publisher('motor_encoder/velocity', Float32, queue_size=1)
        self.messages = Int32(), Float32()

        WHEEL_CIRCUMFRENCE = (
            3.14159 # circumfrence [diameters]
            * rospy.get_param('~meters_per_wheel_diameter', 105.) # circumfrence [m]
        )
        GEAR_RATIO = 90.0 / 12.0 * 1.4 # 90 is spur; 12 is pinion; final factor is emperical (differential maybe? Half encoder?)
        ENCODER_CPR = 20.0 # [count/rev]
        self.CPS_TO_SPEED = (
            1.0                   # motor [count/s]
            / ENCODER_CPR         # motor [rev/s]
            / GEAR_RATIO          # wheel [rev/s]
            * WHEEL_CIRCUMFRENCE  # wheel [m/s]
        )
        self.last_times = deque(maxlen=10)
        self.last_counts = deque(maxlen=10);

        self.loop()

    def loop(self, max_rate=4000):

        t_last = time.time()
        count_last = count = 0
        # max_rate limits cpu usage, which could otherwise be quite high
        # if we're constantly polling the serial port.
        # If the rate is low enough to to cause problems (which shouldn't be the case,
        # since the Serial object should buffer for us), I'll revisit this.
        rate = rospy.Rate(max_rate)
        while not rospy.is_shutdown():
            
            try:
                if self.ser.in_waiting == 0:
                    rate.sleep()
                else:

                    l = ''
                    while self.ser.in_waiting > 0:
                        l = self.ser.readline().strip()

                    if len(l.strip()) == 0:
                        # Likely, we hit the serial timeout because the car isn't moving.
                        count = count_last

                    else:
                        count_last = count
                        items = l.split(',')
                        if len(items) == 1:
                            count = int(items[0])
                        else:
                            rospy.logerr('Failed to parse serial line: "%s"' % l)
                    self.last_counts.append(count)
                    self.last_times.append(time.time())

                t = time.time()
                if t - t_last > 1. / self.READ_RATE and len(self.last_times) > 1:
                    t_last = t

                    counts_per_second, unused_intercept = polyfit(self.last_times, self.last_counts, 1)
                    speed = counts_per_second * self.CPS_TO_SPEED

                    # Sometimes we read 0 speed in error every other call; not sure why.
                    # Solve this with some basic smoothing.
                    # self.messages[1].data = self.speed_measurement_smoother(speed)
                    self.messages[1].data = speed
                    self.messages[0].data = count

                    if abs(self.messages[1].data) < 100:
                        self.position_publisher.publish(self.messages[0])
                        self.speed_publisher.publish(self.messages[1])

            except IOError as e:
                rospy.logerr('Caught serial error! %s' % e)
                del self.device
                self.device = Encoder()

        if rospy.is_shutdown():
            print('Caught shutdown signal in listen_to_encoder!')

