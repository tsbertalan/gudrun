#!/usr/bin/env python
from __future__ import print_function
from subprocess import check_output
import serial, time, rospy

from std_msgs.msg import Header, Int32, Float32

class Encoder(object):

    def __init__(self,
        BAUDRATE = 115200,
        PORT = None,
        # READ_RATE=30, # max speed seen 1670 or so
        READ_RATE = 20,  # 1639
        # READ_RATE = 10,  # 1552
        ):
        rospy.init_node('listen_to_encoder')
        
        if PORT is None:
            PORT = check_output(['rosrun', 'gudrun_sensors', 'get_usb_device_by_ID.py', 'encoder_micro']).strip()
        print('Connecting to %s on port %s.' % (type(self).__name__, PORT))


        self.BAUDRATE = BAUDRATE
        self.PORT = PORT
        self.READ_RATE = READ_RATE

        self.ser = serial.Serial(self.PORT, self.BAUDRATE, timeout=.1)

        self.position_publisher = rospy.Publisher('motor_encoder_count', Int32, queue_size=1)
        self.speed_publisher = rospy.Publisher('motor_encoder_speed', Float32, queue_size=1)
        self.messages = Int32(), Float32()

        WHEEL_CIRCUMFRENCE = (
            3.14159 # circumfrence [diameters]
            * 2.0   # circumfrence [in]
            * 2.54  # circumfrence [cm]
            / 100.  # circumfrence [m]
        )
        GEAR_RATIO = 90.0 / 12.0 / 1.5 # 90 is spur; 12 is pinion; 1.5 quotient is emperical (differential maybe?)
        ENCODER_CPR = 48.0 # [count/rev]
        self.CPS_TO_SPEED = (
            1.0                   # motor [count/s]
            / ENCODER_CPR         # motor [rev/s]
            / GEAR_RATIO          # wheel [rev/s]
            * WHEEL_CIRCUMFRENCE  # wheel [m/s]
        )

        self.loop()

    def loop(self):

        t_last = time.time()
        count_last = 0
        while not rospy.is_shutdown():
            if self.ser.in_waiting == 0:
                continue
            l = ''
            while self.ser.in_waiting > 0:
                l = self.ser.readline().strip()

            if len(l.strip()) == 0:
                # Likely, we hit the serial timeout because the car isn't moving.
                count = count_last

            else:
                items = l.split(',')
                if len(items) == 1:
                    count = int(items[0])
                else:
                    import warnings
                    warnings.warn('Failed to parse serial line: "%s"' % l)
                    continue

            t = time.time()
            if t - t_last > 1. / self.READ_RATE:

                    dcount = count - count_last
                    count_last = count

                    dt = t - t_last
                    t_last = t

                    counts_per_second = float(dcount) / dt

                    self.messages[0].data = count
                    self.messages[1].data = counts_per_second * self.CPS_TO_SPEED
                    if abs(self.messages[1].data ) < 100:
                        self.position_publisher.publish(self.messages[0])
                        self.speed_publisher.publish(self.messages[1])

        if rospy.is_shutdown():
            print('Caught shutdown signal in listen_to_encoder!')


if __name__ == '__main__':

    try:
        Encoder()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start encoder node.')
