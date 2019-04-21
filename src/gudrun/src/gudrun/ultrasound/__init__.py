from __future__ import print_function
from subprocess import check_output
import serial, time, rospy

from sensor_msgs.msg import Range
from std_msgs.msg import Header

class UltrasoundNode(object):

    def __init__(self,
        NUM_SENSORS = 2,
        BAUDRATE = 19200,
        PORT = None,
        READ_RATE = 10,
        ):
        if PORT is None:
            PORT = check_output(['rosrun', 'gudrun', 'get_usb_device_by_ID.py', 'encoder_mega']).strip(),

        rospy.init_node('listen_to_ultrasound')

        self.NUM_SENSORS = NUM_SENSORS
        self.BAUDRATE = BAUDRATE
        self.PORT = PORT
        self.READ_RATE = READ_RATE

        self.ser = serial.Serial(self.PORT, self.BAUDRATE)

        self.publishers = [
            rospy.Publisher('sensors/ultrasound_%d' % i, Range, queue_size=1)
            for i in range(self.NUM_SENSORS)
        ]

        m = Range()
        m.radiation_type = Range.ULTRASOUND
        m.field_of_view = 22.5 * 3.14159 / 180.
        m.min_range = 6.
        m.max_range = 200.
        m.range = 0
        m.header = Header()
        self.message = m

        self.loop()

    def loop(self):

        t_last = time.time()
        while not rospy.is_shutdown():
            l = self.ser.readline().strip()
            now = rospy.Time.now()

            t = time.time()
            if t - t_last > 1. / self.READ_RATE:

                items = l.split(',')
                if len(items) == self.NUM_SENSORS:
                    try:
                        distances = [float(i) for i in items]
                    
                        t_last = t
                        for pub, distance in zip(self.publishers, distances):
                            self.message.range = distance
                            self.message.header.stamp = now
                            pub.publish(self.message)

                    except ValueError:
                        import warnings
                        warnings.warn('Failed to parse serial line: "%s"' % l)

