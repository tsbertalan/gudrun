from __future__ import print_function
import time, datetime
import numpy as np
from gudrun.imu import IMU

imu = IMU()

def get_time_micros():
    return int((datetime.datetime.utcnow() - datetime.datetime(1970, 1, 1)).total_seconds() * 1000000)
    return(int(round(time.time() * 1000000)))
    
micros = get_time_micros()

for values in imu.stream():

    values = tuple(values)

    current_time = get_time_micros()
    time_elapsed = current_time - micros
    micros = current_time

    print("Time elapsed since last (us): " + str(time_elapsed))
    print("------")
    print("Accelerations: %.2f %.2f %.2f" % values[0:3])
    print("Angular rates: %.2f %.2f %.2f" % values[3:6])
    print("Orientation:   %.2f %.2f %.2f" % values[6:9])
    print(u"{}[2J{}[;H".format(chr(27), chr(27)), end="")  # clear screen https://stackoverflow.com/a/2084521/1224886
