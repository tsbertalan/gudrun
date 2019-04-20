#!/usr/bin/env python
from gudrun.encoder import Encoder

try:
    Encoder()
except rospy.ROSInterruptException:
    rospy.logerr('Could not start encoder node.')
