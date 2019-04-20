#!/usr/bin/env python
from gudrun.encoder import EncoderNode
import rospy

try:
    EncoderNode()
except rospy.ROSInterruptException:
    rospy.logerr('Could not start encoder node.')
