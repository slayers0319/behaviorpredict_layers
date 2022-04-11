#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import random
import math


def talker():
    rospy.init_node('pedestrain_behavior', anonymous=True)
    pub = rospy.Publisher('back', String, queue_size=1)
    rate = rospy.Rate(10) # 1hz
    i = 0
    while not rospy.is_shutdown():
        rate.sleep()
        if i<50:
            pub.publish('action:go')
            i=i+1
        else:
            pub.publish('action:stop')
            i=i+1
            if i>=100:
                i=0

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
