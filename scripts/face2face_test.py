#!/usr/bin/env python
import rospy
from std_msgs.msg import String

rospy.init_node('face2face_test',anonymous=True)
pub_point = rospy.Publisher('/fusion_data', String, queue_size=1)


r = rospy.Rate(10)
print("start")
output = 'personF,1.1,1.2,B,personR,-1.05,1.12,B,personR,-2.23,1.11,B,personL,2.13,1.07,B'
# output = 'personR,-2.23,1.11,B,personL,2.13,3,B'
for i in range(10):
    pub_point.publish(output)
    print(output)
    r.sleep()