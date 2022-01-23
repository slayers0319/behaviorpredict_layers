#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import random

data_x = 0.0
data_y = 0.0
data_yaw = 0.0

def talker():
    
    global name, data_x, data_y, data_yaw, data
    rospy.init_node('pedestrain_behavior', anonymous=True)
    pub = rospy.Publisher('behavior', String, queue_size=10)
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        data = String()
        n=input()
        if n==-1:
            data.data = "clear"
            rospy.loginfo(data)
            pub.publish(data)
            break
        elif n==1:
            data_x = random.uniform(-0.1,-0.9)
            data_y = random.uniform(1.5,2)
            for i in range(1):
                rate.sleep()
                name = "R"
                data.data = name+","+str(data_x)+","+str(data_y)
                rospy.loginfo(data)
                pub.publish(data)
                # data_x = data_x+0.09
                # data_y = data_y-0.1
        elif n==2:
            for i in range(3):
                rate.sleep()
                name = "L"
                data_x = random.uniform(0.25,0.3)
                data_y = random.uniform(0.5,1.0)
                data.data = name+","+str(data_x)+","+str(data_y)
                rospy.loginfo(data)
                pub.publish(data)
        elif n==3:
            name = "R"
            data_x = random.uniform(0.1,0.5)
            data_y = random.uniform(0.5,1.0)
            data.data = name+","+str(data_x)+","+str(data_y)
            rospy.loginfo(data)
            pub.publish(data)
        elif n==4:
            name = "L"
            data_x = random.uniform(-0.5,-0.1)
            data_y = random.uniform(0.5,1.0)
            data.data = name+","+str(data_x)+","+str(data_y)
            rospy.loginfo(data)
            pub.publish(data)
            
        rospy.loginfo("-------------")
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
