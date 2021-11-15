#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import random
import math

data_x = 0.0
data_y = 0.0
name = "P"
speed = ",V, 0.7, 0.3"

def getRobotSpeed(data):
    global robot_speed

    robot_speed = data.twist.twist.linear.x
    robot_speed = round(robot_speed, 3)

def talker():
    
    global name, data_x, data_y, robot_speed
    rospy.init_node('pedestrain_behavior', anonymous=True)
    rospy.Subscriber('/odom', Odometry, getRobotSpeed, queue_size=1)
    pub = rospy.Publisher('behavior', String, queue_size=1)
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        data = String()
        n=input()
        if n==-1:
            data.data = "clear"
            rospy.loginfo(data)
            pub.publish(data)
            break
        elif n==0:
            t=random.uniform(6,8)
            p_speed=random.uniform(0.6,1)
            y_cross=t*robot_speed
            v = [random.choice((-1, 1))*random.uniform(0.5,0.6),random.uniform(-0.5,0.5)]
            v_abs = math.sqrt(math.pow(v[0],2)+math.pow(v[1],2))
            v_speed = [i*p_speed/v_abs for i in v]
            P = [(-i)*t for i in v_speed]
            P[1] = P[1]+y_cross
            data.data = "P,"+str(P[0])+","+str(P[1])+",V,"+str(v_speed[0])+","+str(v_speed[1])
            rospy.loginfo(data)
            pub.publish(data)
        elif n==1:
            data_x = random.uniform(-0.1,-0.9)
            data_y = random.uniform(1.5,2)
            for i in range(1):
                rate.sleep()
                data.data = name+","+str(data_x)+","+str(data_y)+speed
                rospy.loginfo(data)
                pub.publish(data)
                # data_x = data_x+0.09
                # data_y = data_y-0.1
        elif n==2:
            for i in range(3):
                rate.sleep()
                data_x = random.uniform(0.25,0.3)
                data_y = random.uniform(0.5,1.0)
                data.data = name+","+str(data_x)+","+str(data_y)+speed
                rospy.loginfo(data)
                pub.publish(data)
        elif n==3:
            data_x = random.uniform(0.1,0.5)
            data_y = random.uniform(0.5,1.0)
            data.data = name+","+str(data_x)+","+str(data_y)+speed
            rospy.loginfo(data)
            pub.publish(data)
        elif n==4:
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
