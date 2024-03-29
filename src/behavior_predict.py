#!/usr/bin/env python
import math
import time
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point


robot_speed = 0
time_start = 0
flag = 0

def abs_vector(x, y):
    """
    return absolute value of velocity
    """
    return math.sqrt(math.pow(x,2) + math.pow(y,2))

def scale(c, point):
    return Point(c*point.x, c*point.y,0)

def normalize_vector(point):
    return Point(point.x/abs_vector(point.x, point.y),point.y/abs_vector(point.x, point.y),0)

def rotate_vector(point, angle):
    P=Point()
    P.x=point.x*math.cos(angle*math.pi/180)-point.y*math.sin(angle*math.pi/180)
    P.y=point.x*math.sin(angle*math.pi/180)+point.y*math.cos(angle*math.pi/180)
    return P

def predict(data):
    global time_start
    global flag
    flag = 1
    time_start = time.time()
    if data.data=="clear":
        pub_point.publish("clear")
        return
    
    global robot_speed

    data_dic = {}   #data_dic = {'P':(px, py), 'V':(vx, vy)}
    splited = data.data.split(',')
    data_dic['P'] = Point(float(splited[1]),float(splited[2]),0)   #point of person
    data_dic['V'] = Point(float(splited[4]),float(splited[5]),0)   #velocity vector for person
    if (data_dic['P'].x*data_dic['V'].x)>0:
        return
    
    #y_cross = py-(px*vy/vx) the point which person crossing the front of robot
    y_cross = data_dic['P'].y-(data_dic['P'].x*data_dic['V'].y/data_dic['V'].x) 
    P_speed = abs_vector(data_dic['V'].x, data_dic['V'].y)  #absolute value of velocity vector for person
    P_ycross_distance = abs_vector(data_dic['P'].x, data_dic['P'].y-y_cross)    #the distance form person to the crossing point
    cross_time = P_ycross_distance/P_speed
    robot_predict_y = robot_speed*cross_time    #robot position after corss_time seconds

    #---------------------------------
    if abs(y_cross-robot_predict_y)<0.375:
        V_scale = normalize_vector(data_dic['V'])
        V_N = Point(-data_dic['V'].y,data_dic['V'].x,0)
        V_N = scale(0.375, normalize_vector(V_N))
        point_list=[]
        point_list.append(Point(V_N.x, V_N.y+y_cross, 0))
        #result = result+",Point,"+str(V_N.x+V_scale.x)+","+str(V_N.y+y_cross+V_scale.y)
        for i in range(4):
            V_N=rotate_vector(V_N, 45)
            point_list.append(Point(V_N.x, V_N.y+y_cross, 0))

        point_list.append(Point(point_list[-1].x+V_scale.x, point_list[-1].y+V_scale.y, 0))
        point_list.append(Point(point_list[0].x+V_scale.x, point_list[0].y+V_scale.y, 0))
        result=""

        for i in point_list:
            result=result+"Point,"+str(i.x)+","+str(i.y)+","
        result = result[:-1]
        #rospy.loginfo(result)
        pub_point.publish(result)
    else:
        return

def getRobotSpeed(data):
    global robot_speed

    robot_speed = data.twist.twist.linear.x
    robot_speed = round(robot_speed, 3)
    # rospy.loginfo(robot_speed)

rospy.init_node('behavior_predict',anonymous=True)
rospy.Subscriber('/behavior', String, predict, queue_size=1)
rospy.Subscriber('/odom', Odometry, getRobotSpeed, queue_size=1)
pub_point = rospy.Publisher('/behaviorpredict', String, queue_size=1)

r = rospy.Rate(4)

while not rospy.is_shutdown():
    time_interval = time.time()-time_start
    if time_interval>=1.5 and flag==1:
        flag = 0
        rospy.loginfo("claer")
        pub_point.publish("clear")
    r.sleep()