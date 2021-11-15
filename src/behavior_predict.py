#!/usr/bin/env python
import math
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point


robot_speed = 0

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

    
    # label = result[0]
    # pixel = float(result[1])
    # deep = float(result[2])/100

    # if label not in action_set:
    #     pub_flag.publish(1) #finished
    #     return
    
    # sleep = rospy.Rate(0.33)
    # wait = rospy.Rate(2)
    # if label=='w' or label=='W':
    #     rospy.loginfo("wwwwwwww")

    #     degree = (pixel/1920)*75
    #     beta = abs((75/2)-degree)

    #     if pixel>(1920/2):  #right (degree/180.0)*pi
    #         related_goal_x = (deep-0.3)*math.sin((beta/180.0)*pi)
    #         related_goal_y = (deep-0.3)*math.cos((beta/180.0)*pi)
    #     else:   #left
    #         related_goal_x = -(deep-0.3)*math.sin((beta/180.0)*pi)
    #         related_goal_y = (deep-0.3)*math.cos((beta/180.0)*pi)

    #     r = math.sqrt(math.pow(related_goal_x,2) + math.pow(related_goal_y,2))
    #     theta = math.atan2(related_goal_y, related_goal_x) - (pi/2)

    #     if ((theta + pi) <= 0):
    #         theta = (theta + 2 * pi)
        
    #     goal_x = robot_pose[0] + math.cos(theta + robot_pose[2])*r
    #     goal_y = robot_pose[1] + math.sin(theta + robot_pose[2])*r
        
    #     angle_to_goal = (theta + robot_pose[2])*180/pi

    #     back_pose = [robot_pose[0], robot_pose[1], robot_pose[2]*180/pi]
    #     nav.goto([goal_x, goal_y, angle_to_goal])

    # elif label=='a' or label=='A':    #rurn left
    #     rospy.loginfo("turn left")
    #     speed = Twist()
    #     speed.linear.x = 0
    #     speed.linear.y = 0
    #     speed.linear.z = 0
    #     speed.angular.x = 0
    #     speed.angular.y = 0
    #     speed.angular.z = -pi/6  #180/6=30 degree/sec
    #     pub_v.publish(speed)
    #     sleep.sleep()
    #     speed.angular.z = 0
    #     pub_v.publish(speed)
    # elif label=='d' or label=='D':    #rurn right
    #     rospy.loginfo("turn right")
    #     speed = Twist()
    #     speed.linear.x = 0
    #     speed.linear.y = 0
    #     speed.linear.z = 0
    #     speed.angular.x = 0
    #     speed.angular.y = 0
    #     speed.angular.z = pi/6  #180/6=30 degree/sec
    #     pub_v.publish(speed)
    #     sleep.sleep()
    #     speed.angular.z = 0
    #     pub_v.publish(speed)
    # elif label=='b' or label=='B':
    #     nav = navigation_demo()
    #     nav.goto(back_pose)

    # pub_flag.publish(1)

def getRobotSpeed(data):
    global robot_speed

    robot_speed = data.twist.twist.linear.x
    robot_speed = round(robot_speed, 3)
    # rospy.loginfo(robot_speed)


rospy.init_node('behavior_predict',anonymous=True)
rospy.Subscriber('/behavior', String, predict, queue_size=1)
rospy.Subscriber('/odom', Odometry, getRobotSpeed, queue_size=1)
pub_point = rospy.Publisher('/behaviorpredict', String, queue_size=1)

r = rospy.Rate(1)

while not rospy.is_shutdown():
    r.sleep()