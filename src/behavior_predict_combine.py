#!/usr/bin/env python
import math
import time
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from tf import TransformListener
from geometry_msgs.msg import Point, PointStamped
from geometry_msgs.msg import PoseWithCovarianceStamped


robot_speed = 0
time_start = 0
flag = 0
calcualte_impact = 1
first = 0
min_dis = 1000
last_point = Point(0, 0, 0)
robot_pose = Point(0, 0, 0)
last_point_map = Point(0, 0, 0)
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
    global calcualte_impact
    global first
    global last_point
    global last_point_stamped
    global last_point_map
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

    if calcualte_impact==2:
        return

    if calcualte_impact==1:
    # calculate impact
        if (data_dic['P'].x*data_dic['V'].x)>0:
            return
        #y_cross = py-(px*vy/vx) the point which person crossing the front of robot
        y_cross = data_dic['P'].y-(data_dic['P'].x*data_dic['V'].y/data_dic['V'].x) 
        P_speed = abs_vector(data_dic['V'].x, data_dic['V'].y)  #absolute value of velocity vector for person
        P_ycross_distance = abs_vector(data_dic['P'].x, data_dic['P'].y-y_cross)    #the distance form person to the crossing point
        cross_time = P_ycross_distance/P_speed
        robot_predict_y = robot_speed*cross_time    #robot position after corss_time seconds
        # if abs(y_cross-robot_predict_y)<0.375:
        if True:
            calcualte_impact = 0
            first = 1
            rospy.loginfo("calcualte_impact = "+str(calcualte_impact))

    if calcualte_impact==0:
        if first==0:
            new_point_dis = abs_vector(last_point_stamped.point.x-data_dic['P'].x, last_point_stamped.point.y-data_dic['P'].y)
            if new_point_dis>0.5:
                calcualte_impact = 2
                rospy.loginfo("calcualte_impact = "+str(calcualte_impact))
                return
        first = 0
        V_scale = normalize_vector(data_dic['V'])
        V_N = Point(-data_dic['V'].y,data_dic['V'].x,0)
        V_N = scale(0.375, normalize_vector(V_N))
        point_list=[]
        point_list.append(Point(V_N.x+data_dic['P'].x+V_scale.x*0.55, V_N.y+data_dic['P'].y+V_scale.y*0.55, 0))
        #result = result+",Point,"+str(V_N.x+V_scale.x)+","+str(V_N.y+y_cross+V_scale.y)
        for i in range(4):
            V_N=rotate_vector(V_N, 45)
            point_list.append(Point(V_N.x+data_dic['P'].x+V_scale.x*0.55, V_N.y+data_dic['P'].y+V_scale.y*0.55, 0))
            if i==1:
                last_point_stamped.point = point_list[-1]
                last_point_map = TF_listener.transformPoint('odom',last_point_stamped)


        point_list.append(Point(point_list[-1].x+V_scale.x*1.3+V_scale.x*0.55, point_list[-1].y+V_scale.y*1.3+V_scale.y*0.55, 0))
        point_list.append(Point(point_list[0].x+V_scale.x*1.3+V_scale.x*0.55, point_list[0].y+V_scale.y*1.3+V_scale.y*0.55, 0))
        result=""

        for i in point_list:
            result=result+"Point,"+str(i.x)+","+str(i.y)+","
        result = result[:-1]

        pub_point.publish(result)

def getRobotSpeed(data):
    global robot_speed
    robot_speed = data.twist.twist.linear.x
    robot_speed = round(robot_speed, 3)
    global robot_pose
    robot_pose = data.pose.pose.position
    
# def getRobotPosition(data):
#     global robot_pose
#     robot_pose = data.pose.pose.position
#     rospy.loginfo(data)


rospy.init_node('behavior_predict',anonymous=True)
rospy.Subscriber('/behavior', String, predict, queue_size=1)
rospy.Subscriber('/odom', Odometry, getRobotSpeed, queue_size=1)
#rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, getRobotPosition, queue_size=1)
pub_point = rospy.Publisher('/behaviorpredict', String, queue_size=1)


# set tf
TF_listener = TransformListener()
TF_listener.waitForTransform('odom', 'base_footprint', rospy.Time(0), rospy.Duration(20000.0))

# set point stamped
last_point_stamped = PointStamped()
last_point_stamped.header.frame_id = '/base_footprint'
last_point_stamped.header.stamp = rospy.Time(0)
last_point_stamped.point = Point(0, 0, 0)

r = rospy.Rate(4)

pub_point.publish("clear")
while not rospy.is_shutdown():
    # time clear
    time_interval = time.time()-time_start
    if time_interval>=1.5 and flag==1:
        flag = 0
        rospy.loginfo("in flag claer")
        calcualte_impact=1
        rospy.loginfo("calcualte_impact = "+str(calcualte_impact))
        pub_point.publish("clear")
        min_dis = 1000
    r.sleep()

    # distance clear
    if calcualte_impact==2:
        
        dis = abs_vector(last_point_map.point.x-robot_pose.x, last_point_map.point.y-robot_pose.y)
        print("dis = ",dis,"min = ", min_dis,"--- = ",dis-min_dis)
        if dis<min_dis:
            min_dis = dis
        elif (dis-min_dis)>0.1:
            rospy.loginfo("dis claer")
            pub_point.publish("clear")
            calcualte_impact=1
            rospy.loginfo("calcualte_impact = "+str(calcualte_impact))
            min_dis = 1000

   
