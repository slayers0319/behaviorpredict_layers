#!/usr/bin/env python
import math
from rospy import timer
from tf import TransformListener
import rospy
from rospy.timer import sleep
from std_msgs.msg import String
from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import PointCloud
from people import *


class people:
    def __init__(self, people_list):
        self.people_list = people_list

    def start(self):
        for p in self.people_list:
            p.start()

    def set_goal(self, goal):
        for p in self.people_list:
            p.set_goal(goal)

    def move(self):
        for p in self.people_list:
            p.move()
    
    def pose_list(self):
        return [p.pose for p in self.people_list]

    def speed_list(self):
        return [p.speed for p in self.people_list]

def pub_marker(pedestrian):
    data=""
    for p in pedestrian.people_list:
        data=data+"people,"+str(p.pose.x)+","+str(p.pose.y)+",B,"
    #rospy.loginfo(data[:-1])
    pub_point.publish(data)

def pub_people_detect(point, speed):
    data="P,"+str(-point.y)+","+str(point.x)+",V,"+str(-speed.y)+","+str(speed.x)
    rospy.loginfo(data)
    pub_behavior.publish(data)

def nearest(points):
    index = -1
    d = 10000000
    for i in range(len(points)):
        if points[i].x<0:
            continue

        temp = points[i].x**2+points[i].y**2
        if temp<d:
            d = temp
            index = i
    if d==10000:
        return -1
    else:
        return index


#set node
rospy.init_node('poeple_sim',anonymous=True)
pub_point = rospy.Publisher('/marker', String, queue_size=1)
pub_behavior = rospy.Publisher('/behavior', String, queue_size=1)
R = rospy.Rate(6)

#set pedestrian
people_list = [person([-2,1.5], [-0.3,0]), person([-4.5,-0], [0.6,0]), person([-4.5,-1.5], [0.4,0])]
#people_list = [person([-2,1.5], [-0.5,0])]
pedestrian = people(people_list)
pedestrian.start()
pedestrian.set_goal(2.5)

#set tf
TF_listener = TransformListener()
TF_listener.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(20000.0))

#set point cloud
PC_pose=PointCloud()
PC_pose.header.frame_id = '/map'
PC_pose.header.stamp = rospy.Time(0)

PC_speed=PointCloud()
PC_speed.header.frame_id = '/map'
PC_speed.header.stamp = rospy.Time(0)

zero_point = PointStamped()
zero_point.header.frame_id = '/map'
zero_point.header.stamp = rospy.Time(0)
zero_point.point = Point(0, 0, 0)

#simulation start
while not rospy.is_shutdown():
    pedestrian.move()
    #pedestrian.print_pose()
    
    #transfer pose
    PC_pose.points = pedestrian.pose_list()
    PC_pose_foot = TF_listener.transformPointCloud('base_footprint', PC_pose)
    #rospy.loginfo(PC_pose_foot.points[0])

    #transfer speed
    zero_foot = TF_listener.transformPoint('base_footprint',zero_point)
    #rospy.loginfo(zero_foot)
    PC_speed.points = pedestrian.speed_list()
    PC_speed_foot = TF_listener.transformPointCloud('base_footprint', PC_speed)
    for i in range(len(PC_speed_foot.points)):
        PC_speed_foot.points[i].x = PC_speed_foot.points[i].x-zero_foot.point.x
        PC_speed_foot.points[i].y = PC_speed_foot.points[i].y-zero_foot.point.y

    #calculate nearest point
    index = nearest(PC_pose_foot.points)
    # rospy.loginfo(index)
    # rospy.loginfo(PC_pose_foot.points[index])
    if index!=-1:
        # rospy.loginfo(PC_pose.points[i])
        # rospy.loginfo(PC_pose_foot.points[i])
        pub_people_detect(PC_pose_foot.points[index], PC_speed_foot.points[index])
    else:
        rospy.loginfo("clear")
        pub_behavior.publish("clear")

    pub_marker(pedestrian)
    R.sleep()

