#!/usr/bin/env python
from geometry_msgs.msg import Point
import time
import math
import rospy

class X_Y:
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

class person:
    def __init__(self, pose, speed):
        self.pose = Point(pose[0], pose[1], 0)#X_Y(pose[0], pose[1])
        self.speed = Point(speed[0], speed[1], 0)
        self.start_pose = Point(pose[0], pose[1], 0)
        self.time_start = 0
        self.time_interval = 0
        self.goal = 3
        
    def set_goal(self, goal=3):
        self.goal = goal

    def start(self):
        self.start_pose.x = self.pose.x
        self.start_pose.y = self.pose.y
        self.time_start = time.time() 

    def move_distance(self):
        return  math.sqrt((self.pose.x-self.start_pose.x)**2+(self.pose.y-self.start_pose.y)**2) 

    def move(self):
        self.time_interval = time.time()-self.time_start
        self.pose.x = self.pose.x+self.speed.x*self.time_interval
        self.pose.y = self.pose.y+self.speed.y*self.time_interval
        distance = self.move_distance()
        if distance>=self.goal: #reverse
            self.speed.x=-self.speed.x
            self.speed.y=-self.speed.y
            self.start()
        self.time_start = time.time()
    
    def print_pose(self):
        rospy.loginfo("pose:(%f, %f)" % (self.pose.x, self.pose.y))

