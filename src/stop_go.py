#!/usr/bin/env python

#from launch_demo import launch_demo
import imp
import rospy
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from tf_conversions import transformations
from tf.transformations import euler_from_quaternion
from point_nav import navigation_demo

class action_handle:
    def __init__(self):
        rospy.init_node('stop_go',anonymous=True)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.get_goal)
        rospy.Subscriber('/action_go', String, self.action_go, queue_size=1)
        rospy.Subscriber('/action_stop', String, self.action_stop, queue_size=1)
        self.nav = navigation_demo()
        self.goal = PoseStamped()
        r = rospy.Rate(5)

    def get_goal(self, data):
        self.goal = data
        print(self.goal)
        self.nav.goto(self.goal, 'map')
        
    def action_go(self, data):
        print('action:go')
        state = self.nav.move_base.get_state()
        if state!=GoalStatus.ACTIVE and state!=GoalStatus.LOST and state!=GoalStatus.ABORTED:
            print("====================================")
            print(state)
            self.nav.goto(self.goal, 'map')

    def action_stop(self, data):
        print('action:stop')
        state = self.nav.move_base.get_state()
        i=0
        while state!=GoalStatus.PREEMPTED and state!=GoalStatus.LOST and state!=GoalStatus.ABORTED:
            self.nav.cancel()
            r.sleep()
            print("====================================")
            print('i=',i)
            i=i+1
            state = self.nav.move_base.get_state()


A = action_handle()
r = rospy.Rate(5)
print("start")
while not rospy.is_shutdown():
    # 
    # print(A.nav.move_base.get_state())
    r.sleep()
