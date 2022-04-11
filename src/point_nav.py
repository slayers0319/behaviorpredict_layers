#!/usr/bin/env python

#from launch_demo import launch_demo
import rospy

import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf_conversions import transformations
from tf.transformations import euler_from_quaternion
from math import pi
from geometry_msgs.msg import PoseStamped

class navigation_demo:
    def __init__(self):
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server()

    def _done_cb(self, status, result):
        rospy.loginfo("navigation done! status:%d result:%s"%(status, result))

    def _active_cb(self):
        rospy.loginfo("[Navi] navigation has be actived")

    def _feedback_cb(self, feedback):
        rospy.loginfo("[Navi] navigation feedback\r\n%s"%feedback)

    def goto(self, p, frame):
        r = rospy.Rate(1)
        
        rospy.loginfo("[Navi] goto %s"%p)
        #r.sleep()
        goal = MoveBaseGoal()
        
        goal.target_pose.header.frame_id = p.header.frame_id
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = p.pose.position.x
        goal.target_pose.pose.position.y = p.pose.position.y
        # q = transformations.quaternion_from_euler(0.0, 0.0, (p[2]/180.0)*pi)
        #q = transformations.quaternion_from_euler(0.0, 0.0, p[2])
        goal.target_pose.pose.orientation.x = p.pose.orientation.x
        goal.target_pose.pose.orientation.y = p.pose.orientation.y
        goal.target_pose.pose.orientation.z = p.pose.orientation.z
        goal.target_pose.pose.orientation.w = p.pose.orientation.w

        # self.move_base.send_goal(goal, self._done_cb, self._active_cb, self._feedback_cb)
        self.move_base.send_goal(goal)
        result = self.move_base.wait_for_result()
        rospy.loginfo("wait_for_result")
        if not result:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
            #r.sleep()
        else:
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("reach goal %s succeeded!"%p)
                #r.sleep()

        return True

    def cancel(self):
        self.move_base.cancel_all_goals()
        return True
    

