#!/usr/bin/env python

#from launch_demo import launch_demo
import rospy
from std_msgs.msg import String
import actionlib

def action(data):
    global pub_go
    global pub_stop
    if data.data=='action:go':
        print(data.data)
        pub_go.publish(data.data)
    elif data.data=='action:stop':
        print(data.data)
        pub_stop.publish(data.data)

def actionnnn(data):
    global pub_go
    global pub_stop
    global last
    if last==data.data:
        lsat=data.data
        return
    if data.data=='action:go':
        print(data.data)
        pub_go.publish(data.data)
    elif data.data=='action:stop':
        print(data.data)
        pub_stop.publish(data.data)
    last=data.data

rospy.init_node('stop_go',anonymous=True)
rospy.Subscriber('/back', String, action)
# rospy.Subscriber('/back', String, actionnnn)
pub_go = rospy.Publisher('action_go', String, queue_size=5)
pub_stop = rospy.Publisher('action_stop', String, queue_size=5)

last = ""

r = rospy.Rate(10)
print("start")
while not rospy.is_shutdown():
    r.sleep()
