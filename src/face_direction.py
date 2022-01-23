#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import message_filters
from behaviorpredict_layers.msg import my_msg5

global last_data
last_data=""

def repro_sample(data):
    global last_data
    if data.content=="person, nan, nan," or data.content=="person, 0, 1000," or data.content==last_data:
        return
    last_data = data.content
    people_list = data.content.split(",")
    nearest = 1000000
    people = []
    arg_nearest = -1
    # deal with people point
    for i in range((len(people_list)-1)/3):
        if people_list[i*3+1]==" nan" or people_list[i*3+2]==" nan":
            continue
        try:
            if float(people_list[1])==0 and float(people_list[2])==1000:
                return
            if float(people_list[i*3+1])**2+float(people_list[i*3+2])**2<nearest:
                nearest= float(people_list[i*3+1])**2+float(people_list[i*3+2])**2
                people = []
                people.append(float(people_list[i*3+1]))
                people.append(float(people_list[i*3+2]))
                arg_nearest = i
        except ValueError:
            print("inValid number")
            continue
    
    if len(people)==0: # poeple is empty
        return

    if people_list[i*3]=="personL":
        speed = -0.5
    else:
        speed = 0.5

    output = "{},{},{},V,{},{}".format(people_list[i*3],people[0],people[1],speed,0)
    print(output)
    # pub_point.publish(output)


rospy.init_node('data_combine',anonymous=True)
rospy.Subscriber("/reprojection", my_msg5, repro_sample, queue_size=1)

pub_point = rospy.Publisher('/behaviorpredict', String, queue_size=1)

r = rospy.Rate(10)
print("start")
while not rospy.is_shutdown():
    r.sleep()