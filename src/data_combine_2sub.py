#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import message_filters
from behaviorpredict_layers.msg import my_msg5


def combine_data(data_arv, data_repro):
    print("=======================")
    if data_arv.data=="delete" or data_arv.data=="done" or data_arv.data=="nan":
        return
    if data_repro.content=="person, nan, nan," or data_repro.content=="person, 0, 1000,":
        return

    people_list = data_repro.content.split(",")
    nearest = 1000000
    people = []
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
        except ValueError:
            print("inValid number")
            continue
    
    if len(people)==0: # poeple is empty
        return
    print("people: ",people)

    speed_list = data_arv.data.split(",")[:-1]
    speed = []
    try:
        for i in speed_list:
            speed.append(float(i))
    except ValueError:
        print("inValid number")
    
    if len(speed)==0: # poeple is empty
        return
    
    # output = "P,{},{},V,{},{}".format(people[1],people[2],speed[1],speed[2])
    # print(output)
    # pub_point.publish(output)

global give_arv

def speed_sample(data):
    global give_arv
    give_arv = data.data

def repro_sample(data):
    global give_arv
    if give_arv=="delete" or give_arv=="done" or give_arv=="nan":
        return

    if data.content=="person, nan, nan," or data.content=="person, 0, 1000,":
        return

    people_list = data.content.split(",")
    nearest = 1000000
    people = []
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
        except ValueError:
            print("inValid number")
            continue
    
    if len(people)==0: # poeple is empty
        return
    # print("people: ",people)

    speed_list = give_arv.split(",")[2:-1]
    speed = []
    try:
        for i in speed_list:
            speed.append(float(i))
    except ValueError:
        print("inValid number")
    
    if len(speed)==0: # poeple is empty
        return

    # print(give_arv)
    # print(people)
    # print(people)
    # print(speed)
    output = "P,{},{},V,{},{}".format(people[0],people[1],speed[0],speed[1])
    print(output)
    pub_point.publish(output)


rospy.init_node('data_combine',anonymous=True)
rospy.Subscriber('/give_arv', String, speed_sample, queue_size=1)
rospy.Subscriber("/reprojection", my_msg5, repro_sample, queue_size=1)
# arv = message_filters.Subscriber("/give_arv", String,  queue_size=10)
# repro = message_filters.Subscriber("/reprojection", my_msg5,  queue_size=10)
# ts = message_filters.ApproximateTimeSynchronizer([arv, repro], 10, 0.1, allow_headerless=True)
# ts.registerCallback(combine_data)
# ts.registerCallback(test)
pub_point = rospy.Publisher('/behaviorpredict', String, queue_size=1)

r = rospy.Rate(10)
print("start")
while not rospy.is_shutdown():
    r.sleep()