#!/usr/bin/env python
from audioop import add
import imp
from unittest import result
import rospy
from std_msgs.msg import String
import message_filters
from behaviorpredict_layers.msg import my_msg5
import time

global last_data
last_data=""

global id_list
id_list = ""

global last_dict
last_dict = {}

global speed_dict
speed_dict = {}

global last_time
last_time = 0

def repro_sample(data):
    global last_data
    global id_list
    global last_dict
    global speed_dict
    global last_time
    now = time.time()
    speed_dict = {}
    if data.content=="person, nan, nan," or data.content=="person,0,1000," or data.content==last_data:
        return
    

    people = data.content.split(',')[:-1]
    id = id_list[1:-1].split('.')[:-1]
    dict = {}
    try:
        for i, _id in enumerate(id):
            dict[_id] = [people[i*3],people[i*3+1],people[i*3+2]]
    except IndexError:
        return

    # print(dict)
    for k, v in dict.items():
        # print(v)
        if k in last_dict.keys():
            point = last_dict[k]
            if v[1]==' nan' or v[2]==' nan' or point[1]==' nan' or point[2]==' nan':
                continue
            # print(point)
            try:
                v_x = (float(v[1])-float(point[1]))/(now-last_time)
                v_y = (float(v[2])-float(point[2]))/(now-last_time)
                if (v_x**2+v_y**2)**0.5>4:
                    v_x = 0
                    v_y = 0
                # print(v_x,"=========",v_y)
            except ValueError:
                continue
            speed_dict[k]=[v_x, v_y]
    
    last_dict = dict
    
    last_time = time.time()

    if not speed_dict:
        return
    
    result = ""
    for k, v in dict.items():
        if k in speed_dict.keys():
            result = result + "{},{},{},V,{},{},".format(v[0],v[1],v[2], speed_dict[k][0], speed_dict[k][1])
    
    print(result[:-1])
    print("\n")


def get_id(data):
    global id_list
    id_list = data.data

rospy.init_node('data_combine',anonymous=True)
rospy.Subscriber("/reprojection", my_msg5, repro_sample, queue_size=1)
rospy.Subscriber("/id_info", String, get_id, queue_size=1)
pub_point = rospy.Publisher('/behavior', String, queue_size=1)

r = rospy.Rate(10)
print("start")
while not rospy.is_shutdown():
    r.sleep()