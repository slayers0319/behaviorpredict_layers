#!/usr/bin/env python
from audioop import add
from copy import deepcopy
from dis import dis
import math
import string
import rospy
from std_msgs.msg import String
import message_filters
from geometry_msgs.msg import Point
from behaviorpredict_layers.msg import BoundingBox
from behaviorpredict_layers.msg import BoundingBoxes
from sensor_msgs.msg import LaserScan
import numpy as np

# estimate:true
degree_map={0:0, 1:1.19, 2:2.37, 3:3.56, 4:4.74, 5:5.93, 6:7.09, 7:8.27, 8:9.43, 9:10.59, 10:11.73, 11:12.86, 
            12:13.99, 13:15.1, 14:16.21, 15:17.3, 16:18.38, 17:19.44, 18:20.49, 19:21.53, 20:22.55, 21:23.56,
            22:24.55, 23:25.53, 24:26.48, 25:27.43, 26:28.36, 27:29.28, 28:30.17, 29:31.05, 30:31.92, 31:32.77, 
            32:33.6, 33:34.42, 34:35.22, 35:36.01, 36:36.78, 37:37.54, 38:38.27, 39:39}

WIDTH=640
ANGLE_OF_VIEW=78
global last_bbx
last_bbx = BoundingBox()
last_point = Point(0,0,0)
class_list = ["personR", "personL", "personF", "personB"]

# def get_BBX(data):
#     #print(data.bounding_boxes)
#     pass

def get_BBX(data):
    print("*************************")
    # print(data.bounding_boxes)
    for b in data.bounding_boxes:
        print(b.Class)
        print(b.xmin)
        print(b.ymin)
        print(b.xmax)
        print(b.ymax)
        print("---")
    
def get_scan(data):
    global WIDTH
    global last_bbx
    global degree_map

    print("="*10)
    # degree mapping
    degree = (ANGLE_OF_VIEW/2)-((550*2)*ANGLE_OF_VIEW)/(2.0*WIDTH)
    degree = round(degree)
    # print(degree_map[int(degree)])
    degree = degree_map[int(degree)] if degree>0 else -degree_map[int(-degree)]
    print(degree)
    N = int(degree/(data.angle_increment*180/math.pi))-int(data.angle_min/data.angle_increment)

    scan_arr = np.array(data.ranges[N-3:N+3])
    scan_arr = scan_arr[scan_arr!=np.inf]
    print(scan_arr)

def combine_data(bbx, scan):
    global WIDTH
    global last_bbx
    global degree_map
    global pub_marker
    global last_point
    global class_list
    
    if last_bbx==bbx.bounding_boxes:
        return
    
    output = ""
    result = ""
    min_d = 10000
    for b in bbx.bounding_boxes:
        if b.Class not in class_list:
            continue

        degree_increment = scan.angle_increment*180/math.pi # degree increment per laser
        # degree mapping
        degree = (ANGLE_OF_VIEW/2)-((b.xmin+b.xmax)*ANGLE_OF_VIEW)/(2.0*WIDTH)
        if abs(degree)>39:
            return

        degree = round(degree)
        
        degree = degree_map[int(degree)] if degree>0 else -degree_map[int((-1)*degree)]
    
        N = int(degree/(scan.angle_increment*180/math.pi))-int(scan.angle_min/scan.angle_increment)
    
        scan_arr = np.array(scan.ranges[N-int(round(8/degree_increment)):N+int(round(8/degree_increment))])
        scan_arr = scan_arr[scan_arr!=np.inf]

        if len(scan_arr)==0:
            print("inf")
            continue
    
        distance = scan_arr.min()
    
        x = -distance*math.sin(degree*math.pi/180)
        y = distance*math.cos(degree*math.pi/180)

        if b.Class=="personR":
            V = 0.5
        elif b.Class=="personL":
            V = -0.5
        else:
            V = 0

        output = output + "{},{},{},B,".format(b.Class,x,y)

        if distance < min_d:
            result = "P,{},{},V,{},0".format(x,y,V)
            min_d = distance

    last_bbx = bbx.bounding_boxes

    if result=="" and output=="":
        return

    print("="*10)
    
    print(output)
    pub_marker.publish(output[:-1])

    if result=="":
        return
    print(result)
    pub_point.publish(result)

    
    



rospy.init_node('combine_BBX_Lidar',anonymous=True)
# rospy.Subscriber('darknet_ros/bounding_boxes1', BoundingBoxes, get_BBX, queue_size=1)
# rospy.Subscriber('scan', LaserScan, get_scan, queue_size=1)

bbx = message_filters.Subscriber('darknet_ros/bounding_boxes1', BoundingBoxes, queue_size=1)
scan = message_filters.Subscriber('scan', LaserScan, queue_size=1)
ts = message_filters.ApproximateTimeSynchronizer([bbx, scan], 10, 0.1) # allow_headerless=True
ts.registerCallback(combine_data)

pub_marker = rospy.Publisher("fusion_data", String, queue_size=10)
pub_point = rospy.Publisher('/behavior', String, queue_size=1)


r = rospy.Rate(10)
print("start")
while not rospy.is_shutdown():
    r.sleep()
