#!/usr/bin/env python
import math
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point
from behaviorpredict_layers.msg import BoundingBox
from behaviorpredict_layers.msg import BoundingBoxes
from sensor_msgs.msg import LaserScan
import numpy as np

WIDTH=640.0
ANGLE_OF_VIEW=78.0
global last_bbx
last_bbx = BoundingBox()
last_point = Point(0,0,0)
class_list = ["personR", "personL", "personF", "personB"]
scan = LaserScan()

def get_scan(data):
    global scan
    scan = data
    
def get_BBX(bbx):
    global WIDTH
    global ANGLE_OF_VIEW
    global last_bbx
    global degree_map
    global pub_marker
    global last_point
    global class_list
    global scan
    
    if last_bbx==bbx.bounding_boxes:
        return
    
    D_Range = 5
    degree_increment = int(round((D_Range*math.pi/180.0)/scan.angle_increment))# degree increment per laser
    output = ""
    result = ""
    min_d = 10000
    for b in bbx.bounding_boxes:
        if b.Class not in class_list:
            continue

        x_mid = (b.xmin+b.xmax)/2
        degree = int(round(ANGLE_OF_VIEW*((WIDTH-x_mid)/WIDTH)))
        degree = degree + ((39-degree)*9)/39

        N = -((39*math.pi/180.0)+scan.angle_min)/scan.angle_increment #start index
        N = (degree*math.pi/180.0)/scan.angle_increment + N
        N = int(round(N))

        scan_arr = np.array(scan.ranges[N-degree_increment:N+degree_increment])
        scan_arr = scan_arr[scan_arr!=np.inf]
        if len(scan_arr)==0:
            print("inf")
            continue
        
        distance = scan_arr.min()
        degree = degree + 51
        x = distance*math.cos(degree*math.pi/180)
        y = distance*math.sin(degree*math.pi/180)

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
    
    print(output[:-1])
    pub_marker.publish(output[:-1])

    if result=="":
        return
    print(result)
    pub_point.publish(result)

def combine_data(bbx, scan):
    global WIDTH
    global ANGLE_OF_VIEW
    global last_bbx
    global degree_map
    global pub_marker
    global last_point
    global class_list
    
    if last_bbx==bbx.bounding_boxes:
        return
    
    D_Range = 5
    degree_increment = int(round((D_Range*math.pi/180.0)/scan.angle_increment))# degree increment per laser
    output = ""
    result = ""
    min_d = 10000
    for b in bbx.bounding_boxes:
        if b.Class not in class_list:
            continue

        x_mid = (b.xmin+b.xmax)/2
        degree = int(round(ANGLE_OF_VIEW*((WIDTH-x_mid)/WIDTH)))
        degree = degree + ((39-degree)*9)/39

        N = -((39*math.pi/180.0)+scan.angle_min)/scan.angle_increment #start index
        N = (degree*math.pi/180.0)/scan.angle_increment + N
        N = int(round(N))

        scan_arr = np.array(scan.ranges[N-degree_increment:N+degree_increment])
        scan_arr = scan_arr[scan_arr!=np.inf]
        if len(scan_arr)==0:
            print("inf")
            continue
        
        distance = scan_arr.min()
        degree = degree + 51
        x = distance*math.cos(degree*math.pi/180)
        y = distance*math.sin(degree*math.pi/180)

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
    
    print(output[:-1])
    pub_marker.publish(output[:-1])

    if result=="":
        return
    print(result)
    pub_point.publish(result)


rospy.init_node('combine_BBX_Lidar',anonymous=True)
pub_marker = rospy.Publisher("fusion_data", String, queue_size=1)
pub_point = rospy.Publisher('/behavior', String, queue_size=1)

rospy.Subscriber('/darknet_ros/bounding_boxes1', BoundingBoxes, get_BBX, queue_size=1)
rospy.Subscriber('/scan', LaserScan, get_scan, queue_size=1)

r = rospy.Rate(10)
r.sleep()
print("start")
while not rospy.is_shutdown():
    r.sleep()
