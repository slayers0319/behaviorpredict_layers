#!/usr/bin/env python
import math
import rospy
from std_msgs.msg import String
import message_filters
from geometry_msgs.msg import Point
from behaviorpredict_layers.msg import BoundingBox
from behaviorpredict_layers.msg import BoundingBoxes
from sensor_msgs.msg import LaserScan
import numpy as np

# estimate:true
degree_map={78:78, 77:77.27, 76:76.54, 75:75.78, 74:75.01, 73:74.22, 72:73.42, 71:72.6,
            70:71.77, 69:70.92, 68:70.05, 67:69.17, 66:68.28, 65:67.36, 64:66.43, 63:65.48, 62:64.53, 61:63.55,
            60:62.56, 59:61.55, 58:60.53, 57:59.49, 56:58.44, 55:57.39, 54:56.3, 53:55.21, 52:54.1, 51:52.99,
            50:51.86, 49:50.73, 48:49.59, 47:48.43, 46:47.27, 45:46.09, 44:44.93, 43:43.74, 42:42.56, 41:41.37, 40:40.19,
            39:39,38:37.81, 37:36.63, 36:35.44, 35:34.26, 34:33.07, 33:31.91, 32:30.73, 31:29.57, 30:28.41, 29:27.27, 28:26.14, 
            27:25.01, 26:23.9, 25:22.79, 24:21.7, 23:20.62, 22:19.56, 21:18.51, 20:17.47, 19:16.45, 18:15.44,
            17:14.45, 16:13.47, 15:12.52, 14:11.57, 13:10.64, 12:9.72, 11:8.83, 10:7.95, 9:7.08, 8:6.23, 
            7:5.4, 6:4.58, 5:3.78, 4:2.99, 3:2.22, 2:1.46, 1:0.73, 0:0}

WIDTH=640.0
ANGLE_OF_VIEW=78.0
last_bbx = BoundingBox()
last_point = Point(0,0,0)
class_list = ["personR", "personL", "personF", "personB"]

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
    global ANGLE_OF_VIEW
    global degree_map

    print("="*10)
    # degree mapping
    degree = ANGLE_OF_VIEW*((WIDTH-320)/WIDTH)
    degree = round(degree_map[degree])
    N = -((39*math.pi/180.0)+data.angle_min)/data.angle_increment #start index
    N = (degree*math.pi/180.0)/data.angle_increment + N
    N = int(round(N))
    
    scan_arr = np.array(data.ranges[N-3:N+3])
    scan_arr = scan_arr[scan_arr!=np.inf]
    print(scan_arr)

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

        
        # degree mapping
        # degree = (ANGLE_OF_VIEW/2)-((b.xmin+b.xmax)*ANGLE_OF_VIEW)/(2.0*WIDTH)
        x_mid = (b.xmin+b.xmax)/2
        degree = int(round(ANGLE_OF_VIEW*((WIDTH-x_mid)/WIDTH)))
        # degree = degree_map[degree]
        # degree = degree -7
        # print(degree)
        degree = degree + ((39-degree)*9)/39
        # print(degree)
        # if degree<=35:
        #     print("######")
            
        #     degree = degree + 5

        N = -((39*math.pi/180.0)+scan.angle_min)/scan.angle_increment #start index
        N = (degree*math.pi/180.0)/scan.angle_increment + N
        N = int(round(N))

        scan_arr = np.array(scan.ranges[N-degree_increment:N+degree_increment])
        # scan_arr = np.array(scan.ranges[N:N+degree_increment])
        scan_arr = scan_arr[scan_arr!=np.inf]
        # print(scan_arr)
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
# rospy.Subscriber('/darknet_ros/bounding_boxes1', BoundingBoxes, get_BBX, queue_size=1)
# rospy.Subscriber('/scan', LaserScan, get_scan, queue_size=1)

pub_marker = rospy.Publisher("fusion_data", String, queue_size=1)
pub_point = rospy.Publisher('/behavior', String, queue_size=1)

bbx = message_filters.Subscriber('darknet_ros/bounding_boxes1', BoundingBoxes, queue_size=1)
scan = message_filters.Subscriber('scan', LaserScan, queue_size=1)
ts = message_filters.ApproximateTimeSynchronizer([bbx, scan], 1, 0.3) # allow_headerless=True
ts.registerCallback(combine_data)




r = rospy.Rate(10)
r.sleep()
print("start")
while not rospy.is_shutdown():
    r.sleep()
