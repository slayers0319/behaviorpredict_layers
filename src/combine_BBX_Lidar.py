#!/usr/bin/env python
from audioop import add
from copy import deepcopy
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
# def get_BBX(data):
#     #print(data.bounding_boxes)
#     pass

def get_scan(data):
    # N=int(((180.0-90)/360.0)*len(data.ranges)) # N=0 degree
    # print(data.angle_min+data.angle_increment*286)
    # N = int(0/(data.angle_increment*180/math.pi))-int(data.angle_min/data.angle_increment)
    # N = -int(data.angle_min/data.angle_increment)
    # print(N)
    # print(data.ranges[N-5:N+5])
    # scan_arr = np.array(data.ranges[N-5:N+5])
    # scan_arr = scan_arr[scan_arr!=np.inf]
    # print(scan_arr)
    # print(data.angle_increment*180/math.pi)
    # print(len(data.ranges))
    # print(int(round((180.0/360)*len(data.ranges))))
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

    if last_bbx==bbx.bounding_boxes:
        return
    print("="*10)
    degree_increment = scan.angle_increment*180/math.pi # degree increment per laser
    # print(round(10/degree_increment))
    # degree mapping
    # print(bbx.bounding_boxes[0].xmin+bbx.bounding_boxes[0].xmax)/2
    degree = (ANGLE_OF_VIEW/2)-((bbx.bounding_boxes[0].xmin+bbx.bounding_boxes[0].xmax)*ANGLE_OF_VIEW)/(2.0*WIDTH)
    if abs(degree)>39:
        return
    # print(degree)
    degree = round(degree)
    # print(degree)
    # print(degree_map[int(degree)])
    degree = degree_map[int(degree)] if degree>0 else -degree_map[int((-1)*degree)]
    # print("deg =",degree)
    N = int(degree/(scan.angle_increment*180/math.pi))-int(scan.angle_min/scan.angle_increment)
    # print("degree N =",scan.ranges[N])
    scan_arr = np.array(scan.ranges[N-int(round(8/degree_increment)):N+int(round(8/degree_increment))])
    scan_arr = scan_arr[scan_arr!=np.inf]
    if len(scan_arr)==0:
        return
    # print(scan_arr)
    print("mid =",scan_arr.min())
    mid = scan_arr.min()
    # print(degree)
    x = -mid*math.sin(degree*math.pi/180)
    y = mid*math.cos(degree*math.pi/180)
    # print(x,y)

    if last_point.x==0 and last_point.y==0:
        last_point = Point(x,y,0)
    
    direct = x - last_point.x
    if direct<0: # R to L
        V = -0.5
    else: # L to R
        V = 0.5

    output = "P,{},{},B".format(x,y)
    # print(output)
    pub_marker.publish(output)
    result = "P,{},{},V,{},0".format(x,y,V)
    print(result)
    pub_point.publish(result)
    # N=int(((180.0+degree)/360.0)*len(scan.ranges)) # N=0 degree
    # print(N)
    # scan_arr = np.array(scan.ranges[N-15:N+15]) #222<N~N+25<334
    # print(scan_arr)
    # scan_arr = scan_arr[scan_arr!=np.inf]
    ''''
    # print(scan_arr)
    # min_scan = scan_arr[np.argpartition(scan_arr,int(len(scan_arr)/2))[:int(len(scan_arr)/2)]]
    # print(min_scan)
    # print(scan_arr.min())
    r = scan_arr.min()
    # d = (((scan_arr.argmin()+N)*360)/len(scan.ranges))-180
    d = (scan_arr.argmin()*360/len(scan.ranges))-180
    # x_ = r*math.sin(d*math.pi/180)
    x = r*math.sin(degree*math.pi/180)
    y = r*math.cos(degree*math.pi/180)
    print(x*0.8)
    print(y)
    # print(x_)
    # print((((scan_arr.argmin()+N)*360)/len(scan.ranges))-180)
    '''
    last_point = Point(x,y,0)
    last_bbx = bbx.bounding_boxes



rospy.init_node('combine_BBX_Lidar',anonymous=True)
# rospy.Subscriber('darknet_ros/bounding_boxes', BoundingBoxes, get_BBX, queue_size=1)
# rospy.Subscriber('scan', LaserScan, get_scan, queue_size=1)
bbx = message_filters.Subscriber('darknet_ros/bounding_boxes', BoundingBoxes, queue_size=1)
scan = message_filters.Subscriber('scan', LaserScan, queue_size=1)
ts = message_filters.ApproximateTimeSynchronizer([bbx, scan], 10, 0.1) # allow_headerless=True
ts.registerCallback(combine_data)
pub_marker = rospy.Publisher("fusion_data", String, queue_size=10)
pub_point = rospy.Publisher('/behavior', String, queue_size=1)


r = rospy.Rate(10)
print("start")
while not rospy.is_shutdown():
    r.sleep()