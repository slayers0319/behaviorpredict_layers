#!/usr/bin/env python
import math
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point
from behaviorpredict_layers.msg import BoundingBox
from behaviorpredict_layers.msg import BoundingBoxes
from sensor_msgs.msg import LaserScan
import numpy as np


class Lidar_BBX_asyn:
    def __init__(self):
        # publisher setting
        self.pub_marker = rospy.Publisher("fusion_data", String, queue_size=1)
        self.pub_point = rospy.Publisher('/behavior', String, queue_size=1)
        # subscriber setting
        rospy.Subscriber('/darknet_ros/bounding_boxes1', BoundingBoxes, self.get_BBX, queue_size=1)
        rospy.Subscriber('/scan', LaserScan, self.get_scan, queue_size=1)

        # parameter setting
        self.WIDTH=640.0
        self.ANGLE_OF_VIEW=78.0
        self.last_bbx = BoundingBox()
        self.class_list = ["personR", "personL", "personF", "personB"]
        self.scan = LaserScan()

    def get_scan(self, data):
        self.scan = data
    
    def get_BBX(self, bbx):
        if self.last_bbx==bbx.bounding_boxes:
            return
        
        D_Range = 5
        degree_increment = int(round((D_Range*math.pi/180.0)/self.scan.angle_increment))# degree increment per laser
        output = ""
        result = ""
        min_d = 10000
        for b in bbx.bounding_boxes:
            if b.Class not in self.class_list:
                continue
            print("p",b.probability)
            if b.probability<0.8:
                print(b.probability)
                continue
            x_mid = (b.xmin+b.xmax)/2
            degree = int(round(self.ANGLE_OF_VIEW*((self.WIDTH-x_mid)/self.WIDTH)))
            degree = degree + ((39-degree)*9)/39

            N = -((39*math.pi/180.0)+self.scan.angle_min)/self.scan.angle_increment #start index
            N = (degree*math.pi/180.0)/self.scan.angle_increment + N
            N = int(round(N))

            scan_arr = np.array(self.scan.ranges[N-degree_increment:N+degree_increment])
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
                result = "{},{},{},V,{},0".format(b.Class,x,y,V)
                min_d = distance

        self.last_bbx = bbx.bounding_boxes

        if result=="" and output=="":
            return

        print("="*10)
        
        print(output[:-1])
        self.pub_marker.publish(output[:-1])

        if result=="":
            return
        print(result)
        self.pub_point.publish(result)

rospy.init_node('combine_BBX_Lidar',anonymous=True) #node initial
r = rospy.Rate(10)
r.sleep()
print("start")
Lidar_BBX = Lidar_BBX_asyn()
while not rospy.is_shutdown():
    r.sleep()
