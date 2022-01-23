#!/usr/bin/env python
import rospy
import math
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import String

font_size = 0.2
point_text_spacing = 0.2

class rviz_maker():
    def __init__(self):
        self._color = {'R':0, 'G':1, 'B':2} #color table
        self.pub = rospy.Publisher("people_marker", MarkerArray, queue_size=10)
        rospy.Subscriber("/marker", String, self.update_marker)
        self.markerarray = MarkerArray()

    def update_marker(self,data):
        #data format="person,1.2,0.87,R,obstacle,0.5,0.66,G,person,1.0,1.3,B"
        data_lsit = data.data.split(',')
        point_list = [[data_lsit[i*4],data_lsit[i*4+1],data_lsit[i*4+2],data_lsit[i*4+3]]for i in range(0,len(data_lsit)/4)]
        #marker array
        self.markerarray.markers = []
        
        for i,point in enumerate(point_list):
            #point setting
            marker_point = Marker()
            marker_point.header.frame_id = '/map'
            marker_point.header.stamp = rospy.get_rostime()
            marker_point.ns = 'points_texts'
            marker_point.id = i
            marker_point.type = Marker.CYLINDER
            marker_point.action = Marker.ADD
            marker_point.lifetime = rospy.Duration()
            marker_point.scale.x = 0.2
            marker_point.scale.y = 0.2
            marker_point.scale.z = 0.2
            marker_point.color.a = 1.0
            if point[3]=='R':
                marker_point.color.r = 1.0
            elif point[3]=='G':
                marker_point.color.g = 1.0
            elif point[3]=='B':
                marker_point.color.b = 1.0
            marker_point.pose.position.x = float(point[1])
            marker_point.pose.position.y = float(point[2])
            marker_point.pose.position.z = 0
            self.markerarray.markers.append(marker_point)

        marker_line = Marker()
        marker_line.header.frame_id = '/base_footprint'
        marker_line.header.stamp = rospy.get_rostime()
        marker_line.ns = 'points_texts'
        marker_line.id = i*2
        marker_line.type = Marker.LINE_STRIP
        marker_line.action = Marker.ADD
        marker_line.lifetime = rospy.Duration()
        marker_line.scale.x = 0.05
        marker_line.color.a = 1.0
        marker_line.color.g = 1.0
        marker_line.points.extend([Point(0.5,0.5/math.tan(math.pi/6),0),Point(0,0,0),Point(0.5,-0.5/math.tan(math.pi/6),0)])
        self.markerarray.markers.append(marker_line)

        self.pub.publish(self.markerarray)
        

def main():
    rospy.init_node("people_marker", anonymous=True)
    rviz_maker()
    rospy.spin()

if __name__=='__main__':
    main()
