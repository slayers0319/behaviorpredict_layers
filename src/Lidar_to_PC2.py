#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import LaserScan
from laser_geometry import LaserProjection

class Laser2PC():
    def __init__(self):
        self.laserProj = LaserProjection()
        self.pcPub = rospy.Publisher('/laserPointCloud', PointCloud2, queue_size = 1)
        self.laserSub = rospy.Subscriber('/scan', LaserScan, self.laserCallback)

    def laserCallback(self, data):
        cloud_out = self.laserProj.projectLaser(data)
        self.pcPub.publish(cloud_out)

if __name__=='__main__':
    rospy.init_node('laser2PointCloud')
    laser2pc = Laser2PC()
    rospy.spin()