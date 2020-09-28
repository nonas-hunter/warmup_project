#!/usr/bin/env python3
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Pose, PoseWithCovariance, Twist, Vector3, Quaternion
from field_process import PotentialField
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, euler_matrix
import sensor_msgs.point_cloud2 as pc2
import rospy
import roslib
import math
import numpy
import tf
from laser_geometry import LaserProjection

class Lidar():
    def __init__(self):
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.on_scan)
        self.laser_projector = LaserProjection()
        rospy.sleep(1)

    def on_scan(self, msg):
        for p in pc2.read_points(point_cloud, field_names = ("x", "y", "z"), skip_nans=True):
            print(" x : %f  y: %f  z: %f" %(p[0],p[1],p[2]))
    def run(self):
        
        while not rospy.is_shutdown():
            
            rospy.sleep(0.1)

if __name__ == "__main__":
    avoider=Lidar()
    avoider.run()
  
    