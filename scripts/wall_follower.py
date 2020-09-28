#!/usr/bin/env python3
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Pose, PoseWithCovariance, Twist, Vector3, Quaternion
import rospy
import math

class WallFollower:
    def __init__(self):
        rospy.init_node('wall_follower')
        self.subscriber = rospy.Subscriber('/scan', LaserScan, self.analyze_scan) 
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.scan = []
        self.min_dist = 0
        self.min_index = 1
        self.p0 = 0
        self.P_GAIN = 0.404494382 # found value based on expected min & max error (0.011, 1)

        self.viz = rospy.Publisher('/sphere_marker', Marker, queue_size=10)
        self.marker = Marker()

        self.run()

    def analyze_scan(self, msg):
        self.scan = msg
        self.min_dist = min(self.scan.ranges)
        self.min_index = self.scan.ranges.index(self.min_dist)

        print("Minimum distance: " + str(self.min_dist))
        print("Index of minimum distance: " + str(self.min_index))
        print("Error: " + str(self.find_error()["mag"]))
        print("Controller output: " + str(self.controller_output()))
        

    def run(self):
        while not rospy.is_shutdown():
            if(-0.025 <= self.find_error()["mag"] <= 0.025):
                self.publisher.publish(Twist(angular=Vector3(z=0)))
                self.publisher.publish(Twist(linear=Vector3(x=0.3)))
            else:
                self.publisher.publish(Twist(angular=Vector3(z=self.controller_output())))

            self.update("odom", 1, 2)
            self.publisher.publish(self.marker)

    def find_error(self):
        # Identify which side of the NEATO the wall is on
        if(self.min_index - 180 < 0):
            self.p0 = 90
        elif(self.min_index - 180 > 0):
            self.p0 = 270
        else:
            self.p0 = 90
        
        # Return error
        if (self.min_index - self.p0 == 0):
            return {
                "mag" : abs(self.min_index-self.p0)/90,
                "sign" : 1
            }
        else:
            return {
                "mag" : abs(self.min_index-self.p0)/90,
                "sign" : (self.min_index-self.p0)/abs(self.min_index-self.p0)
            }
        
    def controller_output(self):
        error = self.find_error()
        if(error["mag"] < 0):
            return error["sign"] * (((error["mag"] - (1/90)) * self.P_GAIN) + 0.3)
        else:
            return error["sign"] * (((error["mag"] - (1/90)) * self.P_GAIN) + 0.3)

    def update(self, frame_id, x_coord, y_coord):
        self.marker.header.frame_id = frame_id
        self.marker.header.stamp = rospy.Time.now()
        self.marker.ns = "my_namespace"
        self.marker.id = 0
        self.marker.type = Marker.SPHERE
        self.marker.action = Marker.ADD
        self.marker.pose.position.x = x_coord
        self.marker.pose.position.y = y_coord
        self.marker.pose.position.z = 1
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0
        self.marker.scale.x = 1
        self.marker.scale.y = 1
        self.marker.scale.z = 1
        self.marker.color.a = 1.0 # Don't forget to set the alpha!
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0

if __name__ == "__main__":
    wall_follower = WallFollower()
    rospy.spin()