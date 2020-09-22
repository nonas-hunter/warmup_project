#!/usr/bin/env python3
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Pose, PoseWithCovariance, Twist, Vector3, Quaternion
import rospy

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



if __name__ == "__main__":
    wall_follower = WallFollower()
    rospy.spin()