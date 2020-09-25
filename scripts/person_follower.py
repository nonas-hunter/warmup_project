#!/usr/bin/env python3
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Pose, PoseWithCovariance, Twist, Vector3, Quaternion
import rospy
import math

class PersonFollower:
    def __init__(self):
        rospy.init_node('person_follower')
        rospy.Subscriber('/scan', LaserScan, self.analyze_scan) 
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        self.follow_object = {"cm":0, "dist":0}

        self.p0 = 180
        self.target_dist = 1
        self.CM_GAIN = 2
        self.APPROACH_GAIN = 1
        
    def analyze_scan(self, msg):
        scan = msg.ranges
        self.follow_object["dist"] = min(scan)
        min_index = scan.index(self.follow_object["dist"])
        self.follow_object["cm"] = self.find_cm(scan, min_index)
    
    def find_cm(self, scan, index_original):
        step_lower = 0
        step_upper = 0
        while scan[self.loop_around(index_original + step_lower)] != math.inf:
            step_lower -= 1 
        while scan[self.loop_around(index_original + step_upper)] != math.inf:
            step_upper += 1
        average = (step_lower + step_upper) / 2
        center_mass = self.loop_around(index_original + average)
        print("CM: "+ str(center_mass))
        return center_mass
       
    def loop_around(self, num):
        return abs(num % 360) 

################################################

    def run(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            error_angular = self.find_error_cm()
            error_linear = self.find_error_dist()
            angular_speed = self.controller_output_angular(error_angular)
            linear_speed = self.controller_output_linear(error_linear)
            self.publisher.publish(Twist(linear=Vector3(x=linear_speed), \
                angular=Vector3(z=angular_speed)))
            r.sleep()

    def find_error_cm(self):
        cm = self.follow_object["cm"]
        error = (self.loop_around((cm + 180))- 180) / 180
        print("Error: " + str(error))
        return error

    def find_error_dist(self):
        dist = self.follow_object["dist"]
        error = (dist - self.target_dist) / self.target_dist
        return error

    def controller_output_angular(self, error):
        output = error * self.CM_GAIN
        print("Controller Output: " + str(output))
        return output

    def controller_output_linear(self, error):
        output = error * self.APPROACH_GAIN
        print("Controller Output: " + str(output))
        return output

if __name__ == "__main__":
    person_follower = PersonFollower()
    person_follower.run()
    