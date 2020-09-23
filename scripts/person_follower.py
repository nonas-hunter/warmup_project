#!/usr/bin/env python3
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Pose, PoseWithCovariance, Twist, Vector3, Quaternion
import rospy
import math

class PersonFollower:
    def __init__(self):
        rospy.init_node('person_follower')
        self.subscriber = rospy.Subscriber('/scan', LaserScan, self.analyze_scan) 
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.scan = []
        self.p0 = 180
        self.target_dist = 1
        self.CM_GAIN = 0.4
        self.APPROACH_GAIN = 0.0816326531
        #rospy.sleep(0.5)
        self.run()

    def analyze_scan(self, msg):
        self.scan = msg
       
    def run(self):
        while not rospy.is_shutdown():
            if(-0.05  <= self.find_error_cm() <= 0.05):
                self.publisher.publish(Twist(angular=Vector3(z=self.controller_output_cm())))

    def find_error_cm(self):
        error = (self.find_cm() - self.p0)/self.p0
        print("Error: " + str(error))
        return error
        """
    def find_error_approach(self):
        if (self.find_error_cm() != 0):
            return None
        else:
            return (self.scan.ranges[0] - self.target_dist)/self.target_dist
        """
    def controller_output_cm(self):
        error = self.find_error_cm()
        if(error > 0):
            output = (error*self.CM_GAIN) + 0.3
        else:
            output = (error*self.CM_GAIN) - 0.3
        print("Controller output: " + str(output))
        return output

        """
    def controller_output_target(self):
        return (self.find_error_approach()*self.APPROACH_GAIN) + 0.3
        """
    def find_cm(self):
        min_cm = 0
        max_cm = 0
        cm = 0 
        for i in range(len(self.scan.ranges)-1):
            if(min_cm == 0 and self.scan .ranges[i] == math.inf):
                self.min_cm = i
            if(self.min_cm != 0 and self.scan.ranges[i+1] != math.inf):
                self.max_cm = i
            self.cm = (self.max_cm - self.min_cm)/2
        print("cm: " + str(self.cm))
        return self.cm

    

if __name__ == "__main__":
    person_follower = PersonFollower()
    rospy.spin()