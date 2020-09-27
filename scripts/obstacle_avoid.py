#!/usr/bin/env python3
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Pose, PoseWithCovariance, Twist, Vector3, Quaternion
from field_process import PotentialField
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, euler_matrix
import rospy
import math
import numpy

class AvoidObstacle:
    def __init__(self):
        rospy.init_node('person_follower')
        rospy.Subscriber('/scan', LaserScan, self.analyze_scan) 
        rospy.Subscriber('/odom', Odometry, self.odom_process) 
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        self.potential_field = PotentialField()
        self.agent_global = {"x": None, "y": None, "orientation": None}

        
    def analyze_scan(self, msg):
        scan = msg.ranges

        for point in scan

    def odom_process(self, msg):
        agent_global["x"] = msg.pose.pose.position.x
        agent_global["y"] = msg.pose.pose.position.y

        quat_x, quat_y, quat_z, quat_w = (scan.pose.pose.orientation.x, \
            scan.pose.pose.orientation.y, \
            scan.pose.pose.orientation.z, \
            scan.pose.pose.orientation.w)
        angles_euler = euler_from_quaternion([quat_x, quat_y, quat_z, quat_w], 'sxyz')
        agent_global["orientation"] = math.pi + math.degrees(angles_euler[2])

    def run(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            velocity, angle = self.potential_field.calc_output(0,0)
            r.sleep()
    def scan_to_point_local(self, dist, angle):
        if(angle > 180):
            angle = angle - 180
        x = dist * math.cos(math.radians(angle))
        y = dist * math.sin(math.radians(angle))
    def point_to_global(self, x, y):
        theta = self.agent_global["orientation"]
        point = numpy.array([x], [y], [1])
        rot_matrix = numpy.array([math.cos(theta), math.sin(theta), 0], \
                                 [-math.sin(theta), math.cos(theta), 0], \
                                 [0, 0, 1])
        trans_matrix = numpy.array([1, 0 ,self.agent_global["x"]], \
                                   [0, 1, self.agent_global["y"]], \
                                   [0, 0, 1])
        temp = numpy.multiply(rot_matrix, point)
        temp = numpy.multiply(trans_matrix, temp)
        return temp[0,1], temp[0,2], temp[0,3]



if __name__ == "__main__":
    avoider=AvoidObstacle()
    avoider.run()
    