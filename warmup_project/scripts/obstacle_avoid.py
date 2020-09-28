#!/usr/bin/env python3
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Pose, PoseWithCovariance, Twist, Vector3, Quaternion
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, euler_matrix
import sensor_msgs.point_cloud2 as pc2
from field_process import PotentialField
import rospy
import roslib
import math
import numpy
import tf
from laser_geometry import LaserProjection
class AvoidObstacle:
    def __init__(self):
        rospy.init_node('person_follower')
        rospy.Subscriber('/scan', LaserScan, self.analyze_scan) 
        rospy.Subscriber('/odom', Odometry, self.odom_process) 
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        self.potential_field = PotentialField()
        self.agent_global = {"x": 1, "y": 1, "orientation": 1}
        rospy.sleep(1)

    def analyze_scan(self, msg):
        scan = msg.ranges
        self.potential_field.clear()
        for i in range(len(scan)-1):
            if scan[i] != math.inf:
                dist = scan[i]
                angle = i 
                local_x, local_y = self.scan_to_point_local(dist, angle)
                # if 0.9 < scan[i] < 1.1:
                #     print("local_x: " + str(local_x))
                #     print("local_y: " + str(local_y))
                global_x, global_y = self.point_to_global(local_x, local_y)
                # if 0.9 < scan[i] < 1.1:
                    # print("orientation: " + str(self.agent_global["orientation"]))
                    # print("angle: " + str(angle))
                    # print("global x: " + str(global_x))
                    # print("global y: " + str(global_y))
                if i % 2 == 0:
                    self.potential_field.add_point(10, 10, True, 3, 3)
                self.potential_field.add_point(float(global_x), float(global_y))
        
        
    def odom_process(self, msg):
        self.agent_global["x"] = msg.pose.pose.position.x
        self.agent_global["y"] = msg.pose.pose.position.y
        quat_x = msg.pose.pose.orientation.x
        quat_y = msg.pose.pose.orientation.y
        quat_z = msg.pose.pose.orientation.z
        quat_w = msg.pose.pose.orientation.w
        
        angles_euler = euler_from_quaternion([quat_x, quat_y, quat_z, quat_w], 'sxyz')
        z_shifted = self.loop_around(math.degrees(2*math.pi + angles_euler[2]))
        self.agent_global["orientation"] = z_shifted
    
    def scan_to_point_local(self, dist, angle):
        x = dist * math.cos(math.radians(angle))
        y = dist * math.sin(math.radians(angle))

        return x, y
    
    def point_to_global(self, x, y):
        theta = math.radians(self.agent_global["orientation"])
        point = [x, y, 1]
        rot_matrix = [[math.cos(theta), -math.sin(theta), 0], \
                      [math.sin(theta), math.cos(theta), 0], \
                      [0, 0, 1]]
        trans_matrix = [[1, 0 ,self.agent_global["x"]], \
                        [0, 1, self.agent_global["y"]], \
                        [0, 0, 1]]
        temp1 = self.vector_multiply(rot_matrix, point)
        temp2 = self.vector_multiply(trans_matrix, temp1)
        return temp2[0], temp2[1]

    def vector_multiply(self, matrix, vector):
        output = [0,0,0]
        for i in range(len(matrix)):
            for j in range(len(matrix[i])):
                output[i] += matrix[i][j] * vector[j]
        return output
        
    def find_error_angular(self, p0):
        error = (self.loop_around(math.degrees(p0))-self.agent_global["orientation"]) / 180
        # print("Error: " + str(error))
        return error

    def controller_output_angular(self, error):
        # print("Controller Output: " + str(output))
        return error * 2

    def loop_around(self, num):
        return abs(num % 360) 

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            velocity, angle = self.potential_field.calc_output(self.agent_global["x"], self.agent_global["y"])
            error = self.find_error_angular(angle)
            if not (-0.1 < error < 0.1):
                angular_speed = self.controller_output_angular(error)
                self.publisher.publish(Twist(linear=Vector3(x=0), \
                angular=Vector3(z=angular_speed)))
                print("angle: " + str(math.degrees(angle)))
                print("error: " + str(error))
            else:
                linear_speed = velocity
                self.publisher.publish(Twist(linear=Vector3(x=linear_speed), \
                angular=Vector3(z=0)))
                print("speed: " + str(linear_speed))
            # print("v: " + str(velocity))
            # print("theta: " + str(angle))
            r.sleep()



if __name__ == "__main__":
    avoider=AvoidObstacle()
    avoider.run()
    