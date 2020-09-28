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
class AvoidObstacle:
    def __init__(self):
        rospy.init_node('person_follower')
        # rospy.Subscriber('/scan', LaserScan, self.analyze_scan) 
        # rospy.Subscriber('/projected_stable_scan', LaserScan, self.stable_process)
        # rospy.Subscriber('/odom', Odometry, self.odom_process) 
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.on_scan)
        self.laser_projector = LaserProjection()
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.listener = tf.TransformListener()
        
        self.potential_field = PotentialField()
        self.agent_global = {"x": 1, "y": 1, "orientation": 1}
        rospy.sleep(1)

    def on_scan(self, scan):
        rospy.loginfo("Got scan, projecting")
        cloud = self.laser_projector.projectLaser(scan)
        gen = pc2.read_points(cloud, skip_nans=True, field_names=("x", "y", "z"))
        self.xyz_generator = gen
        print(self.xyz_generator)
    def analyze_scan(self, msg):
        scan = msg.ranges
        self.potential_field.clear()
        for i in range(len(scan)-1):
            if scan[i] != math.inf:
                dist = scan[i]
                angle = i 
                local_x, local_y = self.scan_to_point_local(dist, angle)
                # print("local_x: " + str(local_x))
                # print("local_y: " + str(local_y))
                global_x, global_y = self.point_to_global(local_x, local_y)
                # if 0.9 < scan[i] < 1.1:
                    # print("global x: " + str(global_x))
                    # print("global y: " + str(global_y))
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
        if(angle > 180):
            angle = angle - 180
        x = dist * math.cos(math.radians(angle))
        y = dist * math.sin(math.radians(angle))

        return x, y
    
    def point_to_global(self, x, y):
        theta = math.radians(self.agent_global["orientation"])
        point = [x, y, 1]
        rot_matrix = [[math.cos(theta), math.sin(theta), 0], \
                      [-math.sin(theta), math.cos(theta), 0], \
                      [0, 0, 1]]
        trans_matrix = [[1, 0 ,self.agent_global["x"]], \
                        [0, 1, self.agent_global["y"]], \
                        [0, 0, 1]]
        temp1 = self.vector_multiply(rot_matrix, point)
        temp2 = self.vector_multiply(trans_matrix, temp1)
        return temp2[0], temp2[1]

        # Numpy implementation
        # theta = math.radians(self.agent_global["orientation"])
        # point = numpy.array([x], [y], [1]])
        # rot_matrix = numpy.array([[math.cos(theta), -math.sin(theta), 0], \
        #                          [math.sin(theta), math.cos(theta), 0], \
        #                          [0, 0, 1]])
        # trans_matrix = numpy.array([[1, 0 ,self.agent_global["x"]], \
        #                            [0, 1, self.agent_global["y"]], \
        #                            [0, 0, 1]])
        # temp1 = numpy.dot(rot_matrix, point)
        # temp2 = numpy.dot(trans_matrix, temp1)
        # return temp2[0,0], temp2[1,0]

    def vector_multiply(self, matrix, vector):
        output = [0,0,0]
        for i in range(len(matrix)):
            for j in range(len(matrix[i])):
                output[i] += matrix[i][j] * vector[j]
        return output
        
    def find_error_angular(self, p0):
        error = (self.agent_global["orientation"] - math.degrees(p0)) / 360
        # print("Error: " + str(error))
        return error

    def controller_output_angular(self, error):
        output = error
        # print("Controller Output: " + str(output))
        return output

    def loop_around(self, num):
        return abs(num % 360) 

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            velocity, angle = self.potential_field.calc_output(0, 0)
            linear_speed = velocity 
            error = self.find_error_angular(angle)
            angular_speed = self.controller_output_angular(error) * 2
            # self.publisher.publish(Twist(linear=Vector3(x=0), \
            #    angular=Vector3(z=0)))
            # print("v: " + str(velocity))
            # print("theta: " + str(angle))
            
            try:
                (trans,rot) = self.listener.lookupTransform('/LaserScan', '/base_link', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print("error")
                continue
            print("trans: " + str(trans))
            print("rot: " + str(rot))
            
  
            r.sleep()



if __name__ == "__main__":
    avoider=AvoidObstacle()
    avoider.run()
    