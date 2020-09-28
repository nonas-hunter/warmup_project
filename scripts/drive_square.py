#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, PoseWithCovariance, Twist, Vector3, Quaternion
from tf.transformations import euler_from_quaternion
from tf.transformations import euler_matrix
import math


class DriveSquare:
    def __init__(self):
        rospy.init_node('square')
        self.subscriber = rospy.Subscriber('/odom', Odometry, self.odom_process) 
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.linear_velocity = 0.3
        self.angular_velocity = 0.3
        self.x = 0
        self.y = 0
        self.z = 0
        self.w = 0
        self.z_orientation = 0
        self.x_coord = 0
        self.y_coord = 0
        self.x_dist = 0
        self.y_dist = 0
        self.message = 0
        self.square_completed = False
        self.run()
    def odom_process(self, msg):
        self.message = msg
        self.x = msg.pose.pose.orientation.x
        self.y = msg.pose.pose.orientation.y
        self.z = msg.pose.pose.orientation.z
        self.w = msg.pose.pose.orientation.w
        angles_euler = euler_from_quaternion([self.x,self.y,self.z,self.w], 'sxyz')
        self.z_orientation = math.degrees(angles_euler[2])

        self.x_coord = msg.pose.pose.position.x
        self.y_coord = msg.pose.pose.position.y

        print("X position: " + str(self.x_coord))
        print("Y position: " + str(self.y_coord))
        print("Roll: " + str(self.z_orientation))




    def check_square(self):
        if(0.95<= self.x_coord <= 1.05 and self.x_dist == 0):
            self.publisher.publish(Twist(angular=Vector3(z=self.angular_velocity)))
            while(True):
                if(90 <= self.z_orientation <= 180):
                    self.x_dist = 1
                    break
            self.publisher.publish(Twist(angular=Vector3(z=0.0)))
        if(0.95<= self.y_coord <= 1.05 and self.y_dist == 0):
            self.publisher.publish(Twist(angular=Vector3(z=self.angular_velocity)))
            while(True):
                if(-180 <= self.z_orientation <= -90):
                    self.y_dist = 1
                    break
            self.publisher.publish(Twist(angular=Vector3(z=0.0)))
        if(-0.05 <= self.x_coord <= 0.02 and self.x_dist == 1):
            self.publisher.publish(Twist(angular=Vector3(z=self.angular_velocity)))
            while(True):
                if(-90 <= self.z_orientation <= 0):
                    self.x_dist = 2
                    break
            self.publisher.publish(Twist(angular=Vector3(z=0.0)))
        if(-0.05 <= self.y_coord <= 0.02 and self.y_dist == 1):
            self.publisher.publish(Twist(angular=Vector3(z=self.angular_velocity)))
            while(True):
                if(0<= self.z_orientation <= 90):
                    self.x_dist = 2
                    break
            self.publisher.publish(Twist(angular=Vector3(z=0.0)))
            self.square_completed = True

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if(self.square_completed == False):
                self.publisher.publish(Twist(linear=Vector3(x= self.linear_velocity)))
            self.check_square()

            r.sleep()

    
        
if __name__ == "__main__":
    drive_square = DriveSquare()
    rospy.spin()