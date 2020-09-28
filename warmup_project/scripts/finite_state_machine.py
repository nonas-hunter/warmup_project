#!/usr/bin/env python3

""" This code implements the finite state controller described in https://comprobo20.github.io/in-class/day05.
    In this version we will use the ROS smach library
    There are three states: moving forward, moving backward, and turning left.  The initial state is moving_forward.
    The rules for transitioning between these states are as follows.

    moving_forward:
        - if a bump sensor is activated transition to moving backward
    moving_backward:
        - once the obstacle in front of the robot is greater than a threshold, transition to turning left
    turning_left:
        - once 1 second has elapsed, transition to moving_forward
"""

import rospy
import smach
# try using smach viewer as your own risk
# import smach_ros
from geometry_msgs.msg import Twist, Point, Pose, PoseWithCovariance, Twist, Vector3, Quaternion
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Int8MultiArray
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math

class FSM_SQUARE_FOLLOW():
    """ This class encompasses the finite state controller ROS Node"""
    def __init__(self):
        rospy.init_node('square_follow')
        # the angular velocity when in the "turning_left" state
        # self.angular_velocity = 0.5
        # the forward speed when in the "moving_forward" state.  The speed is reversed in the "moving_backward" state.
        # self.forward_speed = 0.1
        # the distance threshold to use to transition from "moving_backward" to turning left
        # self.distance_threshold = 0.8

        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

    def run(self):
        # needed in the case where you send the velocity commands once (in some ways sending the commands repeatedly is
        # more robust.
        rospy.sleep(1)

        # Create a SMACH state machine
        sm = smach.StateMachine(outcomes=[])

        # Open the container
        with sm:
            # Add states to the container
            smach.StateMachine.add('square_driver', SQUARE_DRIVER(),
                                   transitions={'detected object': 'person_follow'})
            smach.StateMachine.add('person_follow', PERSON_FOLLOW(),
                                   transitions={'follow object lost': 'square_driver'})

        # Create and start the introspection server
        # if you want to try smach viewer, consider uncommenting these lines
        # sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
        # sis.start()
        # Execute SMACH plan
        outcome = sm.execute()

class SQUARE_DRIVER(smach.State):
    """ Implements the square driver state. """

    def __init__(self):
        self.subscriber = rospy.Subscriber('/odom', Odometry, self.odom_process) 
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.laser_scan = rospy.Subscriber('/scan', LaserScan, self.analyze_scan)
        smach.State.__init__(self, outcomes=['detected object'])

        self.scan_data = []
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
    def analyze_scan(self, msg):
        self.scan_data = msg.ranges

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

        # print("X position: " + str(self.x_coord))
        # print("Y position: " + str(self.y_coord))
        # print("Roll: " + str(self.z_orientation))

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

    def no_object(self, data):
        count = 0
        for i in range(len(data) - 1):
            if(data[i] == math.inf):
                count += 1
                if(count == 360):
                    return True
        print("no object")
        return False

    def execute(self, user_data):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if(self.square_completed == False):
                self.publisher.publish(Twist(linear=Vector3(x= self.linear_velocity)))
            self.check_square()
            
            if(self.no_object(self.scan_data) == False):
                # print("detected object")
                return 'detected object'

            r.sleep()


class PERSON_FOLLOW(smach.State):
    """ Implements the moving backward state. """

    def __init__(self):
        rospy.Subscriber('/scan', LaserScan, self.analyze_scan) 
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        self.follow_object = {"cm":0, "dist":0}
        self.scan_data = []
        self.p0 = 180
        self.target_dist = 1.5
        self.CM_GAIN = 2
        self.APPROACH_GAIN = 1

        smach.State.__init__(self, outcomes=['follow object lost'])

    def analyze_scan(self, msg):
        self.scan_data = msg.ranges
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
        # print("CM: "+ str(center_mass))
        return center_mass
       
    def loop_around(self, num):
        return abs(num % 360) 

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
        # print("Controller Output: " + str(output))
        return output

    def no_object(self, data):
        count = 0
        for i in range(len(data) - 1):
            if(data[i] == math.inf):
                count += 1
                if(count == 360):
                    return True
        print("no object")
        return False

    def execute(self, user_data):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            error_angular = self.find_error_cm()
            error_linear = self.find_error_dist()
            angular_speed = self.controller_output_angular(error_angular)
            linear_speed = self.controller_output_linear(error_linear)
            self.publisher.publish(Twist(linear=Vector3(x=linear_speed), \
                angular=Vector3(z=angular_speed)))
            
            if(self.no_object(self.scan_data) == True):
                # print("follow object lost")
                return 'follow object lost' 
            r.sleep()


if __name__ == '__main__':
    node = FSM_SQUARE_FOLLOW()
    node.run()