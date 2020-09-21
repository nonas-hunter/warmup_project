#!/usr/bin/env python3
import tty
import select
import sys
import termios
import rospy
from geometry_msgs.msg import Twist, Vector3

class Teleop:
    def __init__(self):
        rospy.init_node('teleop')
        self.key = None
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.linear_velocity = 0.4
        self.angular_velocity = 0.6
        self.stop_velocity = 0.0
        self.run()

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, SETTINGS)
        return key

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown() and self.key != '\x03':
            self.key = self.getKey()
            print(self.key)
            if self.key == 'w':
                self.pub.publish(Twist(linear=Vector3(x= self.linear_velocity)))
            if self.key == 'a':
                self.pub.publish(Twist(angular=Vector3(z= self.angular_velocity)))
            if self.key == 's':
                self.pub.publish(Twist(linear=Vector3(x= -self.linear_velocity)))
            if self.key == 'd':
                self.pub.publish(Twist(angular=Vector3(z= -self.angular_velocity)))
            if self.key == 'e':
                self.pub.publish(Twist(linear=Vector3(x= self.stop_velocity), angular=Vector3(x=self.stop_velocity)))

            r.sleep()

if __name__ == "__main__":
    SETTINGS = termios.tcgetattr(sys.stdin)
    teleop = Teleop()

    
