#!/user/bin/env python3

""" publisher node that writes a sphere to rviz every 10 seconds"""
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped

from std_msgs.msg import Header
from geometry_msgs.msg import Point
import rospy

class SphereMarker:
    def __init__(self, name):
        rospy.init_node(name) # inittialize ourselves with roscore
        
        self.header = Header(stamp=rospy.Time.now(), frame_id = "odom")

        # Replace with visualization_msgs/Marker
        self.point = Point(1.0, 2.0, 0.0)
        self.point_stamped = PointStamped(header = self.header, point = self.point)
        
        self.publisher = rospy.Publisher('/sphere_marker', PointStamped, queue_size=10)

        self.run()

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.point_stamped.header.stamp = rospy.Time.now() # update the time stamp
            self.publisher.publish(self.point_stamped)
            rate.sleep()

if __name__ == "__main__":
    sphere = SphereMarker("sphere1")    



    