#!/user/bin/env python3

""" publisher node that writes a sphere to rviz every 10 seconds"""
from visualization_msgs.msg import Marker
import rospy

class SphereMarker:
    def __init__(self):
        rospy.init_node('make_sphere') # inittialize ourselves with roscore
        self.publisher = rospy.Publisher('/sphere_marker', Marker, queue_size=10)

        self.marker = Marker()
        
        self.run()

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.update("odom", 1, 2)
            self.publisher.publish(self.marker)
            rate.sleep()

    def update(self, frame_id, x_coord, y_coord):
        self.marker.header.frame_id = frame_id
        self.marker.header.stamp = rospy.Time.now()
        self.marker.ns = "my_namespace"
        self.marker.id = 0
        self.marker.type = Marker.SPHERE
        self.marker.action = Marker.ADD
        self.marker.pose.position.x = x_coord
        self.marker.pose.position.y = y_coord
        self.marker.pose.position.z = 1
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0
        self.marker.scale.x = 1
        self.marker.scale.y = 1
        self.marker.scale.z = 1
        self.marker.color.a = 1.0 # Don't forget to set the alpha!
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0


if __name__ == "__main__":
    sphere = SphereMarker()    



    