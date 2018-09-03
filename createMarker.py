#!/usr/bin/env python

"""
Ben Ziemann
Last updated: 9/2/18

Create a marker in rviz
"""

import rospy
from std_msgs.msg import String
from visualization_msgs.msg import Marker

def talker():
    pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
    rospy.init_node('MarkerCreator')

    rate = rospy.Rate(2)

    #Set up marker
    marker = Marker()
    marker.header.frame_id = "odom"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "myMarker"
    marker.id = 0
    marker.type = 2 #Sphere
    marker.action = 0 #Add
    marker.pose.position.x = 1
    marker.pose.position.y = 2
    marker.pose.position.z = 0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 1
    marker.scale.y = 1
    marker.scale.z = 1
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0

    while not rospy.is_shutdown():
        pub.publish(marker)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass