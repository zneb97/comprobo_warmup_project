#!/usr/bin/env python

"""
Ben Ziemann
Last updated: 9/10/18

Visualize the wall the robot is following
Intended to run alongside wall_follower.py
"""

import rospy
from std_msgs.msg import Int64
from visualization_msgs.msg import Marker
from sensor_msgs.msg import PointCloud


class WallVisualizer:

    def __init__(self):

        self.angPerp = None
        self.laserRanges = []
        self.wallAngle = 30

        self.pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        rospy.init_node('wall_visualizer')
        self.rate = rospy.Rate(2)
        rospy.Subscriber("/projected_stable_scan", PointCloud, self.getLaser)
        rospy.Subscriber("/wall_viz", Int64, self.getAngle)
        

    def getLaser(self, msg):
        """
        Get the laser scan
        """

        self.laserRanges = msg.points


    def getAngle(self, msg):
        """
        Get the angle the robot is perpendicular to the wall
        Retrieved from wall_follower.py
        """
  
        self.angPerp = int(msg.data)


    def run(self):
        """
        Use the angle perpendicular to the wall to write the laser scans
        30 degrees out on either side, which should be the wall.
        """

        rospy.sleep(1)

        while not rospy.is_shutdown():
            # markers = []

            # #Angle wrapping fix
            # angleWrapperFlag = 0

            # if self.angPerp - self.wallAngle < 0:
            #     upper = self.angPerp + self.wallAngle
            #     lower = self.angPerp - self.wallAngle + 360
                
            #     for i in range(0, upper):
            #         marker = self.markerMaker(i, self.laserRanges[i])
            #         makrers.append(marker)
            #     for i in range(self.wallAngle - 359-self.angPerp, 359):
            #         marker = self.markerMaker(i, self.laserRanges[i])
            #         makrers.append(marker)

            # elif self.angPerp + self.wallAngle > 359:
            #     upper = self.angPerp + self.wallAngle - 360
            #     lower = self.angPerp - self.wallAngle

            #     for i in range(lower, 359):
            #         marker = self.markerMaker(i, self.laserRanges[i])
            #         makrers.append(marker)
            #     for i in range(0, self.wallAngle-359-self.angPerp):
            #         marker = self.markerMaker(i, self.laserRanges[i])
            #         makrers.append(marker)

            # else:
            #     upper = self.angPerp + self.wallAngle
            #     lower = self.angPerp - self.wallAngle

            #     for i in range(lower,upper+1):
            #         marker = self.markerMaker(i, self.laserRanges[i])
            #         makrers.append(marker)
            continue


            self.pub.publish(markers)


    def markerMaker(self, ang, rang):
        #Set up marker
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "myMarker"
        marker.id = 0
        marker.type = 1 #Cube
        marker.action = 0 #Add
        marker.pose.position.x = rang.x
        marker.pose.position.y = rang.y
        marker.pose.position.z = rang.z
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = .1
        marker.scale.y = .1
        marker.scale.z = .1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        return marker


if __name__ == '__main__':
    wv = WallVisualizer()
    wv.run()