#!/usr/bin/env python
"""
Ben Ziemann
Last updated: 9/9/18

Have the Neato follow a wall using angle measurements

May expand to have it follow based on assumed location

"""

import rospy
import math
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, Twist, Vector3


class WallFollower:

    def __init__(self):

        self.angScan = 45 #How many degrees angle from central location out to scan to compute direction
        self.angPerp = 0 #The laser scan angle perpendicular to the wall. Goal is to make and keep this either 90 or 270
        self.kP = .1 #Proportional control gain

        #Robot properities
        self.linVector = Vector3(x=0.0, y=0.0, z=0.0)
        self.angVector = Vector3(x=0.0, y=0.0, z=0.0)

        #ROS
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
        rospy.init_node('WallFollower')
        self.rate = rospy.Rate(1)
        rospy.Subscriber("/scan", LaserScan, self.checkLaser)


    def checkLaser(self, msg):
        """
        Determines laser scan angle perpendicular to the wall based on length of scans.
        """

        minDel = 99999
        minDelDeg = 0
        #Compare range of each angle to range self.angScan degrees ahead
        for deg in range(len(msg.ranges)-1):
            range1 = msg.ranges[deg]
            if deg+self.angScan >= 360:
                range2 = msg.ranges[deg+self.angScan-360]
            else:
                range2 = msg.ranges[deg+self.angScan]
            delta = range1 - range2
            

            #Determine angle perpendicular to the wall
            #Based on which two legs
            if math.fabs(delta) < minDel:
                minDel = math.fabs(delta) < minDel
                minDelDeg = deg

        if minDelDeg + (self.angPerp/2) >= 360:
            self.angPerp = minDelDeg + (self.angPerp/2) - 360
        else:
            self.angPerp = minDelDeg + (self.angPerp/2)


    def publish(self, linX, angZ):
        """
        Publish a given velocity
        """
        self.linVector.x = linX
        self.angVector.z = angZ
        self.pub.publish(Twist(linear=self.linVector, angular=self.angVector))


    def run(self):
        """
        Have the robot navigate based on the example's rules for behavior
        """

        #self.publish(0.2,0.0)
        while (not rospy.is_shutdown()):
            # #Proportional control for angular velocity based on how far perpendicular angle is from 90 or 270
            # #First quadrant - wall on left, robot turned too far towards wall
            # if (self.angPerp > 0) and (self.angPerp < 90):
            #     angZ = (self.angPerp - 90)*self.kP
            #     self.publish(0.2, angZ)

            # #Second quadrant - wall on left, robot turned too far away from wall
            # elif (self.angPerp > 90) and (self.angPerp < 180):
            #     angZ = (self.angPerp - 90)*self.kP
            #     self.publish(0.2, angZ)

            # #Third quadrant - wall on right, robot turned too far away from wall
            # elif (self.angPerp > 180) and (self.angPerp < 270):
            #     angZ = (self.angPerp - 270)*self.kP
            #     self.publish(0.2, angZ)

            # #Fourth quadrant - wall on right, robot turned too far towards wall
            # elif (self.angPerp > 270) and (self.angPerp < 359):
            #     angZ = (self.angPerp - 270)*self.kP
            #     self.publish(0.2, angZ)
            print(self.angPerp)
            self.rate.sleep()

        

if __name__ == '__main__':
    wf = WallFollower()
    try:
        wf.run()
    except rospy.ROSInterruptException:
        wf.publish(0.0, 0.0)