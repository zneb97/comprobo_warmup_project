#!/usr/bin/env python
"""
Ben Ziemann
Last updated: 9/9/18

Have the Neato follow a person

Extension:
Use comparisons to previous location to avoid confusin due to large objects
"""

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, Twist, Vector3
from visualization_msgs.msg import Marker


class PersonFollower:

    def __init__(self):

        #Location of the person. Goal is to get this to 1 meters and 0 degrees
        self.linLoc = None 
        self.angLoc = None

        #Previous location of the person. TODO use this not get distracted by objects
        self.pLinLoc = None
        self.pAngLoc = None

        #Angular and depth area to scan in front of the robot for object
        self.depthScan = 1.5 #meters
        self.angScan = 90 #degrees

        #Robot properities
        self.linVector = Vector3(x=0.0, y=0.0, z=0.0)
        self.angVector = Vector3(x=0.0, y=0.0, z=0.0)
        self.kPLin = 0.3
        self.kPAng = 0.75

        #ROS
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
        self.vizPub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        rospy.init_node('PersonFollower')
        self.rate = rospy.Rate(2)
        rospy.Subscriber("/scan", LaserScan, self.checkLaser)


    def checkLaser(self, msg):
        """
        Check laser scan to see if there is an appropriately sized object
        in the region of interest.

        msg - sensor_msgs/LaserScan type ROS message containing laser scan data
        """

        myScan = [] #Specialized scan range from self.angScan/2 to 0 to 360-angScan/2
        for i in range(360-self.angScan/2, 360):
            if msg.ranges[i] > self.depthScan:
                myScan.append(0.0)
            else:
                myScan.append(msg.ranges[i])


        for i in range(0, 1+self.angScan/2):
            if msg.ranges[i] > self.depthScan:
                myScan.append(0.0)
            else:
                myScan.append(msg.ranges[i])

        myScan.reverse()

        #Get averaged center of the points
        sumLin = 0.0
        sumAng = 0.0
        count = 0.0
        for i in range(len(myScan)):
            if myScan[i] == 0.0:
                continue
            else:
                sumLin += myScan[i]
                sumAng += i
                count += 1.0
        if count != 0.0:
            self.pLinLoc = self.linLoc
            self.pAngLoc = self.angLoc
            self.linLoc = sumLin/count
            self.angLoc = sumAng/count

            #Visualize person
            if (self.angLoc < self.angScan/2):
                laserAng = self.angScan/2 - self.angLoc
            else:
                laserAng = 360 - (self.angLoc -self.angScan/2)
            self.visualize(math.radians(laserAng), msg.ranges[int(laserAng)])


    def publish(self, linX, angZ):
        """
        Publish a given velocity
        """
        self.linVector.x = linX
        self.angVector.z = angZ
        self.pub.publish(Twist(linear=self.linVector, angular=self.angVector))


    def visualize(self, laserAng, laserDist):
        """
        Visualize the location of the person relative to the robot using rviz
        """

        #Set up marker
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "myMarker"
        marker.id = 0
        marker.type = 3 #Sphere
        marker.action = 0 #Add

        if laserAng < self.angScan:
            marker.pose.position.x = laserDist*math.cos(laserAng)
            marker.pose.position.y = laserDist*math.sin(laserAng)
        else:
            marker.pose.position.x = laserDist*math.cos(laserAng)
            marker.pose.position.y = -laserDist*math.sin(laserAng)

        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = .1
        marker.scale.y = .1
        marker.scale.z = 1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        self.vizPub.publish(marker)


    def normalizeNeg(self, val, rMin, rMax):
        """
        Normalize a value in a given range to between -1 and 1.

        val - the value to normalize
        rMin - lower limit of the range of possible input values
        rMin - upper limit of the range of possible input values
        """
        return (-2.0*(val-rMin) / (rMax-rMin))+1.0


    def normalize(self, val, rMin, rMax):
        """
        TODO: make a single normalize function

        Normalize a value in a given range to between 0 and 1.

        val - the value to normalize
        rMin - lower limit of the range of possible input values
        rMin - upper limit of the range of possible input values
        """
        return float(val-rMin) / (rMax-rMin)


    def run(self):
        """
        Turn to center the center mass and move to keep the center of mass
        a certain distance and keep it there.
        """
        while((self.linLoc == None) and not rospy.is_shutdown()):
            print("Waiting for data")

        while not rospy.is_shutdown():
            if self.linLoc < 1:
                linX = 0
            else:
                linX = self.normalize(self.linLoc, 0.25, self.depthScan)*self.kPLin
            angZ = self.normalizeNeg(self.angLoc, 0.0, self.angScan)*self.kPAng
            self.publish(linX, angZ)


        

if __name__ == '__main__':
    pf = PersonFollower()
    pf.run()
 