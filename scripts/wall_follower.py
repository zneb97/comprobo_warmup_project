#!/usr/bin/env python
"""
Ben Ziemann
Last updated: 9/9/18

Have the Neato follow a wall using angle measurements

Possible extensions:
Handle 90 degree corners (currently can only handle shallow corners)
Use line of best fit on laser scan data to estimate wall (Theil-sen, RANSAC)
"""

import rospy
import math
from std_msgs.msg import Int64
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, Twist, Vector3
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix



class WallFollower:

    def __init__(self):

        #How many degrees angle from central location out to scan to compute direction.
        #Too large and missing data points may cause issues
        #Too small and imprecision of the sensor may cause issues.
        self.scanSections = 45

        #The laser scan angle perpendicular to the wall. Goal is to make and keep this either 90 or 270
        #depending on which side the wall is on
        self.angPerp = 0 

        #Proportional control gain
        self.kP = .5 

        #Robot properities
        self.linVector = Vector3(x=0.0, y=0.0, z=0.0)
        self.angVector = Vector3(x=0.0, y=0.0, z=0.0)
        self.x = 0
        self.y = 0
        self.theta = 0

        #ROS
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
        self.vizPub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        rospy.init_node('WallFollower')
        self.rate = rospy.Rate(2)
        rospy.Subscriber("/scan", LaserScan, self.checkLaser)
        rospy.Subscriber("/odom", Odometry, self.setLocation)


    def setLocation(self, odom):
        """ 
        Convert pose (geometry_msgs.Pose) to a (x, y, theta) tuple 
        Constantly being called as it is the callback function for this node's subscription

        odom is Neato ROS' nav_msgs/Odom msg composed of pose and orientation submessages

        Used for visualization
        """
        pose = odom.pose.pose
        orientation_tuple = (pose.orientation.x,
                             pose.orientation.y,
                             pose.orientation.z,
                             pose.orientation.w)
        angles = euler_from_quaternion(orientation_tuple)
        self.x = pose.position.x
        self.y = pose.position.y
        self.theta = angles[2]

        return (pose.position.x, pose.position.y, angles[2])


    def checkLaser(self, msg):
        """
        Determines laser scan angle perpendicular to the wall by on finding
        two scan lengths self.scanSections degrees apart to create an isosceles
        triangle.

        msg - sensor_msgs/LaserScan type ROS message containing laser scan data
        """


        angDifferences = []
        #Compare range of each angle to range self.scanSections degrees ahead
        for deg in range(len(msg.ranges)-1):

            range1 = msg.ranges[deg]

            #Missing data point for that angle
            if range1 == 0.0:
                angDifferences.append(9999999)
                continue

            #Angle wrapping over 360 degrees
            if deg+self.scanSections >= 360:
                range2 = msg.ranges[deg+self.scanSections-360]
            else:
                range2 = msg.ranges[deg+self.scanSections]

            angDifferences.append(math.fabs(range1 - range2))

        #Find the halfway angle between the smallest diffence in length. Angle wrapping
        if angDifferences.index(min(angDifferences))+(self.scanSections/2) > 359:
            self.angPerp = angDifferences.index(min(angDifferences))+(self.scanSections/2)-360
        else:
            self.angPerp = angDifferences.index(min(angDifferences))+self.scanSections/2

        
        #Visualize the wall
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "myMarker"
        marker.id = 0
        marker.type = 8 #Points
        marker.action = 0 #Add
        marker.scale.x = .1
        marker.scale.y = .1
        marker.scale.z = .1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.points = []

        #How many degrees to scan on either side of perpendicular for visualizing wall
        scanAng = 25

        #Deal with angle wrapping
        if self.angPerp + scanAng > 359:
            for i in range(self.angPerp-scanAng, 359):
                marker.points.append(self.getPoint(i,msg.ranges[i]))
            for i in range(0, self.angPerp+scanAng-360):
                marker.points.append(self.getPoint(i,msg.ranges[i]))

        elif self.angPerp - scanAng < 0:
            for i in range(0,self.angPerp+scanAng):
                marker.points.append(self.getPoint(i,msg.ranges[i]))
            for i in range(360-(scanAng-self.angPerp), 359):
                marker.points.append(self.getPoint(i,msg.ranges[i]))

        else:
            for i in range(self.angPerp-scanAng,self.angPerp+scanAng):
                marker.points.append(self.getPoint(i,msg.ranges[i]))

        self.vizPub.publish(marker)


    def getPoint(self, angle, dist):
        referenceAng = self.theta + math.radians(angle) # angle of robot + angle of point
        pointX = self.x + math.cos(referenceAng) * dist
        pointY = self.y + math.sin(referenceAng) * dist
        return Vector3(x=pointX, y=pointY)


    def publish(self, linX, angZ):
        """
        Publish a given velocity
        """
        self.linVector.x = linX
        self.angVector.z = angZ
        self.pub.publish(Twist(linear=self.linVector, angular=self.angVector))


    def normalize(self, val, rMin, rMax):
        """
        Normalize a value in a given range to between 0 and 1.

        val - the value to normalize
        rMin - lower limit of the range of possible input values
        rMin - upper limit of the range of possible input values
        """
        return float((val-rMin))/(rMax-rMin)


    def run(self):
        """
        Depending on what quadrant the angle perpendicular to the wall the
        robot is currently at, make the appropriate turn based on proportional
        controlled angular velocity.
        """

        while not rospy.is_shutdown():
            #Proportional control for angular velocity based on how far perpendicular angle is from 90 or 270
            #First quadrant - wall on left, robot turned too far towards wall
            if (self.angPerp >= 0) and (self.angPerp < 90):
                angZ = -self.normalize(self.angPerp, 0, 90)*self.kP

            #Second quadrant - wall on left, robot turned too far away from wall
            elif (self.angPerp >= 90) and (self.angPerp < 180):
                angZ = self.normalize(self.angPerp, 90, 180)*self.kP

            #Third quadrant - wall on right, robot turned too far away from wall
            elif (self.angPerp >= 180) and (self.angPerp < 270):
                angZ = -self.normalize(self.angPerp, 180, 270)*self.kP

            #Fourth quadrant - wall on right, robot turned too far towards wall
            elif (self.angPerp >= 270) and (self.angPerp < 360):
                angZ = self.normalize(self.angPerp, 270, 360)*self.kP
                
            self.publish(0.1, angZ)
            self.rate.sleep()

        

if __name__ == '__main__':
    wf = WallFollower()
    wf.run()
 