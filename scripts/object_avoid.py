#!/usr/bin/env python
"""
Ben Ziemann
Last updated: 9/9/18

Have the Neato move in a general direction avoiding obstacles

TODO:
Make better proportional control
"""

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, Twist, Vector3
from visualization_msgs.msg import Marker


class ObjectAvoid:

    def __init__(self):

        #Potential field properities
        self.spread = 1 #how far the potential field has influence
        self.radius = .25 #Radius of any given point
        self.goalDist = 1
        self.goalAng = 0

        #Robot properities
        self.linVector = Vector3(x=0.0, y=0.0, z=0.0)
        self.angVector = Vector3(x=0.0, y=0.0, z=0.0)
        self.kPLin = .5
        self.kPAng = 1


        #ROS
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
        rospy.init_node('ObstacleAvoidance')
        self.rate = rospy.Rate(2)
        rospy.Subscriber("/scan", LaserScan, self.checkLaser)


    def publish(self, linX, angZ):
        """
        Publish a given velocity
        """
        self.linVector.x = linX
        self.angVector.z = angZ
        self.pub.publish(Twist(linear=self.linVector, angular=self.angVector))


    def getDist(self, x, y):
        """
        Returns the distance between (0,0) (the robot) and a point
        """
        return math.sqrt( math.pow(x,2) + math.pow(y,2) )


    def getAngle(self, x, y):
        """
        Returns the angle from 0 to the given point in radians
        """
        return math.atan2(y,x)


    def getCartPos(self, angle, dist):
        """
        Returns the cartsian point with repspect to the robot
        """

        #Convert to radians, handle wrapping
        angle = math.radians(angle%360)
        x = dist*math.cos(angle)
        y = dist*math.sin(angle)
        return x, y


    def checkLaser(self, msg):
        """
        Identify all potential obstacles and sum their forces on the robot
        """
        netX = 0
        netY = 0
        b = .1

        for i in range(0, 91):
            if (msg.ranges[i] < 2) and (msg.ranges[i] != 0.0):
                pointX, pointY = self.getCartPos(i, msg.ranges[i])
                pointDist = self.getDist(pointX, pointY)
                pointAng = self.getAngle(pointX, pointY)

                #Based on the potential fields spaper
                #Avoids entering radius of object
                #Affect object repellant has is affected by distance to object
                weight = (self.spread + self.radius - pointDist)
                if weight > 0:
                    netX += -b*weight*math.cos(pointAng)
                    netY += -b*weight*math.sin(pointAng)

        for i in range(270, 360):
            if (msg.ranges[i] < 2) and (msg.ranges[i] != 0.0):
                pointX, pointY = self.getCartPos(i, msg.ranges[i])
                pointDist = self.getDist(pointX, pointY)
                pointAng = self.getAngle(pointX, pointY)

                #Based on the potential fields spaper
                #Avoids entering radius of object
                #Affect object repellant has is affected by distance to object
                weight = (self.spread + self.radius - pointDist)
                if weight > 0:
                    netX += -b*weight*math.cos(pointAng)
                    netY += -b*weight*math.sin(pointAng)

        #Recorrect course once object is passed
        for i in range(90, 270):
            if (msg.ranges[i] < 2) and (msg.ranges[i] != 0.0):
                pointX, pointY = self.getCartPos(i, msg.ranges[i])
                pointDist = self.getDist(pointX, pointY)
                pointAng = self.getAngle(pointX, pointY)

                #Based on the potential fields spaper
                #Avoids entering radius of object
                #Affect object repellant has is affected by distance to object
                weight = (self.spread + self.radius - pointDist)
                if weight > 0:
                    netX += b*weight*math.cos(pointAng)
                    netY += b*weight*math.sin(pointAng)
              
              

        netX += self.goalDist

        linX = self.getDist(netX, netY)*self.kPLin
        angZ = self.getAngle(netX, netY)*self.kPAng
        print(linX,angZ)
        self.publish(linX,angZ)
            

    def run(self):
        """
        Move towards target while avoiding obstacles
        """

        while not rospy.is_shutdown():
            rospy.spin()


if __name__ == '__main__':
    oa = ObjectAvoid()
    oa.run()
 