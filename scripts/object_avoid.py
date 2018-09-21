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


class ObjectAvoid:

    def __init__(self):

        #Potential field properities
        self.spread = 2
        self.radius = .25

        #List of locations of obstacles
        self.xObs = []
        self.yObs = []

        #Force vector on robot
        self.fVecX = 0
        self.fVecY = 0
        self.fr = 0
        self.fAng = 0

        #Robot properities
        self.linVector = Vector3(x=0.0, y=0.0, z=0.0)
        self.angVector = Vector3(x=0.0, y=0.0, z=0.0)
        self.kPLin = 0.5
        self.kPAng = 0.6

        #ROS
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
        rospy.init_node('ObstacleAvoidance')
        self.rate = rospy.Rate(2)
        rospy.Subscriber("/scan", LaserScan, self.checkLaser)


    def checkLaser(self, msg):
        """
        Identify all potential obstacles and transform polar to cartesian
        coordians for easier summation vector work
        """

        #Collect range values then average. The closer to something the slower it goes linearly
        closeness = []
        #Sum forces of angles of objects around it


        for i in range(0, 45):
            if (msg.ranges[i] != 0.0) and (msg.ranges[i] < 2.0):
                closeness.append(msg.ranges[i])

                # self.xObs.append(math.cos(math.radians(i))*msg.ranges[i])
                # self.yObs.append(math.sin(math.radians(i))*msg.ranges[i])

        for i in range(315, 360):
            if (msg.ranges[i] != 0.0) and (msg.ranges[i] < 2.0):
                closeness.append(msg.ranges[i])

                # self.xObs.append(math.cos(math.radians(i))*msg.ranges[i])
                # self.yObs.append(math.sin(math.radians(i))*msg.ranges[i])
    
        # for i in range(len(self.xObs)):
        #     print(self.xObs[i], self.yObs[i])
        # print("----------------------------------------------------")
        self.vectorize()
        

    def publish(self, linX, angZ):
        """
        Publish a given velocity
        """
        self.linVector.x = linX
        self.angVector.z = angZ
        self.pub.publish(Twist(linear=self.linVector, angular=self.angVector))


    def vectorize(self):
        """
        Turn identified points into directed forces and sum
        Spread and effectiveness based on linked paper in project page
        https://phoenix.goucher.edu/~jillz/cs325_robotics/goodrich_potential_fields.pdf
        """
        self.fVecX = 0
        self.fVecY = 0

        #Sum forces in cartestian
        for i in range(len(self.yObs)):
            dist = math.sqrt( math.pow(self.yObs[i],2) + math.pow(self.xObs[i],2) )
            ang = math.atan2(self.yObs[i], self.xObs[i])

            #Create the object's repellent force
            self.fVecX -= self.radius*math.cos(ang)
            self.fVecY -= self.radius*math.sin(ang)

        print(self.fVecX, self.fVecY)
        #Convert to polar for motion
        self.fr = math.sqrt(math.pow(self.fVecY,2) + math.pow(self.fVecX,2))
        self.fAng = math.degrees(math.atan2(self.fVecY, self.fVecX))
        
        # print(self.fr, self.fAng)



    def run(self):
        """
        Move towards target while avoiding obstacles
        """
        rospy.sleep(1)

        while not rospy.is_shutdown():
            # linX = self.fr*self.kPLin
            # angZ = self.fAng*self.kPAng
            # self.publish(linX, angZ)
            continue


if __name__ == '__main__':
    oa = ObjectAvoid()
    oa.run()
 