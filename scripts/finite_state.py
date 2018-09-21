#!/usr/bin/env python
"""
Ben Ziemann
Last updated: 9/20/18

Finite state controller to switch between wall following
and object avoidance based on whether there is something immediately
in front of the robot
"""

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, Twist, Vector3
from visualization_msgs.msg import Marker
import wall_follower
import object_avoid


class FiniteStateController:

    def __init__(self):

        self.linX = None
        self.angZ = None
        self.state = 0 #0 = stop, 1 = wall follow, 2 = avoid
        self.states = {0:"STOP" , 1:"Wall", 2:"Avoid"}

        self.avoidTwist = None
        self.wallTwist = None

        self.linVector = Vector3(x=0.0, y=0.0, z=0.0)
        self.angVector = Vector3(x=0.0, y=0.0, z=0.0)
        
        #ROS
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
        rospy.init_node('FiniteStateController')
        self.rate = rospy.Rate(2)
        rospy.Subscriber("/scan", LaserScan, self.checkLaser)
        rospy.Subscriber('/ObstacleAvoidance/cmd_vel', Twist, self.getAvoidTwist)
        rospy.Subscriber('/WallFollower/cmd_vel', Twist, self.getWallTwist)


    def getAvoidTwist(self, msg):
        """
        Get the velocity obstacle avoidance is putting out
        """
        
        self.avoidTwist = msg


    def getWallTwist(self, msg):
        """
        Get the velocity wall follower is putting out
        """
        print(msg)
        self.wallTwist = msg


    def checkLaser(self, msg):
        """
        Check laser scan data to see if there is an object needing avoiding
        """
        if (msg.ranges[0] != 0) and (msg.ranges[0] < 1):
            self.state = 1
        else:
            self.state = 2


    def publish(self, linX, angZ):
        """
        Publish a given velocity
        """
        self.linVector.x = linX
        self.angVector.z = angZ
        self.pub.publish(Twist(linear=self.linVector, angular=self.angVector))


    def run(self):
        """
        Publish according to state
        """
        while(self.wallTwist == None) and (self.avoidTwist == None) and (not rospy.is_shutdown()):
            continue

        while not rospy.is_shutdown():
            print("Current state is: %s" %states[self.state])
            if self.state == 0:
                self.publish(0,0)
            elif self.state == 1:
                self.pub.publish(self.avoidTwist)
            else:
                self.pub.publish(self.wallTwist)
            
            

        

if __name__ == '__main__':
    fs = FiniteStateController()
    fs.run()
 