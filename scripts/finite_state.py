#!/usr/bin/env python
"""
Ben Ziemann
Last updated: 9/21/18

Finite state controller to switch between wall following
and n-gon driving. As the number of sides increases, the tangential
polygon will become more and more like a tangential circle to the wall
"""

import rospy
import math
from geometry_msgs.msg import Pose, Twist, Vector3
import wall_follower
import drive_ngon
import subprocess as sp
from nav_msgs.msg import Odometry


class FiniteStateController:

    def __init__(self):

        self.driveDistance = 2
        self.startX = None
        self.startY = None

        self.state = 1 #0 = stop, 1 = wall follow, 2 = avoid
        self.states = {0:"STOP" , 1:"Wall", 2:"Ngon"}

        self.linVector = Vector3(x=0.0, y=0.0, z=0.0)
        self.angVector = Vector3(x=0.0, y=0.0, z=0.0)
        self.x = None
        self.y = None
        self.theta = None

        
        #ROS
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
        rospy.init_node('FiniteStateController')
        self.rate = rospy.Rate(2)
        rospy.Subscriber("/odom", Odometry, self.setLocation)        


    def setLocation(self, odom):
        """ 
        Convert pose (geometry_msgs.Pose) to a (x, y, theta) tuple 
        Constantly being called as it is the callback function for this node's subscription

        odom is Neato ROS' nav_msgs/Odom msg composed of pose and orientation submessages
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



    def publish(self, linX, angZ):
        """
        Publish a given velocity
        """
        self.linVector.x = linX
        self.angVector.z = angZ
        self.pub.publish(Twist(linear=self.linVector, angular=self.angVector))


    def getDist(self, x1, y1, x2, y2):
        """
        Returns the distance between two points
        """
        return math.sqrt( math.pow(x1-x2,2) + math.pow(y1-y2,2) )


    def run(self):
        """
        Follow the wall for a given distance then create increasing circle like shapes
        """

        rospy.sleep(1)

        sideCounter = 3
        while not rospy.is_shutdown():
            print("Current state is: %s" %self.states[self.state])
            if self.state == 0:
                self.publish(0,0)

            #Wall follow for a given distance
            elif self.state == 1:
                extProc = sp.Popen(['python','wall_follower.py'])
                while(self.getDist(self.x,self.y,self.startX,self.startY) < self.driveDistance) and (not rospy.is_shutdown()):
                    continue
                sp.Popen.terminate(extProc)
                self.state = 2
                

            #Continue adding sides to better approximate a circle
            else:
                shaped = 1
                shaped = ShapeDriver(sideCounter, 0.5).run()
                while(shaped == 1) and (not rospy.is_shutdown()):
                    continue
                self.startX = self.x
                self.startY = self.y
                self.state = 1
        

if __name__ == '__main__':
    fs = FiniteStateController()
    fs.run()
 