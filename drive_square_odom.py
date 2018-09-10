#!/usr/bin/env python
"""
Ben Ziemann
Last updated: 9/2/18

Moves the Neato counterclock wise around a regular polygon
Based on the odometry of the robot

Helped by NINJA Nathan for state structuring of the code
"""

import rospy
import math
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix
from geometry_msgs.msg import Pose, Twist, Vector3
from nav_msgs.msg import Odometry

class PolygonDriver():

	def __init__(self, sideNum, sideLength):

		self.state = {'forward': self.forward, 'leftTurn': self.leftTurn}

		#Robot Properties
		self.x = 0.0
		self.y = 0.0
		self.theta = 0.0
		self.linVector = Vector3(x=0.0, y=0.0, z=0.0)
		self.angVector = Vector3(x=0.0, y=0.0, z=0.0)
		self.previousCorner = (0.0, 0.0, 0.0)

		#Shape properties
		self.sideNum = sideNum
		self.sideLength = sideLength
		self.angle = 

		#Ros Setup
		self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		rospy.init_node('timeSquare')

		rospy.Subscriber("/odom", Odometry, self.setLocation)
        rospy.spin()

    def setLocation(self, odom):
        """ 
        Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple 

        Function written by professor Paul Ruvolo
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

	def forward(self):
		print("forward")
		self.linearVector.x = 0.5
		self.angularVector.z = 0.0


	def leftTurn(self):
		print("leftTurn")
		self.linearVector.x = 0.0
		self.angularVector.z = 0.5

	def stop(self):
		print("stop")
		self.linearVector.x = 0.0
		self.angularVector.z = 0.0


	def publishVelocity(self):
		print("publishVelocity")
		self.pub.publish(Twist(linear=self.linearVector, angular=self.angularVector))


	def driveForward(self):
		print("driving forward")
		
		nextCorner = calculateNextCorner()
		self.forward()
		self.publishVelocity()
		while(self.x <)
		self.stop()
		self.publishVelocity()

	def turn(self):
		print("turning")
		startTime = rospy.Time.now()
		self.leftTurn()
		self.publishVelocity()
		rospy.sleep(1)
		self.stop()
		self.publishVelocity()

	def run(self):
		for i in range(4):
			self.previousCorner = (self.x, self.y, self.theta)
			self.driveForward()
			self.turn()

		print("Done!")

if __name__ == '__main__':
	polygonDriver = PolygonDriver(4,1)


