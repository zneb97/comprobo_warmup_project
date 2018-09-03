#!/usr/bin/env python
"""
Ben Ziemann
Last updated: 9/2/18

Moves the Neato counterclock wise around a regular polygon
Based on the odometry of the robot
"""

import rospy
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix
from geometry_msgs.msg import Pose

initFlag = 0 #Flag for determining the start point when the script is called
cornerNumber = 1 #Index of next corner
corners = [] #(x,y, yaw) corners of the square to drive
sides = 4 #How many sides
sideLength = 1 #Side length of the regular polygon (meters)
extAng = 180-(((sides-2)*180)/sides)
tolerance = .01 #How close to the target side length before turning (percent)


def convertPose(pose):
    """ 
    Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple 

    Function written by professor Paul Ruvolo
    """
    orientation_tuple = (pose.orientation.x,
                         pose.orientation.y,
                         pose.orientation.z,
                         pose.orientation.w)
    angles = euler_from_quaternion(orientation_tuple)
    return (pose.position.x, pose.position.y, angles[2])

def calcCorners(startPoint):
    """
    Calculate the corners (turning points) for the
    regular polygon to trace
    """

    #First is the script's start point
    corners.append(startPoint)
    #Second corner is side length straight out from first
    corners.append((startPoint[0]+sideLength,startPoint[1],sideLength[2]+extAng))

    #Generate remaining corners
    for i in range(2,sides):
        if extAng == 0:
            corners.append(corners[i-1][0]+sideLength, corners[i-1][1], corners[i-1][2]+extAng)
        elif extAng > 0 and extAng < 90:
            corners.append((corners[i-1][0]+(sideLength*math.sin(corners[i-1][2])), corners[i-1][1]+(sideLength*math.cos(corners[i-1][2])), corners[i-1][2]+extAng))
        elif extAng == 90:
            corners.append(corners[i-1][0], corners[i-1][1]+sideLength, corners[i-1][2]+extAng)
        elif extAng > 90 and extAng < 180:
            corners.append((corners[i-1][0]-(sideLength*math.cos(corners[i-1][2])), corners[i-1][1]+(sideLength*math.sin(corners[i-1][2])), corners[i-1][2]+extAng))
        elif extAng == 180:
            corners.append(corners[i-1][0]-sideLength, corners[i-1][1], corners[i-1][2]+extAng)
        elif extAng > 180 and extAng < 270:
            corners.append((corners[i-1][0]-(sideLength*math.sin(corners[i-1][2])), corners[i-1][1]-(sideLength*math.cos(corners[i-1][2])), corners[i-1][2]+extAng))
        if extAng == 270:
            corners.append(corners[i-1][0], corners[i-1][1]-sideLength, corners[i-1][2]+extAng)
        if extAng > 270 and extAng < 360:
            corners.append((corners[i-1][0]+(sideLength*math.cos(corners[i-1][2])), corners[i-1][1]-(sideLength*math.sin(corners[i-1][2])), corners[i-1][2]+extAng))

def writeMotors():
    #Make a publisher
    #send info to motor controller

def drive(data):
    """
    Compare current position to corner locations to determine when to turn
    First pass through calculates the corners
    """

    rospy.loginfo("Log data %s",data.data)
    #First pass through, will take marginally longer than normal cycle
    # if initFlag == 0:
    #     startPoint = convertPose(data)
    #     calculateCorners(startPoint)
    #     initFlag = 1

    #Neato reachs a corner
    # elif 

    #     if cornerNumber == sides-1:
    #         cornerNumber = 0
    #     else:
    #         cornerNumber += 1





def run():

    #Set up subscriber
    rospy.init_node('squareDriver', anonymous=True)
    rospy.Subscriber("/odom", Pose, drive)
    rospy.spin()

if __name__ == '__main__':
    run()