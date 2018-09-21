#!/usr/bin/env python
"""
Ben Ziemann
Last updated: 9/20/18

Moves the Neato using teleop keystrokes
"""

import tty
import select
import sys
import termios
import rospy
from geometry_msgs.msg import Twist


def talker():
    """
    Publishes keystrokes to control the Neato teleop style
    """

    #ROS setup
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('myTeleOp')
    rate = rospy.Rate(2)

    msg = Twist()
    msg.linear.y = 0
    msg.linear.z = 0
    msg.angular.x = 0
    msg.angular.y = 0

    #Keystroke setup
    key = None

    while not rospy.is_shutdown():
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1).strip()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

        keyDict = {"u":[.5,1],
                    "i":[.5,0],
                    "o":[.5,-1],
                    "j":[0,1],
                    "k":[0,0],
                    "l":[0,-1],
                    "m":[-.5,-1],
                    ",":[-.5,0],
                    ".":[-.5,1]}
        
        if key not in keyDict:
            print("Not valid key")
        else:
            msg.linear.x = keyDict[key][0]
            msg.angular.z = keyDict[key][1]

        pub.publish(msg)
        rate.sleep()


if __name__ == "__main__":
    print("Use these keys to move and turn:")
    print("u\ti\to")
    print("j\tk\tl")
    print("m\t,\t.\n")
    settings = termios.tcgetattr(sys.stdin)
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
