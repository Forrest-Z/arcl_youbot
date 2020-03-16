from __future__ import print_function
import rospy
import arcl_youbot_application.application_util as app_util
import arcl_youbot_planner.arm_planner.arm_util as arm_util
from geometry_msgs.msg import Pose, PointStamped, Point, Quaternion, Twist
from std_msgs.msg import String
from std_msgs.msg import UInt8
import os.path
import math
import time
# import TSP_DP_new as TSP_DP
import copy
import sys, termios, tty, os, time


def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
 
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def walk(vel_pub):
    # forward
    msg = Twist()
    start_time = time.time()
    while time.time() - start_time < 2.5:
        msg.linear.x = 0.2
        vel_pub.publish(msg)

    # right
    msg = Twist()
    start_time = time.time()
    while time.time() - start_time < 2.5:
        msg.linear.y = -0.2
        vel_pub.publish(msg)

    # back to origin
    msg = Twist()
    start_time = time.time()
    while time.time() - start_time < 2.5:
        msg.linear.x = -0.2
        msg.linear.y = 0.2
        vel_pub.publish(msg)

if __name__ == "__main__":
    rospy.init_node("base_key_control")

    youbot_name = 'youbot_0'
    vel_pub = rospy.Publisher('/' + youbot_name + '/robot/cmd_vel', Twist, queue_size=1)
    msg = Twist()

    while True:
        char = getch()
    
        if (char == "x"):
            print("Stop!")
            exit(0)
    
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.angular.z = 0.0

        if (char == "a"):
            print("Left pressed")
            msg.linear.y = 0.2
        elif (char == "d"):
            print("Right pressed")
            msg.linear.y = -0.2
        elif (char == "w"):
            print("Forward pressed") 
            msg.linear.x = 0.2
        elif (char == "s"):
            print("Back pressed") 
            msg.linear.x = -0.2
        elif (char == "q"):
            print("Left rotate pressed") 
            msg.angular.z = 0.4
        elif (char == "e"):
            print("Right rotate  pressed")
            msg.angular.z = -0.4
        elif (char == "r"):
            walk(vel_pub)

        vel_pub.publish(msg)