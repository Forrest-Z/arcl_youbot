from __future__ import print_function
import rospy
import arcl_youbot_application.application_util as app_util
import arcl_youbot_planner.arm_planner.arm_util as arm_util
from geometry_msgs.msg import Pose, PointStamped, Point, Quaternion, Twist
from sensor_msgs.msg import JointState
from arcl_youbot_msgs.msg import SetGripperAction, SetGripperGoal, MoveToJointPoseGoal, MoveToJointPoseAction
import actionlib
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


def move_arm(youbot_name, joint_change):
    client = actionlib.SimpleActionClient(youbot_name+"/arm_1/to_joint_pose/plan", MoveToJointPoseAction)
    print("wait for server")
    client.wait_for_server()
    goal = MoveToJointPoseGoal()
    goal.pose_is_relative = True
    goal.pose.q1 =  joint_change[0]
    goal.pose.q2 =  joint_change[1]
    goal.pose.q3 =  joint_change[2]
    goal.pose.q4 =  joint_change[3]
    goal.pose.q5 =  joint_change[4]
    
    print(goal.pose)
    client.send_goal(goal)
    client.wait_for_result()
    

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
        elif (char == "t"):
            print("increase joint 1")
            move_arm(youbot_name, [0.1, 0, 0, 0, 0])
        elif (char == "g"):
            print("decrease joint 1")
            move_arm(youbot_name, [-0.1, 0, 0, 0, 0])
        elif (char == "y"):
            print("increase joint 2")
            move_arm(youbot_name, [0, 0.1, 0, 0, 0])
        elif (char == "h"):
            print("decrease joint 2")
            move_arm(youbot_name, [0, -0.1, 0, 0, 0])
        elif (char == "u"):
            print("increase joint 3")
            move_arm(youbot_name, [0, 0, 0.1, 0, 0])
        elif (char == "j"):
            print("decrease joint 3")
            move_arm(youbot_name, [0, 0, -0.1, 0, 0])
        elif (char == "i"):
            print("increase joint 4")
            move_arm(youbot_name, [0, 0, 0, 0.1, 0])
        elif (char == "k"):
            print("decrease joint 4")
            move_arm(youbot_name, [0, 0, 0, -0.1, 0])
        elif (char == "o"):
            print("increase joint 5")
            move_arm(youbot_name, [0, 0, 0, 0, 0.1])
        elif (char == "l"):
            print("decrease joint 5")
            move_arm(youbot_name, [0, 0, 0, 0, -0.1])
        # elif (char == "r"):
        #     walk(vel_pub)

        vel_pub.publish(msg)