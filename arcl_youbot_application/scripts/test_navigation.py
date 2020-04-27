import rospy
import arcl_youbot_application.application_util as app_util
import arcl_youbot_planner.arm_planner.arm_util as arm_util
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from std_msgs.msg import UInt8
import os.path
import math
import numpy as np
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def test_navigation_cb(goal_state, result):
    print("navigation done")

if __name__ == "__main__":
    rospy.init_node("test_navigation")
    # env = app_util.YoubotEnvironment(-1.5, 1.5, -3.0, 1.0)    
    # env = app_util.YoubotEnvironment(-2.5, 2.5, 0.0, 5.0)
    # env.mode = 1
    #import object list from file
    # my_path = os.path.abspath(os.path.dirname(__file__))
    
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    client.wait_for_server()
    
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time(0)
    #1 point
    goal.target_pose.pose.position.x = -0.249759674072
    goal.target_pose.pose.position.y = -1.9749045372
    goal.target_pose.pose.orientation.w = 1.0
    #2 point
    # goal.target_pose.pose.position.x = -1.48454189301
    # goal.target_pose.pose.position.y = -0.72617149353
    client.send_goal(goal, done_cb=test_navigation_cb)
    client.wait_for_result(rospy.Duration.from_sec(10.0))
    
    # env.move_arm_to_joint_pose("youbot_0", arm_up_joint)
    # env.move_arm_to_joint_pose("youbot_0", arm_up_joint_1)
    # env.move_arm_to_joint_pose("youbot_0", arm_up_joint_2)
    # env.move_arm_to_joint_pose("youbot_0", arm_up_joint_3)
    # env.move_arm_to_joint_pose("youbot_0", arm_up_joint_4)
    # env.move_arm_to_joint_pose("youbot_0", arm_up_joint_5)
    # env.move_arm_to_joint_pose("youbot_0", arm_up_joint_6)


    rospy.spin()
   