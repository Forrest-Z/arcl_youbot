import rospy
from arcl_youbot_msgs.msg import SetGripperAction, SetGripperGoal, MoveToJointPoseGoal, MoveToJointPoseAction
import arcl_youbot_planner.arm_planner.arm_util as arm_util
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from std_msgs.msg import UInt8
import os.path
import math
import numpy as np
import actionlib


def move_arm_to_joint_pose_old_direct(youbot_name, joint_target):
        client = actionlib.SimpleActionClient( youbot_name+"/arm_1/to_joint_pose/inter", MoveToJointPoseAction)
        print("wait for server")
        client.wait_for_server()
        goal = MoveToJointPoseGoal()
        goal.pose_is_relative = False
        goal.pose.q1 =  joint_target[0] + arm_util.JOINT_OFFSET[0]
        goal.pose.q2 =  joint_target[1] + arm_util.JOINT_OFFSET[1]
        goal.pose.q3 =  joint_target[2]+ arm_util.JOINT_OFFSET[2]
        goal.pose.q4 =  joint_target[3]+ arm_util.JOINT_OFFSET[3]
        goal.pose.q5 =  joint_target[4]+ arm_util.JOINT_OFFSET[4]
        
        client.send_goal_and_wait(goal)

if __name__ == "__main__":
    rospy.init_node("reachibility")

    youbot_name = 'youbot_1'

    arm_near_reset_joint = [0.2726646259971648, 0.07, -0.3581317007977318, 0.109522973054868, 3.001966313430247]
    arm_up_joint = [202/180.0*math.pi, 65/180.0*math.pi, -146 / 180.0 * math.pi, 102.5 / 180.0 * math.pi, 172 / 180.0 * math.pi]
    arm_up_joint_1 = [155/180.0*math.pi, 85/180.0*math.pi, -146 / 180.0 * math.pi, 102.5 / 180.0 * math.pi, 172 / 180.0 * math.pi]
    arm_up_joint_2 = [202/180.0*math.pi, 25/180.0*math.pi, -86 / 180.0 * math.pi, 102.5 / 180.0 * math.pi, 172 / 180.0 * math.pi]
    arm_up_joint_3 = [202/180.0*math.pi, 65/180.0*math.pi, -146 / 180.0 * math.pi, 76.5 / 180.0 * math.pi, 172 / 180.0 * math.pi]
    arm_up_joint_4 = [202/180.0*math.pi, 65/180.0*math.pi, -146 / 180.0 * math.pi, 102.5 / 180.0 * math.pi, 172 / 180.0 * math.pi]
    arm_up_joint_5 = [202/180.0*math.pi, 120/180.0*math.pi, -80 / 180.0 * math.pi, 102.5 / 180.0 * math.pi, 121 / 180.0 * math.pi]
    arm_up_joint_6 = [202/180.0*math.pi, 65/180.0*math.pi, -146 / 180.0 * math.pi, 102.5 / 180.0 * math.pi, 172 / 180.0 * math.pi]
    
    move_arm_to_joint_pose_old_direct(youbot_name, arm_up_joint)
    move_arm_to_joint_pose_old_direct(youbot_name, arm_up_joint_1)
    move_arm_to_joint_pose_old_direct(youbot_name, arm_up_joint_2)
    move_arm_to_joint_pose_old_direct(youbot_name, arm_up_joint_3)
    move_arm_to_joint_pose_old_direct(youbot_name, arm_up_joint_4)
    move_arm_to_joint_pose_old_direct(youbot_name, arm_up_joint_5)
    move_arm_to_joint_pose_old_direct(youbot_name, arm_up_joint_6)
    move_arm_to_joint_pose_old_direct(youbot_name, arm_near_reset_joint)
   
    client = arm_util.set_gripper_width(youbot_name, 0.06, 1)
    client.wait_for_result(rospy.Duration.from_sec(10.0))
    arm_util.set_gripper_width(youbot_name, 0.0, 1)
    client.wait_for_result(rospy.Duration.from_sec(10.0))