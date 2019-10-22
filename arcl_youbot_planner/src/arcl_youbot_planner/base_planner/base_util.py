import rospy
import math
import pybullet as p
import time
import pybullet_data
import numpy as np
import actionlib
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from arcl_youbot_msgs.msg import MoveBaseAction, MoveBaseGoal
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion

#youbot_name: youbot
def get_youbot_base_pose(youbot_name):
    data = rospy.wait_for_message('gazebo/model_states', ModelStates)
    current_pose = [0,0,0]
    youbot_index = 0
    for name, data_index in zip(data.name, range(len(data.name))):
        if name == youbot_name:
            youbot_index = data_index


    current_pose[0] = data.pose[youbot_index].position.x
    current_pose[1] = data.pose[youbot_index].position.y
    q = (data.pose[youbot_index].orientation.x,
             data.pose[youbot_index].orientation.y,
             data.pose[youbot_index].orientation.z,
             data.pose[youbot_index].orientation.w)
    (roll, pitch, yaw) = euler_from_quaternion(q)
    current_pose[2] = yaw
    return current_pose

# base_action_name:   "youbot_base/move"
def execute_path(final_path, base_action_name):
    client = actionlib.SimpleActionClient(base_action_name, MoveBaseAction)
    client.wait_for_server()
    goal = MoveBaseGoal()
    begin_time = 0
    for pt_index in range(len(final_path)):
        goal.x = final_path[pt_index][0]
        goal.y = final_path[pt_index][1]
        goal.theta = final_path[pt_index][2]
        client.send_goal(goal)
        client.wait_for_result(rospy.Duration.from_sec(10.0)) 
        
        
       