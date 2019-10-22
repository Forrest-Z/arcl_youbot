import rospy
import math
import pybullet as p
import time
import pybullet_data
import numpy as np
import actionlib
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal


JOINT_1_INDEX = 8
JOINT_2_INDEX = 9
JOINT_3_INDEX = 10
JOINT_4_INDEX = 11
JOINT_5_INDEX = 12

SAMPLE_NUM = 500
PATH_INTERPOLATE_NUM=10
MIN_JOINT_POS = [
    -2.9395372288,
    -1.124398163,
    -2.4783710285,
    -1.7668361332,
    -2.8913271881]
MAX_JOINT_POS = [
    2.8905337534,
    1.4835265078,
    2.532469254,
    1.6402432236,
    2.6396457936]
JOINT_OFFSET = [
    -2.9395372288,
    -1.124398163,
    2.532469254,
    -1.7668361332,
    -2.8913271881]

ARM_JOINT_NUM = 5





def set_arm_joint_pos(joint_vector, pybullet_client, youbot_id):
    pybullet_client.resetJointState(youbot_id, JOINT_1_INDEX, joint_vector[0])
    pybullet_client.resetJointState(youbot_id, JOINT_2_INDEX, joint_vector[1])
    pybullet_client.resetJointState(youbot_id, JOINT_3_INDEX, joint_vector[2])
    pybullet_client.resetJointState(youbot_id, JOINT_4_INDEX, joint_vector[3])
    pybullet_client.resetJointState(youbot_id, JOINT_5_INDEX, joint_vector[4])

def find_nearest_neighbor(query, joint_mat, neighbor_num):
    sample_num = joint_mat.shape[1]
    index = 0
    n_index = 0
    dist_arr = np.zeros([sample_num])
    while index < sample_num:
        dist = (joint_mat[:,index] - query)*(joint_mat[:, index] - query)
        dist_arr[index] = dist.sum()
        index += 1
    dist_index = np.argpartition(dist_arr, neighbor_num)
    return dist_index[:neighbor_num]

def execute_path(final_path, joint_action_name):
    client = actionlib.SimpleActionClient(joint_action_name, FollowJointTrajectoryAction)
    client.wait_for_server()
    goal = FollowJointTrajectoryGoal()
    begin_time = 0
    for pt_index in range(len(final_path)):
        if pt_index < (len(final_path)-1):
            for interpolate_index in range(PATH_INTERPOLATE_NUM):
                traj_pt = JointTrajectoryPoint()
                current_pt = final_path[pt_index]
                next_pt = final_path[pt_index+1]
                for index, j in  zip(range(ARM_JOINT_NUM), current_pt):
                    traj_pt.positions.append(j + JOINT_OFFSET[index] + interpolate_index * (next_pt[index] - current_pt[index]) / PATH_INTERPOLATE_NUM)
                begin_time += 0.2
                traj_pt.time_from_start = rospy.Duration.from_sec(begin_time)
                goal.trajectory.points.append(traj_pt)
        else:
            traj_pt = JointTrajectoryPoint()
            for index, j in  zip(range(ARM_JOINT_NUM),final_path[pt_index]):
                traj_pt.positions.append(j + JOINT_OFFSET[index])
            begin_time += 0.2
            traj_pt.time_from_start = rospy.Duration.from_sec(begin_time)
            goal.trajectory.points.append(traj_pt)
    goal.goal_time_tolerance = rospy.Duration.from_sec(10)
    print(goal)
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(10.0))