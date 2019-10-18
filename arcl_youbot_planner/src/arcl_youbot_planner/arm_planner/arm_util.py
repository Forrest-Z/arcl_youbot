import rospy
import math
import pybullet as p
import time
import pybullet_data
import numpy as np

JOINT_1_INDEX = 8
JOINT_2_INDEX = 9
JOINT_3_INDEX = 10
JOINT_4_INDEX = 11
JOINT_5_INDEX = 12



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
5