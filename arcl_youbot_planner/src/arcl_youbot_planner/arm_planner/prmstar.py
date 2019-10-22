#!/usr/bin/env python
import rospy
import math
import pybullet as p
import time
import numpy as np
import pybullet_data
import arcl_youbot_planner.arm_planner.arm_util as arm_util
# import control_msgs.msg.FollowJointTrajectoryActionGoal
from geometry_msgs.msg import Twist 
import arcl_youbot_planner.arm_planner.astar 
import scipy.spatial

SAMPLE_NUM = 500


class PRMStarPlanner():
    def __init__(self, p_client, urdf_path):
        self.sample_num = SAMPLE_NUM
        self.p_client = p_client
        self.urdf_path = urdf_path
        self.robot_id = self.p_client.loadURDF(self.urdf_path, flags=self.p_client.URDF_USE_SELF_COLLISION)
        self.ARM_JOINT_NUM = arm_util.ARM_JOINT_NUM
        self.tree = None
        self.joint_dict = None
        self.joint_neighbor = None
        self.joint_mat = None

    def build_roadmap(self):
        joint_mat = np.zeros([self.ARM_JOINT_NUM, self.sample_num])
        joint_dict = {}
        sample_index = 0
        while sample_index < self.sample_num:
            jnt_vector = []
            for jnt in range(self.ARM_JOINT_NUM):
                joint_mat[jnt, sample_index] = np.random.uniform(arm_util.MIN_JOINT_POS[jnt] - arm_util.JOINT_OFFSET[jnt] ,arm_util.MAX_JOINT_POS[jnt] - arm_util.JOINT_OFFSET[jnt])
            arm_util.set_arm_joint_pos(joint_mat[:, sample_index], self.p_client, self.robot_id)
            self.p_client.stepSimulation()
            collision_list = self.p_client.getContactPoints(self.robot_id)
            if len(collision_list) == 2:
                #print "no collision"
                joint_dict[tuple(joint_mat[:, sample_index])] = sample_index
                sample_index = sample_index + 1
            else:
                pass
                #print "self collision"
        
            #connect neighbors

        
        print "finish"        
        np.save('prm_roadmap.npy', joint_mat)

        self.tree = scipy.spatial.cKDTree(joint_mat.T)
        joint_neighbor = np.zeros([self.sample_num, self.sample_num])
        neighbor_num = int(math.ceil(math.log(self.sample_num) * 2.7 * (1 + 1.0 / self.ARM_JOINT_NUM)))
        sample_index = 0
        print self.tree.m
        print self.tree.n
        while sample_index < self.sample_num:
            dist, index = self.tree.query(joint_mat[:, sample_index], neighbor_num)
            joint_neighbor[sample_index, index] = 1
            sample_index += 1
        np.save('prm_roadmap_neighbor.npy', joint_neighbor)
        print "neighbor finish"
        print len(joint_dict)
        np.save('prm_roadmap_dict.npy', joint_dict)


    def path_plan(self, start, goal):
        start_arr = np.asarray(start)
        goal_arr = np.asarray(goal)
        self.joint_mat = np.load('/home/wei/catkin_youbot_ws/src/arcl_youbot_planner/src/arcl_youbot_planner/arm_planner/prm_roadmap.npy')
        self.joint_neighbor = np.load('/home/wei/catkin_youbot_ws/src/arcl_youbot_planner/src/arcl_youbot_planner/arm_planner/prm_roadmap_neighbor.npy')
        joint_dict_object = np.load('/home/wei/catkin_youbot_ws/src/arcl_youbot_planner/src/arcl_youbot_planner/arm_planner/prm_roadmap_dict.npy', allow_pickle=True)
        self.joint_dict = joint_dict_object.item()
        neighbor_num = int(math.ceil(math.log(self.sample_num) * 2.7 * (1 + 1.0 / self.ARM_JOINT_NUM)))


        temp_joint_mat = self.joint_mat
        temp_joint_neighbor = np.zeros([self.sample_num + 2, self.sample_num + 2])
        temp_joint_neighbor[:-2, :-2] = self.joint_neighbor
        print("before insert start and goal")
        start_neighbor_index = arm_util.find_nearest_neighbor(start_arr, self.joint_mat, neighbor_num)
        goal_neighbor_index = arm_util.find_nearest_neighbor(goal_arr, self.joint_mat, neighbor_num)
        print("after insert start and goal")        
        temp_joint_neighbor[self.sample_num,start_neighbor_index] = 1
        temp_joint_neighbor[start_neighbor_index,self.sample_num] = 1
        temp_joint_neighbor[self.sample_num+1,goal_neighbor_index] = 1
        temp_joint_neighbor[goal_neighbor_index,self.sample_num+1] = 1

        temp_joint_dict = self.joint_dict
        print(temp_joint_mat.shape)
        print(start_arr.shape)
        start_arr = start_arr.reshape(self.ARM_JOINT_NUM, 1)
        goal_arr = goal_arr.reshape(self.ARM_JOINT_NUM, 1)
        temp_joint_mat = np.hstack((temp_joint_mat, start_arr))
        temp_joint_dict[start] = temp_joint_mat.shape[1]-1
        temp_joint_mat = np.hstack((temp_joint_mat, goal_arr))
        temp_joint_dict[goal] = temp_joint_mat.shape[1]-1

                              

        astar_graph = arcl_youbot_planner.arm_planner.astar.AStarGraph(temp_joint_mat, temp_joint_neighbor, temp_joint_dict)
        path, final_cost = arcl_youbot_planner.arm_planner.astar.AStarSearch(start, goal, astar_graph)
        return path, final_cost

if __name__ == "__main__":
    rospy.init_node('prmstar_test')
    physicsClient = p.connect(p.DIRECT)#or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
    p.setGravity(0,0,-10)
    prmstar = PRMStarPlanner(p, "/home/wei/catkin_youbot_ws/src/luh_youbot_description/robots/youbot_0.urdf")
    #prmstar.build_roadmap()
    start = np.zeros(arm_util.ARM_JOINT_NUM)
    for jnt in range(arm_util.ARM_JOINT_NUM):
        start[jnt] = np.random.uniform(arm_util.MIN_JOINT_POS[jnt] - arm_util.JOINT_OFFSET[jnt] ,arm_util.MAX_JOINT_POS[jnt] - arm_util.JOINT_OFFSET[jnt])
    goal = np.zeros(arm_util.ARM_JOINT_NUM)
    for jnt in range(arm_util.ARM_JOINT_NUM):
        goal[jnt] = np.random.uniform(arm_util.MIN_JOINT_POS[jnt] - arm_util.JOINT_OFFSET[jnt] ,arm_util.MAX_JOINT_POS[jnt] - arm_util.JOINT_OFFSET[jnt])
        
    prmstar.path_plan(tuple(start), tuple(goal))
    rospy.spin()










