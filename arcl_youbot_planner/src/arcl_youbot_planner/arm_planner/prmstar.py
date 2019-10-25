#!/usr/bin/env python
import rospy
import math
import pybullet as p
import time
import numpy as np
import pybullet_data
import arcl_youbot_planner.arm_planner.arm_util as arm_util
# import arcl_youbot_application.application_util as app_util
import arcl_youbot_application.common_util as common_util
# import control_msgs.msg.FollowJointTrajectoryActionGoal
from geometry_msgs.msg import Twist 
import arcl_youbot_planner.arm_planner.astar 
import scipy.spatial

SAMPLE_NUM = 500
ROBOT_COLLISION_CHECK_RADIUS = 0.7
PATH_PLAN_TRYING_MAX = 10


class PRMStarPlanner():
    def __init__(self, p_client, urdf_path):
        self.sample_num = SAMPLE_NUM
        self.p_client = p_client
        self.urdf_path = urdf_path
        self.robot_id = self.p_client.loadURDF(self.urdf_path, flags=self.p_client.URDF_USE_SELF_COLLISION)
        self.ARM_JOINT_NUM = arm_util.ARM_JOINT_NUM
        self.joint_mat = np.load('/home/wei/catkin_youbot_ws/src/arcl_youbot_planner/src/arcl_youbot_planner/arm_planner/prm_roadmap/prm_roadmap.npy')
        self.joint_neighbor = np.load('/home/wei/catkin_youbot_ws/src/arcl_youbot_planner/src/arcl_youbot_planner/arm_planner/prm_roadmap/prm_roadmap_neighbor.npy')
        joint_dict_object = np.load('/home/wei/catkin_youbot_ws/src/arcl_youbot_planner/src/arcl_youbot_planner/arm_planner/prm_roadmap/prm_roadmap_dict.npy', allow_pickle=True)
        self.joint_dict = joint_dict_object.item()
        self.tree = None
        self.object_list = None
        self.robot_pose2d = None
        self.robot_position = None
        self.robot_orientation = None
        self.object_position_list = None
        self.object_size_list = None
        self.object_pybullet_id_list = None
        self.object_orientation_list = None
        

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
        np.save('prm_roadmap/prm_roadmap.npy', joint_mat)

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
        np.save('prm_roadmap/prm_roadmap_neighbor.npy', joint_neighbor)
        print "neighbor finish"
        print len(joint_dict)
        np.save('prm_roadmap/prm_roadmap_dict.npy', joint_dict)


    def direct_path(self, start, goal):
        path = []
        path.append(start)
        path.append(goal)
        return path, 0

    def path_plan(self, start, goal):
        start_arr = np.asarray(start)
        goal_arr = np.asarray(goal)
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

        self.p_client.resetBasePositionAndOrientation(self.robot_id, self.robot_position, self.robot_orientation)

        is_path_in_collision = True
        path_plan_trying_times = 0
        path = []
        final_cost = -1.0
        while is_path_in_collision == False:
            astar_graph = arcl_youbot_planner.arm_planner.astar.AStarGraph(temp_joint_mat, temp_joint_neighbor, temp_joint_dict)
            path, path_vertex_list, final_cost = arcl_youbot_planner.arm_planner.astar.AStarSearch(start, goal, astar_graph)

            close_obj_index_list = []
            for obj_pos, obj_index in zip(self.object_position_list, range(len(self.object_position_list))):
                if math.sqrt(math.pow(obj_pos[0] - self.robot_position[0], 2) + math.pow(obj_pos[1] - self.robot_position[1], 2)) < ROBOT_COLLISION_CHECK_RADIUS:
                    close_obj_index_list.append(obj_index)
            
            
            is_path_in_collision, self_collision_edge_list, external_collision_edge_list = check_path_collision(path, path_vertex_list, close_obj_index_list)
            if is_path_in_collision:
                for self_coll_pair in self_collision_edge_list:
                    if self_coll_pair[0] < SAMPLE_NUM and self_coll_pair[1] < SAMPLE_NUM:
                        #delete the neighbor edge from the self.joint_neighbor
                        self.joint_neighbor[self_coll_pair[0], self_coll_pair[1]] = 0
                        self.joint_neighbor[self_coll_pair[1], self_coll_pair[0]] = 0
                    temp_joint_neighbor[self_coll_pair[0], self_coll_pair[1]] = 0
                    temp_joint_neighbor[self_coll_pair[1], self_coll_pair[0]] = 0
                for external_coll_pair in external_collision_edge_list:
                    temp_joint_neighbor[external_coll_pair[0], external_coll_pair[1]] = 0
                    temp_joint_neighbor[external_coll_pair[1], external_coll_pair[0]] = 0
            else:
                # get collision free path
                break

            path_plan_trying_times += 1
            if path_plan_trying_times > PATH_PLAN_TRYING_MAX:
                print("PRMSTAR: path planning failed after " + PATH_PLAN_TRYING_MAX + " times")
                break
        return path, final_cost

    def check_path_collision(self, path, path_vertex_list, close_obj_index_list):
        self_collision_edge_list = []
        external_collision_edge_list = []
        is_self_collision = False
        is_external_collision = False
        is_collision = False
        for pt_index in range(len(path)):
            if pt_index < (len(path)-1):
                for interpolate_index in range(PATH_INTERPOLATE_NUM):
                    inter_pt = []
                    current_pt = path[pt_index]
                    next_pt = path[pt_index+1]
                    for index, j in  zip(range(self.ARM_JOINT_NUM), current_pt):
                        inter_pt.append(j + interpolate_index * (next_pt[index] - current_pt[index]) / PATH_INTERPOLATE_NUM)

                    arm_util.set_arm_joint_pos(inter_pt, self.p_client, self.robot_id)
                    self.p_client.stepSimulation()
                    self_collision_list = self.p_client.getContactPoints(self.robot_id, self.robot_id)
                    if len(self_collision_list) == 2:
                        #print "no collision"
                        pass                        
                    else:
                        self_collision_edge_list.append((path_vertex_list[pt_index], path_vertex_list[pt_index + 1]))
                        # already have self-collision, no need to test external collision anymore
                        is_self_collision = True
                        break
                    is_this_seg_external_collision = False
                    for obj_index in close_obj_index_list:
                        external_collision_list = self.p_client.getContactPoints(self.robot_id, self.object_pybullet_id_list[obj_index])
                        if len(external_collision_list) > 0:
                            #print("exist external collision")
                            external_collision_edge_list.append((path_vertex_list[pt_index], path_vertex_list[pt_index + 1]))
                            is_this_seg_external_collision = True
                            is_external_collision = True
                            break
                    if is_this_seg_external_collision:
                        break
        if is_self_collision or is_external_collision:
            is_collision = True
        return is_collision, self_collision_edge_list, external_collision_edge_list

    def import_obstacles(self, object_list):
        self.object_list = object_list
        # self.object_position = 
        for obj in self.object_list:
            size, position, quaternion = common_util.get_info_from_cube(obj)

            obj_collision_id = self.p_client.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=size)
            obj_pybullet_id = self.p_client.createMultiBody(baseCollisionShapeIndex=obj_collision_id, basePosition=position, baseOrientation=quaternion)
            self.object_pybullet_id_list.append(obj_pybullet_id)
            self.object_position_list.append(position)
            self.object_orientation_list.append(quaternion)
            self.object_size_list.append(size)

    def set_robot_pose2d(self, pose2d):
        self.robot_pose2d = pose2d   
    
    def set_robot_pose(self, position, orientation):
        self.robot_position = position
        self.robot_orientation = orientation  

if __name__ == "__main__":
    rospy.init_node('prmstar_test')
    physicsClient = p.connect(p.DIRECT)#or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
    p.setGravity(0,0,-10)
    prmstar = PRMStarPlanner(p, "/home/baichuan/ws/src/luh_youbot_description/robots/youbot_0.urdf")
    #prmstar.build_roadmap()
    start = np.zeros(arm_util.ARM_JOINT_NUM)
    for jnt in range(arm_util.ARM_JOINT_NUM):
        start[jnt] = np.random.uniform(arm_util.MIN_JOINT_POS[jnt] - arm_util.JOINT_OFFSET[jnt] ,arm_util.MAX_JOINT_POS[jnt] - arm_util.JOINT_OFFSET[jnt])
    goal = np.zeros(arm_util.ARM_JOINT_NUM)
    for jnt in range(arm_util.ARM_JOINT_NUM):
        goal[jnt] = np.random.uniform(arm_util.MIN_JOINT_POS[jnt] - arm_util.JOINT_OFFSET[jnt] ,arm_util.MAX_JOINT_POS[jnt] - arm_util.JOINT_OFFSET[jnt])
        
    prmstar.path_plan(tuple(start), tuple(goal))
    rospy.spin()










