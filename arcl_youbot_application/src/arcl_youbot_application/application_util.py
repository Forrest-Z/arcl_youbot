#!/usr/bin/env python
import rospy
import math
import pybullet as p
import time
import random
import numpy as np
import pybullet_data
import arcl_youbot_planner.arm_planner.arm_util as arm_util
import arcl_youbot_planner.base_planner.base_util as base_util
# import control_msgs.msg.FollowJointTrajectoryActionGoal
from geometry_msgs.msg import Twist 
from gazebo_msgs.srv import SpawnModel, SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String
from std_msgs.msg import UInt8, Float64
import arcl_youbot_planner.arm_planner.astar 
from arcl_youbot_msgs.msg import SetGripperAction, SetGripperGoal, MoveToJointPoseGoal, MoveToJointPoseAction
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_about_axis, quaternion_from_matrix
import scipy.spatial
import string
import actionlib
import StringIO
from arcl_youbot_application.msg import ManipulationAction, ManipulationGoal
from arcl_youbot_application.msg import PlanningSceneMsg
from arcl_youbot_application.msg import SceneObjectMsg
import arcl_youbot_planner.base_planner.visgraph as vg
import arcl_youbot_planner.arm_planner.prmstar as prmstar
import arcl_youbot_application.common_util as common_util
from youbot_forklift_ros_interface.msg import GoToPositionAction, GoToPositionGoal
import os.path
from geometry_msgs.msg import PoseStamped
import re
import threading
from multiprocessing import Process
import signal
import sys
import copy


GAZEBO_COLORS = [ 
"Gazebo/White",
"Gazebo/Grey",
"Gazebo/Red",
"Gazebo/Green",
"Gazebo/Yellow",
"Gazebo/Purple",
"Gazebo/Turquoise",
"Gazebo/Blue",
"Gazebo/Gold",
"Gazebo/FlatBlack"
]

# clock-wise
WALL = [(2.8, -0.3), (0.7, -0.3), (0.7, 0.0), (2.5, 0.0), (2.5, 5.0), (-2.5, 5.0), (-2.5, 0.0), (-0.7, 0.0), (-0.7, -0.3), (-2.8, -0.3), (-2.8, 5.3), (2.8, 5.3)]
WALL.reverse()

def ctrl_c_handler(signal, frame):
    """Gracefully quit the infrared_pub node"""
    print "\nCaught ctrl-c! Stopping node."
    sys.exit()



class YoubotEnvironment(): 
    
    
    def __init__(self, x_min, x_max, y_min, y_max, youbot_name="", mode=0):
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.object_list = {}    #example object_list(contain 3 polygons): [[(1, 1), (2, 1.5), (1, 2)], [(3, 2), (4, 2), (4, 4), (3, 4)], [(4, 3), (4.5, 1), (5, 3)]]    
        self.obj_name_to_index_dict = {}
        self.planning_scene_msg = PlanningSceneMsg()
        self.reserved_planning_scene_msg = PlanningSceneMsg()
        self.mode = mode
        self.base_controller = base_util.BaseController(youbot_name, self.mode)
        self.prmstar_planner = None
    def import_obj_from_file(self, filename):
        # filename: ffabsolute path for the environment file

        f = open(filename, 'r')
        self.object_num = int(f.readline())
        for obj_index in range(self.object_num):
            vertex_num = int(f.readline())
            current_obj = []
            for _ in range(vertex_num):
                point = string.split(f.readline())
                point[0] = float(point[0])
                point[1] = float(point[1])
                current_obj.append((point[0], point[1]))
            self.object_list['obj_'+str(obj_index)] = current_obj
        print(self.object_list)
        self.object_list['wall'] = WALL

        print len(self.object_list)

    def export_obj_to_file(self, filename):
        # filename: ffabsolute path for the environment file

        f = open(filename, 'w')
        f.write(str(self.object_num)+'\n')

        output = copy.deepcopy(self.object_list)
        if 'wall' in output:
            del output['wall']

        for obj in output.values():
            f.write(str(len(obj)-1)+'\n')
            for v in obj[:-1]:
                f.write(str(v[0]) + " " + str(v[1])+'\n')

    def import_obj_from_list(self, object_list):
        self.object_list = object_list
    

    def create_environment(self, object_number, cluster_number):
        # create random object_list in the current environment
        # INPUT: 
            #object_number: total object num 
            #cluster_number: number of clusters the objects are separated to.
        # EXAMPLE: create_environment(20, 20) will create 20 scattered objects
    
        boundary_padding = 0.5
        x_min = self.x_min + boundary_padding
        x_max = self.x_max - boundary_padding
        y_min = self.y_min + boundary_padding
        y_max = self.y_max - boundary_padding

        created_objs = []
        each_cluster_obj_num = 0
        each_cluster_obj_num = int(math.ceil(object_number / cluster_number))
        cluster_obj_num_list = []
        for i in range(cluster_number-1):
            cluster_obj_num_list.append(each_cluster_obj_num)
        cluster_obj_num_list.append(object_number - (cluster_number-1)*each_cluster_obj_num)

        for i in range(cluster_number):
            #generate objects per cluster

            is_valid = False
            random.seed()
            while not is_valid:  
                is_valid = True
                center_x = random.random() * (x_max - x_min) + x_min
                center_y = random.random() * (y_max - y_min) + y_min
                yaw = random.random() * math.pi

                length = random.random() * (common_util.OBJECT_LENGTH_MAX - common_util.OBJECT_LENGTH_MIN) + common_util.OBJECT_LENGTH_MIN
                width = common_util.OBJECT_WIDTH
                new_poly = common_util.generate_poly(center_x, center_y, yaw, length, width)

                for exist_obj in created_objs:
                    if not exist_obj.equals(new_poly):
                        if exist_obj.intersects(new_poly):
                            is_valid = False
                            break
                
                if not is_valid:
                    continue

                created_objs.append(new_poly)
                same_cluster_objs = []
                same_cluster_objs.append(new_poly)
                for j in range(cluster_obj_num_list[i]-1):
                    tp = common_util.add_near_poly(same_cluster_objs, created_objs, x_min, x_max, y_min, y_max, boundary_padding) 
                    same_cluster_objs.append(tp)
                    created_objs.append(tp)
    
        list_created_objs = {}

        for obj, obj_index in zip(created_objs, range(len(created_objs))):
            list_created_objs['obj_'+str(obj_index)] = list(obj.exterior.coords)

        self.import_obj_from_list(list_created_objs)
        self.object_num = len(self.object_list)

        #add the environemnt wall at last
        self.object_list['wall'] = WALL

    def import_obj_from_optitrack(self):
        #auto detect objects in the optitrack region, generate self.object_list and obj_name_list, and generate planning_scene_msg
        obj_name_list = []
        topic_list = rospy.get_published_topics()
        for [topic_name, topic_type] in topic_list:
            if re.match(r"/vrpn_client_node/obj_.*", topic_name):
                obj_name = re.search("obj_\d*", topic_name)
                print(obj_name.group(0))
                obj_name_list.append(obj_name.group(0))


        for obj_name, obj_index in zip(obj_name_list, range(len(obj_name_list))):
            data = rospy.wait_for_message('/vrpn_client_node/' + obj_name + '/pose', PoseStamped)
            
            current_pose = [0, 0, 0]
            current_pose[0] = data.pose.position.x
            current_pose[1] = data.pose.position.y
            q = (data.pose.orientation.x,
                    data.pose.orientation.y,
                    data.pose.orientation.z,
                    data.pose.orientation.w)
            (roll, pitch, yaw) = euler_from_quaternion(q) 
            print('obj_name:' + str(obj_name))
            print('yaw:' + str(yaw))
            print('roll:' + str(roll))
            print('pitch:' + str(pitch))
            current_pose[2] = yaw
            current_poly = common_util.generate_poly(current_pose[0], current_pose[1], yaw, common_util.CUBE_SIZE_DICT[obj_name][2]/2.0, common_util.CUBE_SIZE_DICT[obj_name][0]/2.0)
            current_poly_list = list(current_poly.exterior.coords)
            self.object_list[obj_name] = current_poly_list
        self.object_num = len(self.object_list)
        self.generate_planning_scene_msg()
        self.generate_planning_scene_file()

    def import_obj_from_optitrack_forklift(self, obj_name_list):
        for obj_name, obj_index in zip(obj_name_list, range(len(obj_name_list))):
            data = rospy.wait_for_message('/vrpn_client_node/' + obj_name + '/pose', PoseStamped)
            
            current_pose = [0, 0, 0]
            current_pose[0] = data.pose.position.x
            current_pose[1] = data.pose.position.y
            q = (data.pose.orientation.x,
                    data.pose.orientation.y,
                    data.pose.orientation.z,
                    data.pose.orientation.w)
            (roll, pitch, yaw) = euler_from_quaternion(q) 
            print('obj_name:' + str(obj_name))
            print('yaw:' + str(yaw))
            print('roll:' + str(roll))
            print('pitch:' + str(pitch))
            current_pose[2] = yaw
            current_poly = common_util.generate_poly(current_pose[0], current_pose[1], yaw, common_util.CUBE_SIZE_DICT[obj_name][2]/2.0, common_util.CUBE_SIZE_DICT[obj_name][0]/2.0)
            current_poly_list = list(current_poly.exterior.coords)
            self.object_list[obj_name] = current_poly_list

    def export_objects_poly_list(self):
        return self.object_list

    def manipulation_action_done_cb(self, goal_state, result):
        #callback function for grasp planning action request

        print("manipulationaction returned")
        print(result)
        self.grasp_plan_result = result

    def move_arm_to_joint_pose_old_cb(self, goal_state, result):
        print("move_arm_to_joint_pose_old returned")

    def send_grasp_action(self, manipulation_scene_msg, next_picking_object_name, target_object_pose, last_target_object_name, object_type_name, rest_base_pose, is_synchronize):
        #prepare grasp action request and send action request

        client = actionlib.SimpleActionClient("manipulation_action", ManipulationAction)
        client.wait_for_server()
        goal = ManipulationGoal()
        print("next_picking_object:" + next_picking_object_name)
        goal.planning_scene = manipulation_scene_msg
        goal.target_object_name.data = next_picking_object_name
        goal.target_object_pose = target_object_pose
        goal.last_target_object_name.data = last_target_object_name
        goal.target_object_type.data = object_type_name
        goal.rest_base_pose = rest_base_pose
        goal.is_synchronize.data = is_synchronize
        client.send_goal(goal, done_cb=self.manipulation_action_done_cb)
        client.wait_for_result(rospy.Duration.from_sec(10.0))

    def generate_planning_scene_msg(self):
        #prepare the planning_scene_msg for grasp planning

        obj_index = 0
        for obj_name, obj  in self.object_list.iteritems():
            if obj_name == 'wall': continue
            scene_object = SceneObjectMsg()
            scene_object.object_type.data = "cube"
            scene_object.object_state.data = 0
            
            long_x = obj[0][0] - obj[len(obj)-1][0]
            long_y = obj[0][1] - obj[len(obj)-1][1]
            long_length = math.sqrt(long_x*long_x + long_y*long_y)

            short_x = obj[0][0] - obj[1][0]
            short_y = obj[0][1] - obj[1][1]
            short_length = math.sqrt(short_x*short_x + short_y*short_y)
            
            temp_length = 0
            if long_length < short_length:
                temp_length = long_length
                long_length = short_length
                short_length = temp_length

            center_x = 0
            center_y = 0
            for pt in obj:
                center_x += pt[0]
                center_y += pt[1]
            center_x = center_x / len(obj)
            center_y = center_y / len(obj)
            yaw = common_util.get_yaw_from_polygon(obj)
            print("yaw:"+str(yaw))

            rotation_matrix = np.array((
        (0,     -math.sin(yaw),     -math.cos(yaw), 0.0),
        (0,  math.cos(yaw),     -math.sin(yaw), 0.0),
        (1,     0, 0, 0.0),
        (                0.0,                 0.0,                 0.0, 1.0)
        ), dtype=np.float64)
            q = quaternion_from_matrix(rotation_matrix)
            #q = quaternion_from_euler(0, 0, yaw)
            #q = [0,0,0,1]
            qx = q[0]
            qy = q[1]
            qz = q[2]
            qw = q[3]
            print(q)
            short_length = 0.0376
            size = [short_length, short_length,long_length]
            position = [center_x, center_y, short_length / 2.0]
            quaternion = [qx, qy, qz, qw]
            # color = GAZEBO_COLORS[obj_index % len(GAZEBO_COLORS)]
            object_name = obj_name

            current_obj_pose = Pose()
            current_obj_2dpose = Pose2D()
            current_obj_pose.position.x = position[0]
            current_obj_pose.position.y = position[1]
            current_obj_pose.position.z = position[2]

            current_obj_pose.orientation.x = quaternion[0]
            current_obj_pose.orientation.y = quaternion[1]
            current_obj_pose.orientation.z = quaternion[2]
            current_obj_pose.orientation.w = quaternion[3]

            scene_object.object_pose = current_obj_pose
            current_obj_2dpose.x = position[0]
            current_obj_2dpose.y = position[1]
            current_obj_2dpose.theta = yaw
            scene_object.object_se2_pose = current_obj_2dpose

            scene_object.object_name.data = object_name
            scene_object.mesh_filename.data = " "
            scene_object.dx = size[0]
            scene_object.dy = size[1]
            scene_object.dz = size[2]
            self.obj_name_to_index_dict[obj_name] = obj_index
            self.planning_scene_msg.scene_object_list.append(scene_object)
            self.reserved_planning_scene_msg.scene_object_list.append(scene_object)
            obj_index += 1

    def generate_planning_scene_file(self):
        #prepare the planning_scene_msg for grasp planning
        scene_file = open("current_scene.txt", "w+")
        scene_file.write(str(len(self.object_list)) + "\n")
        num_objs = len(self.object_list) 
        obj_index = 0
        for obj_name, obj  in self.object_list.iteritems():
            if obj_name == 'wall': continue
            
            #write num of object vertices
            scene_file.write("4\n")
            scene_file.write(str(obj[0][0]) + " " + str(obj[0][1])+"\n")
            scene_file.write(str(obj[1][0]) + " " + str(obj[1][1])+"\n")
            scene_file.write(str(obj[2][0]) + " " + str(obj[2][1])+"\n")
            scene_file.write(str(obj[3][0]) + " " + str(obj[3][1])+"\n")

  

    def generate_obj_in_gazebo(self):
        obj_index = 0
        for obj_name, obj in self.object_list.iteritems():
            if obj_name == 'wall': continue
            scene_object = SceneObjectMsg()
            scene_object.object_type.data = "cube"
            scene_object.object_state.data = 0
            
            long_x = obj[0][0] - obj[len(obj)-1][0]
            long_y = obj[0][1] - obj[len(obj)-1][1]
            long_length = math.sqrt(long_x*long_x + long_y*long_y)

            short_x = obj[0][0] - obj[1][0]
            short_y = obj[0][1] - obj[1][1]
            short_length = math.sqrt(short_x*short_x + short_y*short_y)
            
            temp_length = 0
            if long_length < short_length:
                temp_length = long_length
                long_length = short_length
                short_length = temp_length

            center_x = 0
            center_y = 0
            for pt in obj:
                center_x += pt[0]
                center_y += pt[1]
            center_x = center_x / len(obj)
            center_y = center_y / len(obj)
            yaw = common_util.get_yaw_from_polygon(obj)
            print("yaw:"+str(yaw))

            rotation_matrix = np.array((
        (0,     -math.sin(yaw),     -math.cos(yaw), 0.0),
        (0,  math.cos(yaw),     -math.sin(yaw), 0.0),
        (1,     0, 0, 0.0),
        (                0.0,                 0.0,                 0.0, 1.0)
        ), dtype=np.float64)
            q = quaternion_from_matrix(rotation_matrix)
            #q = quaternion_from_euler(0, 0, yaw)
            #q = [0,0,0,1]
            qx = q[0]
            qy = q[1]
            qz = q[2]
            qw = q[3]
            print(q)
            short_length = 0.0376
            size = [short_length, short_length,long_length]
            position = [center_x, center_y, short_length / 2.0]
            quaternion = [qx, qy, qz, qw]
            # color = GAZEBO_COLORS[obj_index % len(GAZEBO_COLORS)]
            color = GAZEBO_COLORS[0]            
            object_name = obj_name
            common_util.spawnCuboid(size, position, quaternion, color, object_name)

            current_obj_pose = Pose()
            current_obj_2dpose = Pose2D()
            current_obj_pose.position.x = position[0]
            current_obj_pose.position.y = position[1]
            current_obj_pose.position.z = position[2]

            current_obj_pose.orientation.x = quaternion[0]
            current_obj_pose.orientation.y = quaternion[1]
            current_obj_pose.orientation.z = quaternion[2]
            current_obj_pose.orientation.w = quaternion[3]

            scene_object.object_pose = current_obj_pose
            current_obj_2dpose.x = position[0]
            current_obj_2dpose.y = position[1]
            current_obj_2dpose.theta = yaw
            scene_object.object_se2_pose = current_obj_2dpose

            scene_object.object_name.data = object_name
            scene_object.mesh_filename.data = " "
            scene_object.dx = size[0]
            scene_object.dy = size[1]
            scene_object.dz = size[2]
            self.obj_name_to_index_dict[obj_name] = obj_index
            self.planning_scene_msg.scene_object_list.append(scene_object)
            
            self.reserved_planning_scene_msg.scene_object_list.append(scene_object)
            obj_index += 1
        print("scene_object_list:")
        print(self.reserved_planning_scene_msg.scene_object_list)   

    def update_env_add(self, added_obj):
        print("add object " + added_obj)
        data = rospy.wait_for_message('/vrpn_client_node/' + added_obj + '/pose', PoseStamped)
            
        current_pose = [0, 0, 0]
        current_pose[0] = data.pose.position.x
        current_pose[1] = data.pose.position.y
        q = (data.pose.orientation.x,
                data.pose.orientation.y,
                data.pose.orientation.z,
                data.pose.orientation.w)
        (roll, pitch, yaw) = euler_from_quaternion(q) 
        print('obj_name:' + str(added_obj))
        print('yaw:' + str(yaw))
        print('roll:' + str(roll))
        print('pitch:' + str(pitch))
        current_pose[2] = yaw
        current_poly = common_util.generate_poly(current_pose[0], current_pose[1], yaw, common_util.CUBE_SIZE_DICT[added_obj][2]/2.0, common_util.CUBE_SIZE_DICT[added_obj][0]/2.0)
        current_poly_list = list(current_poly.exterior.coords)
        self.object_list[added_obj] = current_poly_list

    def update_env_del(self, deleted_obj):
        print("delete object " + deleted_obj)
        del self.object_list[deleted_obj]

    def update_env(self, deleted_obj):
        # when deleted_obj is removed from the scene, this function updates the self.object_list and self.planning_scene_msg

        # print(deleted_obj) 
        # deleted_obj_index = self.object_list.values().index(deleted_obj)        
        # # print(self.object_list)
        # deleted_obj_key = self.object_list.keys()[deleted_obj_index]
        # self.object_list.pop(deleted_obj_key)
        # print("update_env:" + str(deleted_obj_index))
        # print("planning_scene_msg size:" + str(len(self.planning_scene_msg.scene_object_list)))
        # deleted_obj_msg = self.planning_scene_msg.scene_object_list[deleted_obj_index]
        # print("planning_scene_msg size:" + str(len(self.planning_scene_msg.scene_object_list)))
        # self.planning_scene_msg.scene_object_list.remove(deleted_obj_msg)
        # print("planning_scene_msg size:" + str(len(self.planning_scene_msg.scene_object_list)))
        # # raw_input("wait")

        print("delete", deleted_obj) 
        self.object_list.pop(deleted_obj)
        print("planning_scene_msg size:" + str(len(self.planning_scene_msg.scene_object_list)))
        for index, obj in enumerate(self.planning_scene_msg.scene_object_list):
            if obj.object_name.data == deleted_obj:
                deleted_index = index
                break
        self.planning_scene_msg.scene_object_list.pop(deleted_index)
        print("planning_scene_msg size:" + str(len(self.planning_scene_msg.scene_object_list)))


    def move_to_target(self, youbot_name, target_pose):
        # works under gazebo and real world env
        # INPUT:
            # youbot_name: the controlled youbot's name (ex: youbot_0)
            # target_pose: the target pose in global coordinate
                    # type: geometry_msgs/Pose
        current_pos_2d = self.base_controller.get_youbot_base_pose2d()
        print("current_pos_2d")
        print(current_pos_2d)
        target_pos_2d = [0, 0, 0]
        target_pos_2d[0] = target_pose.position.x
        target_pos_2d[1] = target_pose.position.y
        q = (target_pose.orientation.x,
             target_pose.orientation.y,
             target_pose.orientation.z,
             target_pose.orientation.w)
        (_, _, yaw) = euler_from_quaternion(q)
        target_pos_2d[2] = yaw
        
        start_pos = (current_pos_2d[0], current_pos_2d[1])
        goal_pos = (target_pos_2d[0], target_pos_2d[1])
        obstacles = self.object_list.values()
        start_heading = current_pos_2d[2]
        goal_heading = target_pos_2d[2]
        print("start:")
        print(start_pos, start_heading)
        print("goal:")
        print(goal_pos, goal_heading)
        start_time = time.time()
        path_with_heading, g, large_g = base_util.vg_find_combined_path(start_pos, goal_pos, start_heading, goal_heading, obstacles)
        # path_with_heading, g = base_util.vg_find_large_path(start_pos, goal_pos, start_heading, goal_heading, obstacles)
        # path_with_heading, g = base_util.vg_find_small_path(start_pos, goal_pos, start_heading, goal_heading, obstacles)
        print("time: " + str(time.time() - start_time))
        print("path:")
        print(path_with_heading)
        
        # base_util.plot_vg_path(obstacles, path_with_heading, g, large_g)
        # base_util.plot_vg_path(obstacles, path_with_heading, g)

        self.base_controller.execute_path_vel_pub(path_with_heading, True)

    def thread_move_to_target(self, youbot_name, target_pose):
        # works under gazebo and real world env
        # INPUT:
            # youbot_name: the controlled youbot's name (ex: youbot_0)
            # target_pose: the target pose in global coordinate
                    # type: geometry_msgs/Pose

        base_controller = base_util.BaseController(youbot_name, self.mode)
        current_pos_2d = base_controller.get_youbot_base_pose2d()
        print("current_pos_2d")
        print(current_pos_2d)
        target_pos_2d = [0, 0, 0]
        target_pos_2d[0] = target_pose.position.x
        target_pos_2d[1] = target_pose.position.y
        q = (target_pose.orientation.x,
             target_pose.orientation.y,
             target_pose.orientation.z,
             target_pose.orientation.w)
        (_, _, yaw) = euler_from_quaternion(q)
        target_pos_2d[2] = yaw
        
        start_pos = (current_pos_2d[0], current_pos_2d[1])
        goal_pos = (target_pos_2d[0], target_pos_2d[1])
        obstacles = self.object_list.values()
        start_heading = current_pos_2d[2]
        goal_heading = target_pos_2d[2]
        print("start:")
        print(start_pos, start_heading)
        print("goal:")
        print(goal_pos, goal_heading)
        start_time = time.time()
        path_with_heading, g = base_util.vg_find_large_path(start_pos, goal_pos, start_heading, goal_heading, obstacles)
        print("time: " + str(time.time() - start_time))
        print("path:")
        print(path_with_heading)
        
        base_controller.execute_path_vel_pub(path_with_heading, False)


    def multi_move_to_target(self, youbot_names, target_poses):
        # TODO: assume vg_find_large_path for now
        # works under gazebo and real world env
        # INPUT:
            # youbot_name: the controlled youbot's name (ex: youbot_0)
            # target_pose: the target pose in global coordinate
                    # type: geometry_msgs/Pose
        signal.signal(signal.SIGINT, ctrl_c_handler)
        obstacles = self.object_list.values()
        mbc = base_util.MultiBaseController(youbot_names, self.mode, obstacles)
        threads = {}

        for name in youbot_names:
            print('===== ' + name + ' =====')
            current_pos_2d = mbc.base_controllers[name].get_youbot_base_pose2d()
            print("current_pos_2d")
            print(current_pos_2d)
            target_pos_2d = [0, 0, 0]
            target_pos_2d[0] = target_poses[name].position.x
            target_pos_2d[1] = target_poses[name].position.y
            q = (target_poses[name].orientation.x,
                target_poses[name].orientation.y,
                target_poses[name].orientation.z,
                target_poses[name].orientation.w)
            (_, _, yaw) = euler_from_quaternion(q)
            target_pos_2d[2] = yaw
        
            start_pos = (current_pos_2d[0], current_pos_2d[1])
            goal_pos = (target_pos_2d[0], target_pos_2d[1])
            
            start_heading = current_pos_2d[2]
            goal_heading = target_pos_2d[2]
            print("start:")
            print(start_pos, start_heading)
            print("goal:")
            print(goal_pos, goal_heading)
            start_time = time.time()
            path_with_heading, g = base_util.vg_find_large_path(start_pos, goal_pos, start_heading, goal_heading, obstacles)
            print("time: " + str(time.time() - start_time))
            print("path:")
            print(path_with_heading)

            # base_util.plot_vg_path(obstacles, path_with_heading, g, g)
            
            thr = threading.Thread(target=mbc.base_controllers[name].execute_path_vel_pub, args=(path_with_heading,)) 
            threads[name] = thr

        for thr_name in threads:
            threads[thr_name].start()
        r = rospy.Rate(30)
        r.sleep()
        while not rospy.is_shutdown() and not mbc.all_completed:
            mbc.publish_vels()
            r.sleep()
        print("All YouBots have reached target positions!")

    def move_to_target_2d(self, youbot_name, target_pos_2d):
        # works under gazebo and real world env
        # INPUT:
            # youbot_name: the controlled youbot's name (ex: youbot_0)
            # target_pos_2d: the target pose in global coordinate
                    # type: [x,y,theta]
        base_controller = base_util.BaseController(youbot_name, self.mode)
        current_pos_2d = base_controller.get_youbot_base_pose2d()
        print("current_pos_2d")
        print(current_pos_2d)
        
        start_pos = (current_pos_2d[0], current_pos_2d[1])
        goal_pos = (target_pos_2d[0], target_pos_2d[1])
        obstacles = self.object_list.values()
        print("start:")
        print(start_pos)
        print("goal:")
        print(goal_pos)
        path, g = base_util.vg_find_path(start_pos, goal_pos, obstacles)
        print(path)
        start_heading = current_pos_2d[2]
        goal_heading = target_pos_2d[2]
        path_with_heading = base_util.add_orientation(path, start_heading, goal_heading)

        # base_util.plot_vg_path(obstacles, path_with_heading, g)

        base_controller.execute_path(youbot_name, path_with_heading, "youbot_base/move")

    def move_arm_to_joint_pose_old(self, youbot_name, joint_target):
        client = actionlib.SimpleActionClient(youbot_name + "/arm_1/to_joint_pose/plan", MoveToJointPoseAction)
        print("wait for server")
        client.wait_for_server()
        goal = MoveToJointPoseGoal()
        goal.pose_is_relative = False
        goal.pose.q1 =  joint_target[0] + arm_util.JOINT_OFFSET[0]
        goal.pose.q2 =  joint_target[1] + arm_util.JOINT_OFFSET[1]
        goal.pose.q3 =  joint_target[2]+ arm_util.JOINT_OFFSET[2]
        goal.pose.q4 =  joint_target[3]+ arm_util.JOINT_OFFSET[3]
        goal.pose.q5 =  joint_target[4]+ arm_util.JOINT_OFFSET[4]
        
        client.send_goal(goal, done_cb=self.move_arm_to_joint_pose_old_cb)
        client.wait_for_result(rospy.Duration.from_sec(10.0))

    def move_arm_to_joint_pose_old_direct(self, youbot_name, joint_target):
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
        
        client.send_goal(goal, done_cb=self.move_arm_to_joint_pose_old_cb)
        client.wait_for_result(rospy.Duration.from_sec(10.0))
    # joint_target in the range of [0, 5]
    def move_arm_to_joint_pose(self, youbot_name, joint_target):
        # move youbot_arm to target joint_pose
        #INPUT: 
        #  joint_target: [q1,q2,q3,q4,q5], in the range between arm_util.MIN_JOINT_POS and arm_util.MAX_JOINT_POS

        physicsClient = p.connect(p.DIRECT)#or p.DIRECT for non-graphical version
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
        p.setGravity(0,0,-10)
        prmstar_planner = None
        my_path = os.path.abspath(os.path.dirname(__file__))
        my_path = os.path.join(my_path, "../../../luh_youbot_description/robots/youbot_0.urdf")
        print(my_path)
        if self.prmstar_planner == None:
            self.prmstar_planner = prmstar.PRMStarPlanner(p, my_path)
        else:
            self.prmstar_planner.p_client = p
        is_target_in_collision = False
        is_target_in_collision = self.prmstar_planner.check_pose_collision(joint_target)
        if is_target_in_collision == True:
            return False
        else:
            self.prmstar_planner.remove_obstacles()
            self.prmstar_planner.import_obstacles(self.object_list.values())
            base_controller = base_util.BaseController(youbot_name, self.mode)
            current_pos_2d = base_controller.get_youbot_base_pose()
            robot_position = [current_pos_2d.position.x, current_pos_2d.position.y, current_pos_2d.position.z]
            robot_orientation = [current_pos_2d.orientation.x, current_pos_2d.orientation.y, current_pos_2d.orientation.z, current_pos_2d.orientation.w]
            self.prmstar_planner.set_robot_pose(robot_position, robot_orientation)

            arm_controller = arm_util.ArmController(youbot_name, self.mode)
            start = arm_controller.get_current_joint_pos()
            [final_path, final_cost] = self.prmstar_planner.path_plan(tuple(start), tuple(joint_target))
            arm_util.execute_path(youbot_name, final_path)
            return True

    def move_arm_to_joint_pose_direct(self, youbot_name, joint_target):
        arm_controller = arm_util.ArmController(youbot_name, self.mode)
        start = arm_controller.get_current_joint_pos()
        [final_path, final_cost] = self.prmstar_planner.direct_path_slow(tuple(start), tuple(joint_target))
        arm_util.execute_path(youbot_name, final_path)

    def pick_object(self, youbot_name, pick_joint_value, pre_pick_joint_value):
        #Given picking joint configuration, executes the goto following steps:
        # 1, open gripper
        # 2, move arm to pre_pick_joint_value
        # 3, move arm to pick_joint_value
        # 4, close gripper
        # 5, move back to pre_pick_joint_value

        physicsClient = p.connect(p.DIRECT)#or p.GUI for graphical version
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
        p.setGravity(0,0,-10)
        if self.prmstar_planner is None:
            my_path = os.path.abspath(os.path.dirname(__file__))
            my_path = os.path.join(my_path, "../../../luh_youbot_description/robots/youbot_0.urdf")
            print(my_path)
            self.prmstar_planner = prmstar.PRMStarPlanner(p, my_path)
        else:
            self.prmstar_planner.p_client = p
        #prmstar.build_roadmap()

        self.prmstar_planner.remove_obstacles()
        self.prmstar_planner.import_obstacles(self.object_list.values())
        base_controller = base_util.BaseController(youbot_name, self.mode)
        current_pos_2d = base_controller.get_youbot_base_pose()
        robot_position = [current_pos_2d.position.x, current_pos_2d.position.y, current_pos_2d.position.z]
        robot_orientation = [current_pos_2d.orientation.x, current_pos_2d.orientation.y, current_pos_2d.orientation.z, current_pos_2d.orientation.w]
        self.prmstar_planner.set_robot_pose(robot_position, robot_orientation)

        arm_controller = arm_util.ArmController(youbot_name, self.mode)
        start = arm_controller.get_current_joint_pos()
        if self.mode == 0 or self.mode == 1:
            pick_joint_value[4] = -pick_joint_value[4]
            pre_pick_joint_value[4] = -pre_pick_joint_value[4]

        if self.mode == 1:
            if pick_joint_value[4] + 1.57 > arm_util.MAX_JOINT_POS[4]:
                pick_joint_value[4] -= 1.57
                pre_pick_joint_value[4] -= 1.57
            else:
                pick_joint_value[4] += 1.57
                pre_pick_joint_value[4] += 1.57

            

        print('pick_joint_value:')
        print(pick_joint_value)
        arm_util.subtract_offset(pick_joint_value)
        arm_util.subtract_offset(pre_pick_joint_value)

        #plan and move arm to pre_pick_pos
        [final_path, final_cost] = self.prmstar_planner.path_plan(tuple(start), tuple(pre_pick_joint_value))
        if self.mode == 0:
            arm_util.set_gripper_width(youbot_name, 0.06, self.mode)
        else:
            arm_util.set_gripper_width(youbot_name, 0.0, self.mode)
        arm_util.execute_path(youbot_name, final_path)

        print("moved to the pre_pick pose")
        #directly move arm to pick_pos
        arm_controller = arm_util.ArmController(youbot_name, self.mode)
        start = arm_controller.get_current_joint_pos()
        [final_path, final_cost] = self.prmstar_planner.direct_path(tuple(start), tuple(pick_joint_value))
        print("before execution waiting.....")
        arm_util.execute_path(youbot_name, final_path)
        print("moved to the pick pose")
        if self.mode == 0:
            arm_util.set_gripper_width(youbot_name, 0.0, self.mode)
            rospy.sleep(rospy.Duration.from_sec(5.0))
        else:
            arm_util.set_gripper_width(youbot_name, 0.06, self.mode)
            rospy.sleep(rospy.Duration.from_sec(2.5))



        #directly retract arm to pre_pick_pos
        arm_controller = arm_util.ArmController(youbot_name, self.mode)
        start = arm_controller.get_current_joint_pos()
        [final_path, final_cost] = self.prmstar_planner.direct_path(tuple(start), tuple(pre_pick_joint_value))
        arm_util.execute_path(youbot_name, final_path)
        print("moved back to the pre_pick pose")

    def pick_object_old(self, youbot_name, pick_joint_value, pre_pick_joint_value):
        #Given picking joint configuration, executes the goto following steps:
        # 1, open gripper
        # 2, move arm to pre_pick_joint_value
        # 3, move arm to pick_joint_value
        # 4, close gripper
        # 5, move back to pre_pick_joint_value

        physicsClient = p.connect(p.DIRECT)#or p.GUI for graphical version
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
        p.setGravity(0,0,-10)
        prmstar_planner = None
        my_path = os.path.abspath(os.path.dirname(__file__))
        my_path = os.path.join(my_path, "../../../luh_youbot_description/robots/youbot_0.urdf")
        print(my_path)
        # prmstar_planner = prmstar.PRMStarPlanner(p, my_path)
        #prmstar.build_roadmap()

        # prmstar_planner.remove_obstacles()
        # prmstar_planner.import_obstacles(self.object_list.values())
        base_controller = base_util.BaseController(youbot_name, self.mode)
        current_pos_2d = base_controller.get_youbot_base_pose()
        robot_position = [current_pos_2d.position.x, current_pos_2d.position.y, current_pos_2d.position.z]
        robot_orientation = [current_pos_2d.orientation.x, current_pos_2d.orientation.y, current_pos_2d.orientation.z, current_pos_2d.orientation.w]
        # prmstar_planner.set_robot_pose(robot_position, robot_orientation)

        arm_controller = arm_util.ArmController(youbot_name, self.mode)
        start = arm_controller.get_current_joint_pos()
        if self.mode == 0 or self.mode == 1:
            pick_joint_value[4] = -pick_joint_value[4]
            pre_pick_joint_value[4] = -pre_pick_joint_value[4]

        if self.mode == 1:
            if pick_joint_value[4] + 1.57 > arm_util.MAX_JOINT_POS[4]:
                pick_joint_value[4] -= 1.57
                pre_pick_joint_value[4] -= 1.57
            else:
                pick_joint_value[4] += 1.57
                pre_pick_joint_value[4] += 1.57

            

        print('pick_joint_value:')
        print(pick_joint_value)
        arm_util.subtract_offset(pick_joint_value)
        arm_util.subtract_offset(pre_pick_joint_value)

        #plan and move arm to pre_pick_pos
        # [final_path, final_cost] = prmstar_planner.path_plan(tuple(start), tuple(pre_pick_joint_value))
        if self.mode == 0:
            arm_util.set_gripper_width("", 0.06, self.mode)
        else:
            arm_util.set_gripper_width("", 0.0, self.mode)
        
       
        # self.move_arm_to_joint_pose_old("youbot_0", pre_pick_joint_value)
#DEBUG        
        self.move_arm_to_joint_pose_old("", pre_pick_joint_value)
#DEBUG        
        
        # arm_util.execute_path(youbot_name, final_path)

        print("moved to the pre_pick pose")
        #directly move arm to pick_pos
        arm_controller = arm_util.ArmController("", self.mode)
        start = arm_controller.get_current_joint_pos()
        # [final_path, final_cost] = prmstar_planner.direct_path(tuple(start), tuple(pick_joint_value))
        print("before execution waiting.....")
        # arm_util.execute_path(youbot_name, final_path)


#        self.move_arm_to_joint_pose_old_direct("youbot_0", pick_joint_value)
 #DEBUG
        self.move_arm_to_joint_pose_old_direct("", pick_joint_value)
 #DEBUG       
        
        print("moved to the pick pose")
        if self.mode == 0:
            arm_util.set_gripper_width("", 0.0, self.mode)
            rospy.sleep(rospy.Duration.from_sec(5.0))
        else:
            arm_util.set_gripper_width("", 0.06, self.mode)
            rospy.sleep(rospy.Duration.from_sec(2.5))



        #directly retract arm to pre_pick_pos
        arm_controller = arm_util.ArmController("", self.mode)
        start = arm_controller.get_current_joint_pos()
        # [final_path, final_cost] = prmstar_planner.direct_path(tuple(start), tuple(pre_pick_joint_value))
        # arm_util.execute_path(youbot_name, final_path)


        #self.move_arm_to_joint_pose_old_direct("youbot_0", pre_pick_joint_value)
        #DEBUG
        self.move_arm_to_joint_pose_old_direct("", pre_pick_joint_value)        
        #DEBUG


        print("moved back to the pre_pick pose")
    def drop_object(self, youbot_name, obj_name):
        if self.mode == 0:
            arm_util.set_gripper_width(youbot_name, 0.06, self.mode)

            offset = float(obj_name.split('_')[1]) / 10.0

            deserted_pose = Pose()
            deserted_pose.position.x = 0
            deserted_pose.position.y = -10 + offset
            deserted_pose.position.z = 0.1
            deserted_pose.orientation.x = 0
            deserted_pose.orientation.y = 0
            deserted_pose.orientation.z = 0
            deserted_pose.orientation.w = 1
            rospy.sleep(rospy.Duration(3, 0))
            common_util.set_obj_pose(obj_name, deserted_pose)
            rospy.sleep(rospy.Duration(1, 0))
            common_util.set_obj_pose(obj_name, deserted_pose)
            print('drop object', obj_name, deserted_pose)
        else:
            arm_util.set_gripper_width(youbot_name, 0.0, self.mode)

    def forklift_done_cb(self, goal_state, result):
        print("Fork Lift GoToPosition returned")
        print(result)

    def set_forklift_position(self, youbot_name, position, position_reached=None, position_error=None):
        #for controlling real robot 
        print("go to " + str(position))
        client = actionlib.SimpleActionClient("goToPosAction", GoToPositionAction)
        client.wait_for_server()
        goal = GoToPositionGoal()
        goal.goal_position_in_meter = position
        if position_reached:
            goal.position_reached = position_reached
        if position_error:
            goal.position_error = position_error
        client.send_goal(goal, done_cb=self.forklift_done_cb)
        client.wait_for_result(rospy.Duration.from_sec(20.0))
        return client.get_result()

    def set_forklift_position_gazebo(self, youbot_name, position, position_reached=None, position_error=None):
        # for controlling robot in gazebo

        from std_msgs.msg import Float64
        fork_pub = rospy.Publisher('/forklift/forklift_controller/command', Float64, queue_size=1)
        rate = rospy.Rate(5)
        rate.sleep()
        fork_pub.publish(position)
        rate.sleep()

    def plot_reachibility(self):
        # function for testing the reachibility of the youbot arm

        physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
        p.setGravity(0,0,-10)
        prmstar_planner = None
        my_path = os.path.abspath(os.path.dirname(__file__))
        my_path = os.path.join(my_path, "../../../luh_youbot_description/robots/youbot_0.urdf")
        print(my_path)
        prmstar_planner = prmstar.PRMStarPlanner(p, my_path)

        base_controller = base_util.BaseController("youbot_0", self.mode)
        current_pos_2d = base_controller.get_youbot_base_pose()
        robot_position = [current_pos_2d.position.x, current_pos_2d.position.y, current_pos_2d.position.z]
        robot_orientation = [current_pos_2d.orientation.x, current_pos_2d.orientation.y, current_pos_2d.orientation.z, current_pos_2d.orientation.w]
        
        prmstar_planner.generate_reachibility(robot_position, robot_orientation)

    def build_arm_roadmap(self):
        #re-build the PRM roadmap for youbot arm
        physicsClient = p.connect(p.DIRECT)#or p.GUI for graphical version
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
        p.setGravity(0,0,-10)
        prmstar_planner = None
        my_path = os.path.abspath(os.path.dirname(__file__))
        my_path = os.path.join(my_path, "../../../luh_youbot_description/robots/youbot_0.urdf")

        prmstar_planner = prmstar.PRMStarPlanner(p, my_path, False)
        prmstar_planner.build_roadmap()