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
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String
from std_msgs.msg import UInt8, Float64
import arcl_youbot_planner.arm_planner.astar 
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

WALL = [(2.8, -0.3), (0.7, -0.3), (0.7, 0.0), (2.5, 0.0), (2.5, 5.0), (-2.5, 5.0), (-2.5, 0.0), (-0.7, 0.0), (-0.7, -0.3), (-2.8, -0.3), (-2.8, 5.3), (2.8, 5.3), (2.8, 0.0)]


class YoubotEnvironment(): 
    #example object_list: 
    #[[(1, 1), (2, 1.5), (1, 2)],
    #            [(3, 2), (4, 2), (4, 4), (3, 4)],
    #             [(4, 3), (4.5, 1), (5, 3)]]    
    
    def __init__(self, x_min, x_max, y_min, y_max):
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.object_list = []
        self.planning_scene_msg = PlanningSceneMsg()
        self.mode = 0

    # filename: ffabsolute path for the environment file
    def import_obj_from_file(self, filename):
        f = open(filename, 'r')
        self.object_num = int(f.readline())
        for _ in range(self.object_num):
            vertex_num = int(f.readline())
            current_obj = []
            for _ in range(vertex_num):
                point = string.split(f.readline())
                point[0] = float(point[0])
                point[1] = float(point[1])
                current_obj.append((point[0], point[1]))
            self.object_list.append(current_obj)

        self.object_list.append(WALL)

        print len(self.object_list)

    # filename: ffabsolute path for the environment file
    def export_obj_to_file(self, filename):
        f = open(filename, 'w')
        f.write(str(self.object_num)+'\n')
        for obj in self.object_list[:-1]:
            f.write(str(len(obj)-1)+'\n')
            for v in obj[:-1]:
                f.write(str(v[0]) + " " + str(v[1])+'\n')

    def import_obj_from_list(self, object_list):
        self.object_list = object_list
    

    def create_scene(self, object_number, cluster_number):
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
            is_valid = False
            random.seed()
            while not is_valid:  
                is_valid = True
                center_x = random.random() * (x_max - x_min) + x_min
                center_y = random.random() * (y_max - y_min) + y_min
                yaw = random.random() * 3.14159

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
    
        list_created_objs = []
        for obj in created_objs:
            list_created_objs.append(list(obj.exterior.coords))

        self.import_obj_from_list(list_created_objs)
        self.object_num = len(self.object_list)

        self.object_list.append(WALL)

    def import_obj_from_optitrack():
        pass

    def manipulation_action_done_cb(self, goal_state, result):
        print("manipulationaction returned")
        print(result)
        self.grasp_plan_result = result

    def send_grasp_action(self, manipulation_scene_msg, next_picking_object_name, target_object_pose, last_target_object_name, object_type_name, rest_base_pose, is_synchronize):
        client = actionlib.SimpleActionClient("manipulation_action", ManipulationAction)
        client.wait_for_server()
        goal = ManipulationGoal()
        goal.planning_scene = manipulation_scene_msg
        goal.target_object_name.data = next_picking_object_name
        goal.target_object_pose = target_object_pose
        goal.last_target_object_name.data = last_target_object_name
        goal.target_object_type.data = object_type_name
        goal.rest_base_pose = rest_base_pose
        goal.is_synchronize.data = is_synchronize
        client.send_goal(goal, done_cb=self.manipulation_action_done_cb)
        client.wait_for_result(rospy.Duration.from_sec(10.0))

    def generate_obj_in_gazebo(self):
        for obj, obj_index in zip(self.object_list[:-1], range(len(self.object_list) - 1)):
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
            color = GAZEBO_COLORS[obj_index % len(GAZEBO_COLORS)]
            object_name = "obj_" + str(obj_index)
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
            self.planning_scene_msg.scene_object_list.append(scene_object)

    def update_env(self, deleted_obj):
        deleted_obj_index = self.object_list.index(deleted_obj)        
        self.object_list.remove(deleted_obj)
        deleted_obj_msg = self.planning_scene_msg.scene_object_list[deleted_obj_index]
        print("planning_scene_msg size:" + str(len(self.planning_scene_msg.scene_object_list)))
        self.planning_scene_msg.scene_object_list.remove(deleted_obj_msg)
        print("planning_scene_msg size:" + str(len(self.planning_scene_msg.scene_object_list)))

    def move_to_target(self, youbot_name, target_pose):
        current_pos_2d = base_util.get_youbot_base_pose2d(youbot_name, self.mode)
        print("current_pos_2d")
        print(current_pos_2d)
        target_pos_2d = [0, 0, 0]
        target_pos_2d[0] = target_pose.position.x
        target_pos_2d[1] = target_pose.position.y
        q = (target_pose.orientation.x,
             target_pose.orientation.y,
             target_pose.orientation.z,
             target_pose.orientation.w)
        (roll, pitch, yaw) = euler_from_quaternion(q)
        target_pos_2d[2] = yaw
        
        start_pos = (current_pos_2d[0], current_pos_2d[1])
        goal_pos = (target_pos_2d[0], target_pos_2d[1])
        obstacles = self.object_list
        start_heading = current_pos_2d[2]
        goal_heading = target_pos_2d[2]
        print("start:")
        print(start_pos, start_heading)
        print("goal:")
        print(goal_pos, goal_heading)
        path_with_heading, g = base_util.vg_find_path(start_pos, goal_pos, start_heading, goal_heading, obstacles)
        print("path:")
        print(path_with_heading)
        
        # path_with_heading = vg.vg_youbot_path.add_orientation(path, start_heading, goal_heading)
        # path_with_heading = base_util.add_orientation(path, start_heading, goal_heading)

        base_util.plot_vg_path(obstacles, path_with_heading, g)

        # base_util.execute_path(youbot_name, path_with_heading, "/youbot_base/move")
        base_util.execute_path_vel_pub(youbot_name, path_with_heading, self.mode)
        # call base planner
        # execute_path

    def move_to_target_2d(self, youbot_name, target_pos_2d):
        current_pos_2d = base_util.get_youbot_base_pose2d(youbot_name, self.mode)
        print("current_pos_2d")
        print(current_pos_2d)
        
        start_pos = (current_pos_2d[0], current_pos_2d[1])
        goal_pos = (target_pos_2d[0], target_pos_2d[1])
        obstacles = self.object_list
        # path, g = vg.vg_youbot_path.vg_find_path(start_pos, goal_pos, obstacles)
        print("start:")
        print(start_pos)
        print("goal:")
        print(goal_pos)
        path, g = base_util.vg_find_path(start_pos, goal_pos, obstacles)
        print(path)
        start_heading = current_pos_2d[2]
        goal_heading = target_pos_2d[2]
        # path_with_heading = vg.vg_youbot_path.add_orientation(path, start_heading, goal_heading)
        path_with_heading = base_util.add_orientation(path, start_heading, goal_heading)

        base_util.plot_vg_path(obstacles, path_with_heading, g)

        base_util.execute_path(youbot_name, path_with_heading, "youbot_base/move")
        # call base planner
        # execute_path

    def pick_object(self, youbot_name, pick_joint_value, pre_pick_joint_value):
        physicsClient = p.connect(p.DIRECT)#or p.DIRECT for non-graphical version
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
        p.setGravity(0,0,-10)
        prmstar_planner = None
        my_path = os.path.abspath(os.path.dirname(__file__))
        my_path = os.path.join(my_path, "../../../luh_youbot_description/robots/youbot_0.urdf")
        print(my_path)
        prmstar_planner = prmstar.PRMStarPlanner(p, my_path)
        #prmstar.build_roadmap()

        prmstar_planner.import_obstacles(self.object_list)
        robot_position, robot_orientation = base_util.get_youbot_base_pose(youbot_name)
        prmstar_planner.set_robot_pose(robot_position, robot_orientation)

        start = arm_util.get_current_joint_pos(youbot_name)

        pick_joint_value[4] = -pick_joint_value[4]
        pre_pick_joint_value[4] = -pre_pick_joint_value[4]
        arm_util.subtract_offset(pick_joint_value)
        arm_util.subtract_offset(pre_pick_joint_value)

        #plan and move arm to pre_pick_pos
        [final_path, final_cost] = prmstar_planner.path_plan(tuple(start), tuple(pre_pick_joint_value))

        arm_util.execute_path(youbot_name, final_path)

        arm_util.set_gripper_width("youbot_0", 0.068)
        print("moved to the pre_pick pose")
        #directly move arm to pick_pos
        start = arm_util.get_current_joint_pos(youbot_name)
        [final_path, final_cost] = prmstar_planner.direct_path(tuple(start), tuple(pick_joint_value))
        
        arm_util.execute_path(youbot_name, final_path)
        print("moved to the pick pose")

        arm_util.set_gripper_width("youbot_0", 0.01)
        rospy.sleep(rospy.Duration.from_sec(10.0))

        #directly retract arm to pre_pick_pos
        start = arm_util.get_current_joint_pos(youbot_name)
        [final_path, final_cost] = prmstar_planner.direct_path(tuple(start), tuple(pre_pick_joint_value))
        arm_util.execute_path(youbot_name, final_path)
        print("moved back to the pre_pick pose")

    def forklift_done_cb(self, goal_state, result):
        print("Fork Lift GoToPosition returned")
        print(result)

    def set_forklift_position(self, youbot_name, position, position_reached=None, position_error=None):
        client = actionlib.SimpleActionClient(youbot_name + "ForkLift", GoToPositionAction)
        client.wait_for_server()
        goal = GoToPositionGoal()
        goal.goal_position_in_meter = position
        if position_reached:
            goal.position_reached = position_reached
        if position_error:
            goal.position_error = position_error
        client.send_goal(goal, done_cb=self.forklift_done_cb)
        client.wait_for_result(rospy.Duration.from_sec(10.0))
        return client.get_result()
