#!/usr/bin/env python
import rospy
import math
import pybullet as p
import time
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
from std_msgs.msg import UInt8
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


SCALE = 400.0
OFFSET = 2.5
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
        self.env_boundary = []
        self.planning_scene_msg = PlanningSceneMsg()

    # filename: ffabsolute path for the environment file
    def import_obj_from_file(self, filename):
        f = open(filename, 'r')
        self.robot_radius = float(f.readline())
        self.object_num = int(f.readline())
        for o in range(self.object_num):
            vertex_num = int(f.readline())
            current_obj = []
            for v in range(vertex_num):
                point = string.split(f.readline())
                point[0] = float(point[0]) / SCALE - OFFSET
                point[1] = float(point[1]) / SCALE 
                current_obj.append((point[0], point[1]))
            self.object_list.append(current_obj)
        boundary_vertex_num = int(f.readline())
        for v in range(boundary_vertex_num):
            point = string.split(f.readline())
            point[0] = float(point[0])
            point[1] = float(point[1])
            self.env_boundary.append((point[0], point[1]))

        print self.object_list

    def import_obj_from_list(self, object_list):
        self.object_list = object_list
    

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
        for obj, obj_index in zip(self.object_list, range(len(self.object_list))):
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
            color = GAZEBO_COLORS[obj_index]
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

    def move_to_target(self, youbot_name, target_pose):
        current_pos_2d = base_util.get_youbot_base_pose2d(youbot_name)
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
        # path, g = vg.vg_youbot_path.vg_find_path(start_pos, goal_pos, obstacles)
        print("start:")
        print(start_pos)
        print("goal:")
        print(goal_pos)
        start_heading = current_pos_2d[2]
        goal_heading = target_pos_2d[2]
        path_with_heading, g = base_util.vg_find_path(start_pos, goal_pos, start_heading, goal_heading, obstacles)
        print(path_with_heading)
        
        # path_with_heading = vg.vg_youbot_path.add_orientation(path, start_heading, goal_heading)
        # path_with_heading = base_util.add_orientation(path, start_heading, goal_heading)

        base_util.plot_vg_path(obstacles, path_with_heading, g)

        base_util.execute_path(path_with_heading, youbot_name + "_base/move")
        # call base planner
        # execute_path

    def move_to_target_2d(self, youbot_name, target_pos_2d):
        current_pos_2d = base_util.get_youbot_base_pose2d(youbot_name)
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

        base_util.execute_path(path_with_heading, youbot_name + "_base/move")
        # call base planner
        # execute_path

    def pick_object(self, youbot_name, pick_joint_value, pre_pick_joint_value):
        physicsClient = p.connect(p.DIRECT)#or p.DIRECT for non-graphical version
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
        p.setGravity(0,0,-10)
        prmstar_planner = None
        prmstar_planner = prmstar.PRMStarPlanner(p, "/home/wei/catkin_youbot_ws/src/luh_youbot_description/robots/youbot_0.urdf")
        #prmstar.build_roadmap()

        prmstar_planner.import_obstacles(self.object_list)
        prmstar_planner.set_robot_pose(base_util.get_youbot_base_pose(youbot_name))

        start = arm_util.get_current_joint_pos()

        pick_joint_value[4] = -pick_joint_value[4]
        pre_pick_joint_value[4] = -pre_pick_joint_value[4]
        arm_util.subtract_offset(pick_joint_value)
        arm_util.subtract_offset(pre_pick_joint_value)

        #plan and move arm to pre_pick_pos
        [final_path, final_cost] = prmstar_planner.path_plan(tuple(start), tuple(pre_pick_joint_value))

        arm_util.execute_path(final_path, '/arm_1/follow_joint_trajectory')

        arm_util.set_gripper_width("youbot", 0.068)

        #directly move arm to pick_pos
        start = arm_util.get_current_joint_pos()
        [final_path, final_cost] = prmstar_planner.direct_path(tuple(start), tuple(pick_joint_value))
        arm_util.execute_path(final_path, '/arm_1/follow_joint_trajectory')

        arm_util.set_gripper_width("youbot", 0.01)
        rospy.sleep(rospy.Duration.from_sec(3.0))

        #directly retract arm to pre_pick_pos
        start = arm_util.get_current_joint_pos()
        [final_path, final_cost] = prmstar_planner.direct_path(tuple(start), tuple(pre_pick_joint_value))
        arm_util.execute_path(final_path, '/arm_1/follow_joint_trajectory')