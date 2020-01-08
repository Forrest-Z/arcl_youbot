from __future__ import print_function
import rospy
import arcl_youbot_application.application_util as app_util
import arcl_youbot_planner.arm_planner.arm_util as arm_util
from geometry_msgs.msg import Pose, PointStamped, Point, Quaternion
from std_msgs.msg import String
from std_msgs.msg import UInt8
import os.path
import math
import time
import TSP_DP
from copy import deepcopy


base_pose = Pose()

def position_callback(self, data):
    global base_pose
    base_pose.position.x = data.point.x
    base_pose.position.y = data.point.y

if __name__ == "__main__":
    rospy.init_node("single_youbot_pick_demo")
    rospy.Subscriber('/youbot_0/robot/pose', PointStamped, position_callback)

    env = app_util.YoubotEnvironment(-1.5, 1.5, -3.0, 1.0)    
    # env = app_util.YoubotEnvironment(-2.5, 2.5, 0.0, 5.0)
    env.mode = 0
    #import object list from file
    my_path = os.path.abspath(os.path.dirname(__file__))
    env.import_obj_from_file(os.path.join(my_path, "scatter/new_4.txt"))
    # env.create_environment(15, 5)
    # print(env.object_list)
    # env.export_obj_to_file(os.path.join(my_path, "scatter/new_12.txt"))

    #spawn the objects in gazebo, and generate the planningscene msg 
    env.generate_obj_in_gazebo()

    rest_base_pose = Pose()
    rest_base_pose.position.x = 0
    rest_base_pose.position.y = 0
    rest_base_pose.position.z = 0.1
    rest_base_pose.orientation.x = 0
    rest_base_pose.orientation.y = 0
    rest_base_pose.orientation.z = 0.7135624
    rest_base_pose.orientation.w = 0.7006335

    target_base_pose = Pose()
    target_base_pose.position.x = 0.2
    target_base_pose.position.y = -1
    target_base_pose.position.z = 0.1
    target_base_pose.orientation.x = 0
    target_base_pose.orientation.y = 0
    rest_base_pose.orientation.z = 0.7135624
    rest_base_pose.orientation.w = 0.7006335

    arm_drop_joint = [math.radians(169), math.radians(60), math.radians(-190), math.radians(50), math.radians(82)]
    print('=========')
    env_obj_list = deepcopy(env.object_list)
    del env_obj_list['wall']
    print(env_obj_list)
    DP = TSP_DP.DP_solver(env_obj_list)
    DP.experiment()

    pick_list = DP.DP_obj_order
    pick_rounds_index_list = []
    for one_round in pick_list:
        pick_index_list = []
        for obj in one_round:
            pick_index_list.append(obj)
        pick_rounds_index_list.append(pick_index_list)
    
    print('=========')
    print(pick_rounds_index_list)

    pick_list_pose = DP.DP_robot_locations
    pick_rounds_pose_list = []
    for one_round in pick_list_pose:
        pick_pose_list = []
        for p in one_round:
            pose = Pose()
            pose.position.x = p[0]
            pose.position.y = p[1]
            pick_pose_list.append(pose)
        pick_rounds_pose_list.append(pick_pose_list)   

    print('=========')
    print('[', end="")
    for i in pick_rounds_pose_list:
        print('[', end="")
        for j in i:
            print("Pose(Point({},{},0),Quaternion(0,0,0,0))".format(j.position.x, j.position.y), end=",")
        print(']', end=",")
    print(']')
    # pick_rounds_index_list = [[4, 5], [5], [5], [5], [5], [5]]
    # pick_rounds_pose_list = [[Pose(Point(0.110073603033,1.93999787971,0),Quaternion(0,0,0,0)),Pose(Point(0.141308147373,2.3796050782,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.568841216735,3.13117400211,0),Quaternion(0,0,0,0)),],[Pose(Point(0.551076652833,2.03347117204,0),Quaternion(0,0,0,0)),],[Pose(Point(-1.50341119192,2.14773027418,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.428179910782,2.78099270254,0),Quaternion(0,0,0,0)),],[Pose(Point(0.757132313206,2.43490669277,0),Quaternion(0,0,0,0))]]

    for i, pick_index_list in enumerate(pick_rounds_index_list):

        reserved_object_list = {}
        for obj_name, test_obj  in env_obj_list.iteritems():
            reserved_object_list[obj_name] = test_obj

        for j, index in enumerate(pick_index_list):
            print("pick " + str(index))
            obj_name = "obj_" + str(index)
            env.send_grasp_action(env.planning_scene_msg, obj_name, env.reserved_planning_scene_msg.scene_object_list[env.obj_name_to_index_dict[obj_name]].object_pose, " ", "cube", pick_rounds_pose_list[i][j], True)
            target_base_pose = env.grasp_plan_result.final_base_pose
            env.move_to_target("youbot_0", target_base_pose)
            
            pick_joint_value = [env.grasp_plan_result.q1, env.grasp_plan_result.q2, env.grasp_plan_result.q3, env.grasp_plan_result.q4, env.grasp_plan_result.q5]
            pre_pick_joint_value = [env.grasp_plan_result.q1_pre, env.grasp_plan_result.q2_pre, env.grasp_plan_result.q3_pre, env.grasp_plan_result.q4_pre, env.grasp_plan_result.q5_pre]    
            env.pick_object("youbot_0", pick_joint_value, pre_pick_joint_value)
            env.update_env(reserved_object_list[obj_name])
            env.move_arm_to_joint_pose("youbot_0", arm_drop_joint)
            time.sleep(1.5)
            env.drop_object("youbot_0", obj_name)

        env.move_to_target("youbot_0", rest_base_pose)