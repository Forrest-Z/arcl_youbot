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

    
    
    
    # env = app_util.YoubotEnvironment(-1.5, 1.5, -3.0, 1.0)    
    env = app_util.YoubotEnvironment(-2.5, 2.5, 0.0, 5.0, 'youbot_0', 0)
    my_path = os.path.abspath(os.path.dirname(__file__))

    
    #import object list from file
    env.import_obj_from_file(os.path.join(my_path, "scatter/new_12.txt"))
    


    
    # export object
    # env.create_environment(12, 12)
    # env.export_obj_to_file(os.path.join(my_path, "scatter/new_12.txt"))
    

    print(env.object_list)
    





    #spawn the objects in gazebo, and generate the planningscene msg 
    env.generate_obj_in_gazebo() ############ comment it and type "roscore" when gazebo is useless






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



    '''
    # Calculation
    DP = TSP_DP.DP_solver(env_obj_list)
    DP.experiment()
    '''


    '''
    # Generate DP Lists
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
    # Finish Generating DP Lists
    '''
    
    # test01 DP
    pick_rounds_index_list = [[11, 4], [2, 8], [5], [1], [0], [10, 3], [9], [6], [7]]
    pick_rounds_pose_list = [[Pose(Point(-1.36966551903,2.67661788095,0),Quaternion(0,0,0,0)),Pose(Point(-1.31235395001,3.0563780111,0),Quaternion(0,0,0,0)),],[Pose(Point(1.42974166644,1.257625297,0),Quaternion(0,0,0,0)),Pose(Point(1.73132265387,1.58661806475,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.645789244145,1.23750732606,0),Quaternion(0,0,0,0)),],[Pose(Point(0.238576149994,1.33695897234,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.796306242369,1.10730953555,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.00827344024391,3.01839653092,0),Quaternion(0,0,0,0)),Pose(Point(-0.0345655851531,3.56548463888,0),Quaternion(0,0,0,0)),],[Pose(Point(0.763295278464,2.39576355583,0),Quaternion(0,0,0,0)),],[Pose(Point(1.32617180839,3.64663108252,0),Quaternion(0,0,0,0)),],[Pose(Point(0.658004563047,0.336543156788,0),Quaternion(0,0,0,0)),],]    
    '''
    # test02 Greedy
    pick_rounds_index_list = [[7, 1], [6, 9], [4, 0], [10], [11], [8], [3], [5], [2]]
    pick_rounds_pose_list = [[Pose(Point(0.0257430720617,0.211774081541,0),Quaternion(0,0,0,0)),Pose(Point(0.078846220153,0.531629861229,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.468082668307,0.0204410384512,0),Quaternion(0,0,0,0)),Pose(Point(-0.0819101375602,2.00782546733,0),Quaternion(0,0,0,0)),],[Pose(Point(1.42650784669,1.24072790743,0),Quaternion(0,0,0,0)),Pose(Point(0.222309888616,2.30185703351,0),Quaternion(0,0,0,0)),],[Pose(Point(-1.150787692,1.6427049884,0),Quaternion(0,0,0,0)),],[Pose(Point(0.622390570734,2.38253728428,0),Quaternion(0,0,0,0)),],[Pose(Point(-1.24655696595,2.38837646244,0),Quaternion(0,0,0,0)),],[Pose(Point(1.69541548259,2.84284930193,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.500431008682,3.47998772411,0),Quaternion(0,0,0,0)),],[Pose(Point(1.25310607939,3.31091412478,0),Quaternion(0,0,0,0)),],]
    '''
    '''
    # Generate Greedy Lists
    pick_list = DP.greedy_obj_order
    pick_rounds_index_list = []
    for one_round in pick_list:
        pick_index_list = []
        for obj in one_round:
            pick_index_list.append(obj)
        pick_rounds_index_list.append(pick_index_list)
    
    print('=========')
    print(pick_rounds_index_list)

    
    pick_list_pose = DP.greedy_robot_locations
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
    # Finish Generate Greedy Lists
    '''
    # No Change below

    for i, pick_index_list in enumerate(pick_rounds_index_list):
        if i < 6:
            continue

        for j, index in enumerate(pick_index_list):
            print("pick " + str(index))
            obj_name = "obj_" + str(index)
            env.send_grasp_action(env.planning_scene_msg, obj_name, env.reserved_planning_scene_msg.scene_object_list[env.obj_name_to_index_dict[obj_name]].object_pose, " ", "cube", pick_rounds_pose_list[i][j], True)
            target_base_pose = env.grasp_plan_result.final_base_pose
            env.move_to_target("youbot_0", target_base_pose)
            
            pick_joint_value = [env.grasp_plan_result.q1, env.grasp_plan_result.q2, env.grasp_plan_result.q3, env.grasp_plan_result.q4, env.grasp_plan_result.q5]
            pre_pick_joint_value = [env.grasp_plan_result.q1_pre, env.grasp_plan_result.q2_pre, env.grasp_plan_result.q3_pre, env.grasp_plan_result.q4_pre, env.grasp_plan_result.q5_pre]    
            env.pick_object("youbot_0", pick_joint_value, pre_pick_joint_value)
            env.update_env(obj_name)
            env.move_arm_to_joint_pose("youbot_0", arm_drop_joint)
            time.sleep(1.5)
            env.drop_object("youbot_0", obj_name)

        env.move_to_target("youbot_0", rest_base_pose)