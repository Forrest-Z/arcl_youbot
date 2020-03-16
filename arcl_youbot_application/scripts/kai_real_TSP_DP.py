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
import TSP_DP_new as TSP_DP
import copy
import Queue
import threading


def get_next_grasp(env, obj_name, obj_pick_pose, result):
    grasp_plan_result = env.send_grasp_action_asyn(env.planning_scene_msg, obj_name, env.reserved_planning_scene_msg.scene_object_list[env.obj_name_to_index_dict[obj_name]].object_pose, " ", "cube", obj_pick_pose, True)
    env.update_copy_env(obj_name)
    result.put(grasp_plan_result)

if __name__ == "__main__":
    rospy.init_node("single_youbot_pick_demo")

    env = app_util.YoubotEnvironment(-1.5, 1.5, -3.0, 1.0, 'youbot_0', 1)    
    #import object list from file
    my_path = os.path.abspath(os.path.dirname(__file__))
    # env.import_obj_from_file(os.path.join(my_path, "scatter/new_11.txt"))
    # env.create_environment(15, 5)
    # print(env.object_list)
    # env.export_obj_to_file(os.path.join(my_path, "scatter/new_12.txt"))
    env.import_obj_from_optitrack()
    #spawn the objects in gazebo, and generate the planningscene msg 
    # env.generate_obj_in_gazebo()

    rest_base_pose = Pose()
    rest_base_pose.position.x = 0
    rest_base_pose.position.y = 0
    rest_base_pose.position.z = 0.1
    rest_base_pose.orientation.x = 0
    rest_base_pose.orientation.y = 0
    rest_base_pose.orientation.z = -0.7035624
    rest_base_pose.orientation.w = 0.7106335

    target_base_pose = Pose()
    target_base_pose.position.x = 0.2
    target_base_pose.position.y = -1
    target_base_pose.position.z = 0.1
    target_base_pose.orientation.x = 0
    target_base_pose.orientation.y = 0
    target_base_pose.orientation.z = -0.7071068
    target_base_pose.orientation.w = 0.7071068   

    arm_drop_joint = [math.radians(169), math.radians(60), math.radians(-170), math.radians(0), math.radians(82)]
    arm_up_joint = [202/180.0*math.pi, 65/180.0*math.pi, -146 / 180.0 * math.pi, 102.5 / 180.0 * math.pi, 172 / 180.0 * math.pi]
    print('=========')
    print(env.object_list)
    

    # # Calculation
    # env_obj_list = copy.deepcopy(env.object_list)
    # DP = TSP_DP.DP_solver(env_obj_list)
    # DP.experiment()
    
    # # # Generate DP Lists
    # pick_list = DP.DP_obj_order
    # pick_rounds_index_list = []
    # for one_round in pick_list:
    #     pick_index_list = []
    #     for obj in one_round:
    #         pick_index_list.append(obj)
    #     pick_rounds_index_list.append(pick_index_list)
    
    # print('=========')
    # print(pick_rounds_index_list)
    # pick_list_pose = DP.DP_robot_locations
    # pick_rounds_pose_list = []
    # for one_round in pick_list_pose:
    #     pick_pose_list = []
    #     for p in one_round:
    #         pose = Pose()
    #         pose.position.x = p[0]
    #         pose.position.y = p[1]
    #         pick_pose_list.append(pose)
    #     pick_rounds_pose_list.append(pick_pose_list)   
    # print('=========')
    # print('[', end="")
    
    # for i in pick_rounds_pose_list:
    #     print('[', end="")
    #     for j in i:
    #         print("Pose(Point({},{},0),Quaternion(0,0,0,0))".format(j.position.x, j.position.y), end=",")
    #     print(']', end=",")
    # print(']')
    # # # Finish Generating DP Lists

    # # Generate Greedy Lists
    # pick_list = DP.greedy_obj_order
    # pick_rounds_index_list = []
    # for one_round in pick_list:
    #     pick_index_list = []
    #     for obj in one_round:
    #         pick_index_list.append(obj)
    #     pick_rounds_index_list.append(pick_index_list)
    
    # print('=========')
    # print(pick_rounds_index_list)

    
    # pick_list_pose = DP.greedy_robot_locations
    # pick_rounds_pose_list = []
    # for one_round in pick_list_pose:
    #     pick_pose_list = []
    #     for p in one_round:
    #         pose = Pose()
    #         pose.position.x = p[0]
    #         pose.position.y = p[1]
    #         pick_pose_list.append(pose)
    #     pick_rounds_pose_list.append(pick_pose_list)   
    
    # print('=========')
    # print('[', end="")
    
    # for i in pick_rounds_pose_list:
    #     print('[', end="")
    #     for j in i:
    #         print("Pose(Point({},{},0),Quaternion(0,0,0,0))".format(j.position.x, j.position.y), end=",")
    #     print(']', end=",")
    # print(']')
    # # Finish Generate Greedy Lists
    # # print object
    # for obj in env_obj_list.keys():
    #     env_obj_list[obj].pop()
    # print(env_obj_list)

    # DP
    # pick_rounds_index_list = [[10, 11, 3], [7, 8, 5, 12], [0, 6], [9, 4, 2]]
    # pick_rounds_pose_list = [[Pose(Point(-0.643515508055,-0.122821919396,0),Quaternion(0,0,0,0)),Pose(Point(-1.03520174823,-0.336748850021,0),Quaternion(0,0,0,0)),Pose(Point(-0.828584875059,-1.02804475029,0),Quaternion(0,0,0,0)),],[Pose(Point(0.823003386122,-0.802467928306,0),Quaternion(0,0,0,0)),Pose(Point(1.09946250004,-1.38999014649,0),Quaternion(0,0,0,0)),Pose(Point(1.03756499278,-1.917554773,0),Quaternion(0,0,0,0)),Pose(Point(0.505767623677,-2.18939523654,0),Quaternion(0,0,0,0)),],[Pose(Point(0.638322181021,-0.171834099524,0),Quaternion(0,0,0,0)),Pose(Point(0.467518211393,-0.74200452279,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.0278446283252,-1.34225166689,0),Quaternion(0,0,0,0)),Pose(Point(-0.244377644919,-1.98662260365,0),Quaternion(0,0,0,0)),Pose(Point(-0.708715769235,-2.22277885895,0),Quaternion(0,0,0,0)),],]
    # Greedy
    # pick_rounds_index_list = [[11, 10, 6], [0, 7, 8, 5], [3, 9, 4], [12, 2]]
    # pick_rounds_pose_list = [[Pose(Point(-0.438969466282,-0.375161614028,0),Quaternion(0,0,0,0)),Pose(Point(-0.827659955947,-0.561075074991,0),Quaternion(0,0,0,0)),Pose(Point(-0.462567264056,-0.733122865089,0),Quaternion(0,0,0,0)),],[Pose(Point(0.645147491288,-0.0685088091877,0),Quaternion(0,0,0,0)),Pose(Point(1.10612695846,-0.672430226311,0),Quaternion(0,0,0,0)),Pose(Point(1.10046250004,-1.38899014649,0),Quaternion(0,0,0,0)),Pose(Point(1.03856499278,-1.916554773,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.484126854911,-1.03615894724,0),Quaternion(0,0,0,0)),Pose(Point(-0.412863517087,-1.56453111939,0),Quaternion(0,0,0,0)),Pose(Point(-0.243377644919,-1.98562260365,0),Quaternion(0,0,0,0)),],[Pose(Point(0.0778764212601,-1.62485719609,0),Quaternion(0,0,0,0)),Pose(Point(-0.603936896048,-2.15684360428,0),Quaternion(0,0,0,0)),],]
    pick_rounds_index_list = [[3, 9, 4], [12, 2]]
    pick_rounds_pose_list = [[Pose(Point(-0.484126854911,-1.03615894724,0),Quaternion(0,0,0,0)),Pose(Point(-0.412863517087,-1.56453111939,0),Quaternion(0,0,0,0)),Pose(Point(-0.243377644919,-1.98562260365,0),Quaternion(0,0,0,0)),],[Pose(Point(0.0778764212601,-1.62485719609,0),Quaternion(0,0,0,0)),Pose(Point(-0.603936896048,-2.15684360428,0),Quaternion(0,0,0,0)),],]

    env.move_arm_to_joint_pose("youbot_0", arm_drop_joint)
    raw_input("press to start")

    loaded = False
    loaded_obj_name = ''
    result = Queue.Queue()   
    env.copy_env()
    next_obj_name = "obj_" + str(pick_rounds_index_list[0][0])
    next_obj_pick_pose = pick_rounds_pose_list[0][0]
    get_next_grasp(env, next_obj_name, next_obj_pick_pose, result)

    for i, pick_index_list in enumerate(pick_rounds_index_list):
        for j, index in enumerate(pick_index_list):
            print("pick " + str(index))
            obj_name = "obj_" + str(index)
            
            # next obj
            if j < len(pick_index_list) - 1:
                next_obj_name = "obj_" + str(pick_rounds_index_list[i][j+1])
                next_obj_pick_pose = pick_rounds_pose_list[i][j+1]
            else:
                if i < len(pick_rounds_index_list) - 1:
                    next_obj_name = "obj_" + str(pick_rounds_index_list[i+1][0])
                    next_obj_pick_pose = pick_rounds_pose_list[i+1][0]
            
            if not (i == 0 and j == 0):
                grasp.join()
            grasp = threading.Thread(target=get_next_grasp, args=(env, next_obj_name, next_obj_pick_pose, result))
            grasp.start()

            grasp_plan = result.get()
            target_base_pose = grasp_plan.final_base_pose
            pick_joint_value = [grasp_plan.q1, grasp_plan.q2, grasp_plan.q3, grasp_plan.q4, grasp_plan.q5]
            pre_pick_joint_value = [grasp_plan.q1_pre, grasp_plan.q2_pre, grasp_plan.q3_pre, grasp_plan.q4_pre, grasp_plan.q5_pre]    
            
            if loaded:
                env.combined_move_base_and_arm_drop_pick("youbot_0", loaded_obj_name, arm_drop_joint, pre_pick_joint_value, target_base_pose)
            else:
                env.combined_move_base_and_arm_pick("youbot_0", pre_pick_joint_value, target_base_pose)
                loaded = True
                loaded_obj_name = obj_name

            env.pick_object_from_prev("youbot_0", pick_joint_value, pre_pick_joint_value)
            env.update_env(obj_name)

            if j == len(pick_index_list) - 1:
                env.combined_move_base_and_arm_drop("youbot_0", obj_name, arm_drop_joint, rest_base_pose)
                loaded = False
        time.sleep(5)

    # for i, pick_index_list in enumerate(pick_rounds_index_list):
    #     for j, index in enumerate(pick_index_list):
    #         print("pick " + str(index))
    #         obj_name = "obj_" + str(index)
    #         env.send_grasp_action(env.planning_scene_msg, obj_name, env.reserved_planning_scene_msg.scene_object_list[env.obj_name_to_index_dict[obj_name]].object_pose, " ", "cube", pick_rounds_pose_list[i][j], True)
    #         target_base_pose = env.grasp_plan_result.final_base_pose
    #         env.move_to_target("youbot_0", target_base_pose)
            
    #         pick_joint_value = [env.grasp_plan_result.q1, env.grasp_plan_result.q2, env.grasp_plan_result.q3, env.grasp_plan_result.q4, env.grasp_plan_result.q5]
    #         pre_pick_joint_value = [env.grasp_plan_result.q1_pre, env.grasp_plan_result.q2_pre, env.grasp_plan_result.q3_pre, env.grasp_plan_result.q4_pre, env.grasp_plan_result.q5_pre]    
    #         env.pick_object("youbot_0", pick_joint_value, pre_pick_joint_value)
    #         env.update_env(obj_name)
    #         env.move_arm_to_joint_pose("youbot_0", arm_drop_joint)
    #         time.sleep(1.5)
    #         env.drop_object("youbot_0", obj_name)

    #     env.move_to_target("youbot_0", rest_base_pose)