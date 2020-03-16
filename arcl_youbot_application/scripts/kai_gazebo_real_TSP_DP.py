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
from copy import deepcopy
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_about_axis, quaternion_from_matrix
import pickle
from shapely.geometry import Polygon
from arcl_youbot_planner.base_planner.base_util import YOUBOT_LONG_RADIUS
from shapely.ops import unary_union, nearest_points
import Queue
import threading


def get_next_grasp(env, obj_name, obj_pick_pose, result):
    grasp_plan_result = env.send_grasp_action_asyn(env.planning_scene_msg, obj_name, env.reserved_planning_scene_msg.scene_object_list[env.obj_name_to_index_dict[obj_name]].object_pose, " ", "cube", obj_pick_pose, True)
    env.update_copy_env(obj_name)
    result.put(grasp_plan_result)

if __name__ == "__main__":
    rospy.init_node("single_youbot_pick_demo")
    
    
    
    # env = app_util.YoubotEnvironment(-1.5, 1.5, -3.0, 1.0)    
    env = app_util.YoubotEnvironment(-3.2, 3.2, 1.0, 6.8, 'youbot_0', 0)
    my_path = os.path.abspath(os.path.dirname(__file__))

    
    #import object list from file
    # env.import_obj_from_file(os.path.join(my_path, "scatter/new_12.txt"))


    
    # export object
    # env.create_environment(12, 12)
    # env.export_obj_to_file(os.path.join(my_path, "scatter/new_12_05.txt"))
    
    env.object_list = {'obj_9': [(0.21951262507000213, -1.7999411144247686), (0.22072525292985756, -1.7599594994789773), (-0.03615662309684997, -1.7521683654794062), (-0.0373692509567054, -1.7921499804251975)], 
    'obj_8': [(1.055890987209732, -1.8371916960388257), (1.0495684040691744, -1.7976945449715558), (0.9172529479938203, -1.8188751984924243), (0.9235755311343781, -1.8583723495596942)], 
    'obj_3': [(-0.8417369239594095, -1.5898134292550163), (-0.812061695368425, -1.6166347045429816), (-0.6524751074050315, -1.4400670944266243), (-0.6821503359960159, -1.413245819138659)], 
    'obj_2': [(-1.0488619571407027, -2.2277013933842804), (-1.0119974888300578, -2.2121768651733027), (-1.0837984318058305, -2.041678699236569), (-1.1206629001164754, -2.057203227447547)], 
    'obj_0': [(1.0950967451452498, -0.08492102562708688), (1.0553276145547432, -0.08062960953502064), (1.0336559632898088, -0.2814637190170782), (1.0734250938803154, -0.2857551351091444)], 
    'obj_7': [(1.148185821021395, -1.09264003537993), (1.113566252080079, -1.1126771371243804), (1.187202600990934, -1.2399040529837175), (1.22182216993225, -1.219866951239267)], 
    'obj_6': [(0.12967455465820407, -1.0269162495108612), (0.12797795700281148, -0.986952246252983), (-0.14177906498786783, -0.9984042804268828), (-0.14008246733247523, -1.0383682836847612)], 
    'obj_5': [(0.8206329140236112, -2.415904638062328), (0.8420809299795403, -2.4496682665929557), (0.983888169808176, -2.3595865995780527), (0.9624401538522468, -2.325822971047425)], 
    'obj_4': [(-0.5326738792886256, -2.312413385935605), (-0.5542471601246124, -2.278729657609132), (-0.6805611413488866, -2.3596294607440824), (-0.6589878605128998, -2.3933131890705557)], 
    'obj_12': [(0.11717038464939065, -2.0545950410912472), (0.07720099408303428, -2.0561595914273063), (0.08482817697132163, -2.2510103704382938), (0.124797567537678, -2.2494458201022347)], 
    'obj_11': [(-0.9488054730872307, -0.7385086078589789), (-0.9286656026204995, -0.773068492372433), (-0.7567301771660652, -0.672872636800445), (-0.7768700476327964, -0.6383127522869908)], 
    'obj_10': [(-1.0140749040534571, -0.03197532974966874), (-1.0534172381631561, -0.02475170553458078), (-1.0855623659202978, -0.19982509232274184), (-1.0462200318105987, -0.2070487165378298)]}

    print(env.object_list)
    





    #spawn the objects in gazebo, and generate the planningscene msg 
    env.generate_obj_in_gazebo() ############ comment it and type "roscore" when gazebo is useless






    rest_base_pose = Pose()
    rest_base_pose.position.x = 0
    rest_base_pose.position.y = 0
    rest_base_pose.position.z = 0.1
    rest_base_pose.orientation.x = 0
    rest_base_pose.orientation.y = 0
    rest_base_pose.orientation.z = 0.7035624
    rest_base_pose.orientation.w = -0.7106335

    q = (rest_base_pose.orientation.x,
        rest_base_pose.orientation.y,
        rest_base_pose.orientation.z,
        rest_base_pose.orientation.w)
    (_, _, yaw) = euler_from_quaternion(q)
    rest_base_pose_2d = [rest_base_pose.position.x, rest_base_pose.position.y, yaw]

    target_base_pose = Pose()
    target_base_pose.position.x = 0.2
    target_base_pose.position.y = -1
    target_base_pose.position.z = 0.1
    target_base_pose.orientation.x = 0
    target_base_pose.orientation.y = 0

    arm_drop_joint = [math.radians(169), math.radians(60), math.radians(-190), math.radians(50), math.radians(82)]
    print('=========')
    env_obj_list = deepcopy(env.object_list)
    # del env_obj_list['wall']


    # Calculation
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

    # # # Generate Greedy Liststarget_base_pose
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
    # Finish Generate Greedy Lists
    
    # ===== pre-compute =====
    # paths = []
    # return_paths = []
    # pick_joint_values = []
    # pre_pick_joint_values = []
    # arm_paths = []
    # drop_paths = []

    # for i, pick_index_list in enumerate(pick_rounds_index_list):
    #     start_pose = rest_base_pose_2d
    #     one_round_path = []
    #     one_round_pick = []
    #     one_round_pre = []
    #     one_round_arm_pick_path = []
    #     one_round_drop_path = []

    #     for j, index in enumerate(pick_index_list):
    #         print("pick " + str(index))
    #         obj_name = "obj_" + str(index)
    #         env.send_grasp_action(env.planning_scene_msg, obj_name, env.reserved_planning_scene_msg.scene_object_list[env.obj_name_to_index_dict[obj_name]].object_pose, " ", "cube", pick_rounds_pose_list[i][j], True)
    #         target_base_pose = env.grasp_plan_result.final_base_pose
    #         path, target_base_pose_2d  = env.get_path('youbot_0', start_pose, target_base_pose)
    #         start_pose = target_base_pose_2d
    #         one_round_path.append(path)

    #         pick_joint_value = [env.grasp_plan_result.q1, env.grasp_plan_result.q2, env.grasp_plan_result.q3, env.grasp_plan_result.q4, env.grasp_plan_result.q5]
    #         pre_pick_joint_value = [env.grasp_plan_result.q1_pre, env.grasp_plan_result.q2_pre, env.grasp_plan_result.q3_pre, env.grasp_plan_result.q4_pre, env.grasp_plan_result.q5_pre]   
    #         one_round_pick.append(pick_joint_value)
    #         one_round_pre.append(pre_pick_joint_value)
    #         arm_pick_path = env.get_pick_object_path('youbot_0', pick_joint_value, pre_pick_joint_value, target_base_pose, arm_drop_joint)
    #         one_round_arm_pick_path.append(arm_pick_path)
    #         env.update_env(obj_name)
    #         state, drop_path = env.get_move_arm_to_joint_pose_path('youbot_0', target_base_pose, pre_pick_joint_value, arm_drop_joint)
    #         if state == False: 
    #             print("move_arm_to_joint_return_false")
    #             exit(-1)
    #         one_round_drop_path.append(drop_path)

    #     return_path, _ = env.get_path('youbot_0', start_pose, rest_base_pose)

    #     paths.append(one_round_path)
    #     return_paths.append(return_path)
    #     pick_joint_values.append(one_round_pick)
    #     pre_pick_joint_values.append(one_round_pre)
    #     arm_paths.append(one_round_arm_pick_path)
    #     drop_paths.append(one_round_drop_path)

    # # save data
    # with open('new_12_05_dp.pkl', 'wb') as output:
    #     pickle.dump((paths, return_paths, pick_joint_values, pre_pick_joint_values, arm_paths, drop_paths), output, pickle.HIGHEST_PROTOCOL)
    # ===== pre-compute =====

    # ===== executaion =====
    # read data
    # with open('new_12_04_dp.pkl', 'rb') as input:
    #     paths, return_paths, pick_joint_values, pre_pick_joint_values, arm_paths, drop_paths = pickle.load(input)

    # for i, pick_index_list in enumerate(pick_rounds_index_list):
    #     for j, index in enumerate(pick_index_list):
    #         print("pick " + str(index))
    #         obj_name = "obj_" + str(index)
    #         env.follow_path("youbot_0", paths[i][j])
            
    #         env.follow_pick_object_path("youbot_0", arm_paths[i][j])
    #         env.follow_move_arm_to_joint_pose_path("youbot_0", drop_paths[i][j])
    #         env.drop_object("youbot_0", obj_name)

    #     env.follow_path("youbot_0", return_paths[i])
    # ===== executaion =====

    # DP
    # pick_rounds_index_list = [[10, 11, 3], [7, 8, 5, 12], [0, 6], [9, 4, 2]]
    # pick_rounds_pose_list = [[Pose(Point(-0.643515508055,-0.122821919396,0),Quaternion(0,0,0,0)),Pose(Point(-1.03520174823,-0.336748850021,0),Quaternion(0,0,0,0)),Pose(Point(-0.828584875059,-1.02804475029,0),Quaternion(0,0,0,0)),],[Pose(Point(0.823003386122,-0.802467928306,0),Quaternion(0,0,0,0)),Pose(Point(1.09946250004,-1.38999014649,0),Quaternion(0,0,0,0)),Pose(Point(1.03756499278,-1.917554773,0),Quaternion(0,0,0,0)),Pose(Point(0.505767623677,-2.18939523654,0),Quaternion(0,0,0,0)),],[Pose(Point(0.638322181021,-0.171834099524,0),Quaternion(0,0,0,0)),Pose(Point(0.467518211393,-0.74200452279,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.0278446283252,-1.34225166689,0),Quaternion(0,0,0,0)),Pose(Point(-0.244377644919,-1.98662260365,0),Quaternion(0,0,0,0)),Pose(Point(-0.708715769235,-2.22277885895,0),Quaternion(0,0,0,0)),],]
    # Greedy
    pick_rounds_index_list = [[11, 10, 6], [0, 7, 8, 5], [3, 9, 4], [12, 2]]
    pick_rounds_pose_list = [[Pose(Point(-0.438969466282,-0.375161614028,0),Quaternion(0,0,0,0)),Pose(Point(-0.827659955947,-0.561075074991,0),Quaternion(0,0,0,0)),Pose(Point(-0.462567264056,-0.733122865089,0),Quaternion(0,0,0,0)),],[Pose(Point(0.645147491288,-0.0685088091877,0),Quaternion(0,0,0,0)),Pose(Point(1.10612695846,-0.672430226311,0),Quaternion(0,0,0,0)),Pose(Point(1.10046250004,-1.38899014649,0),Quaternion(0,0,0,0)),Pose(Point(1.03856499278,-1.916554773,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.484126854911,-1.03615894724,0),Quaternion(0,0,0,0)),Pose(Point(-0.412863517087,-1.56453111939,0),Quaternion(0,0,0,0)),Pose(Point(-0.243377644919,-1.98562260365,0),Quaternion(0,0,0,0)),],[Pose(Point(0.0778764212601,-1.62485719609,0),Quaternion(0,0,0,0)),Pose(Point(-0.603936896048,-2.15684360428,0),Quaternion(0,0,0,0)),],]

    env.combined_move_base_and_arm("youbot_0", arm_drop_joint, rest_base_pose)

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