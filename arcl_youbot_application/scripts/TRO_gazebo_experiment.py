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
# import TSP_DP_new as TSP_DP
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
    env.import_obj_from_file(os.path.join(my_path, "scatter/15_0_run.txt"))


    
    # export object
    # env.create_environment(12, 12)
    # env.export_obj_to_file(os.path.join(my_path, "scatter/new_12_05.txt"))
    
    # env.object_list = {'obj_9': [(-1.098122932586211, -0.5324908891303185), (-1.1322310918847451, -0.5533866654802371), (-0.9979757288365184, -0.7725315889733192), (-0.9638675695379842, -0.7516358126234006)], 
    # 'obj_3': [(-0.745800071390263, -0.2266099148689336), (-0.7607194475833075, -0.18949641039939555), (-0.9815447991770587, -0.2782666987480098), (-0.9666254229840143, -0.3153802032175478)], 
    # 'obj_2': [(0.7647621217210173, -2.1231637553747804), (0.7508187535932557, -2.0856726566278283), (0.5774224218886018, -2.1501607342187254), (0.5913657900163635, -2.1876518329656776)], 
    # 'obj_1': [(-0.6118046450166261, -1.8511857355833932), (-0.5939399283444082, -1.815396744284093), (-0.8059897017927611, -1.7095482980012016), (-0.823854418464979, -1.7453372893005017)], 
    # 'obj_0': [(1.2421660464481836, -1.737083817527939), (1.2738140375863005, -1.7126215065798363), (1.1258170563502783, -1.521151160194229), (1.0941690652121614, -1.5456134711423317)], 
    # 'obj_6': [(1.1790826548468492, -0.2815092074331322), (1.1390880790178068, -0.28216792152512327), (1.1435343991387465, -0.5521313083711585), (1.1835289749677889, -0.5514725942791675)], 
    # 'obj_5': [(1.2324432599040038, -2.1026539372024953), (1.2110263426984595, -2.0688705735835353), (1.0691362154988282, -2.158821625846821), (1.0905531327043725, -2.192604989465781)], 
    # 'obj_12': [(-1.1146492836944975, -2.1242627160301923), (-1.1503188204354198, -2.142364764220604), (-1.0620713355071627, -2.3162537558326006), (-1.0264017987662404, -2.298151707642189)], 
    # 'obj_11': [(-0.6453468981318842, -2.3371132286618352), (-0.6303888409420546, -2.374211160209373), (-0.44582663149305485, -2.2997948256899714), (-0.4607846886828843, -2.2626968941424335)], 
    # 'obj_10': [(-0.681692665823188, -0.8575739085831304), (-0.6674634535339784, -0.8949574546646434), (-0.5011066734712456, -0.8316374599776606), (-0.5153358857604552, -0.7942539138961476)]}

    print(env.object_list)
    print("DP_seq:")
    print(env.dp_seq)
    print("Greedy_seq:")
    print(env.greedy_seq)





    #spawn the objects in gazebo, and generate the planningscene msg 
    env.generate_obj_in_gazebo() ############ comment it and type "roscore" when gazebo is useless






    rest_base_pose = Pose()
    rest_base_pose.position.x = 0
    rest_base_pose.position.y = 0
    rest_base_pose.position.z = 0.1
    rest_base_pose.orientation.x = 0
    rest_base_pose.orientation.y = 0
    rest_base_pose.orientation.z = -0.7071068
    rest_base_pose.orientation.w = 0.7071068

    init_pose = Pose()
    init_pose.position.x = 0
    init_pose.position.y = -0.5
    init_pose.position.z = 0.1
    init_pose.orientation.x = 0
    init_pose.orientation.y = 0
    init_pose.orientation.z = 0.7071068
    init_pose.orientation.w = 0.7071068

    q = (init_pose.orientation.x,
        init_pose.orientation.y,
        init_pose.orientation.z,
        init_pose.orientation.w)
    (_, _, yaw) = euler_from_quaternion(q)
    init_pose_2d = [init_pose.position.x, init_pose.position.y, yaw]


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

    arm_drop_joint = [math.radians(170), math.radians(55), math.radians(-56), math.radians(180), math.radians(170)]
    print('=========')
    env_obj_list = deepcopy(env.object_list)

    is_dp = False
    is_greedy = False
    method = input('Let us wait for user input. 1 for dp, 0 for greedy.\n')
    print(method)
    if int(method) == 0:
        is_greedy = True 
        print("choose greedy")
    elif int(method) == 1:
        is_dp = True
        print("choose dp")
    

    if is_dp == True:
        pick_index_list = env.dp_seq
    elif is_greedy == True:
        pick_index_list = env.greedy_seq


    

    pick_paths = []
    return_paths = []
    pick_joint_values = []
    pre_pick_joint_values = []
    arm_paths = []
    drop_paths = []
    #===== pre-compute =====


    # start_pose = rest_base_pose_2d
    # for i, index in enumerate(pick_index_list):
    #     print("pick " + str(index))
    #     obj_name = "obj_" + str(index)
    #     env.send_grasp_action(env.planning_scene_msg, obj_name, env.reserved_planning_scene_msg.scene_object_list[env.obj_name_to_index_dict[obj_name]].object_pose, " ", "cube", rest_base_pose, True)
    #     target_base_pose = env.grasp_plan_result.final_base_pose
        
    #     if i == 0:
    #         start_pose = init_pose_2d
    #     else:
    #         start_pose = rest_base_pose_2d
    #     path, target_base_pose_2d  = env.get_path('youbot_0', start_pose, target_base_pose)
    #     start_pose = target_base_pose_2d
    #     pick_paths.append(path)

    #     pick_joint_value = [env.grasp_plan_result.q1, env.grasp_plan_result.q2, env.grasp_plan_result.q3, env.grasp_plan_result.q4, env.grasp_plan_result.q5]
    #     pre_pick_joint_value = [env.grasp_plan_result.q1_pre, env.grasp_plan_result.q2_pre, env.grasp_plan_result.q3_pre, env.grasp_plan_result.q4_pre, env.grasp_plan_result.q5_pre]   
        
    #     arm_pick_path = env.get_pick_object_path('youbot_0', pick_joint_value, pre_pick_joint_value, target_base_pose, arm_drop_joint)
    #     env.update_env(obj_name)

    #     return_path, _ = env.get_path('youbot_0', start_pose, rest_base_pose)
    #     state, drop_path = env.get_move_arm_to_joint_pose_path('youbot_0', rest_base_pose, pre_pick_joint_value, arm_drop_joint)
    #     if state == False: 
    #         print("move_arm_to_joint_return_false")
    #         exit(-1)
    #     arm_paths.append(arm_pick_path)
    #     pick_joint_values.append(pick_joint_value)
    #     pre_pick_joint_values.append(pre_pick_joint_value)
    #     return_paths.append(return_path)
    #     drop_paths.append(drop_path)

    

    # with open('20_run_dp.pkl', 'wb') as output:
    #    pickle.dump((pick_paths, return_paths, pick_joint_values, pre_pick_joint_values, arm_paths, drop_paths), output, pickle.HIGHEST_PROTOCOL)
    #===== end pre-compute =====

    #===== executaion =====
    with open('15_run_dp.pkl', 'rb') as input:
        pick_paths, return_paths, pick_joint_values, pre_pick_joint_values, arm_paths, drop_paths = pickle.load(input)


    start_time = time.time()

    env.move_arm_to_joint_pose_direct("youbot_0", arm_drop_joint)

    
    for j, index in enumerate(pick_index_list):
        print("pick " + str(index))
        obj_name = "obj_" + str(index)
        env.follow_path("youbot_0", pick_paths[j])
        
        env.follow_pick_object_path("youbot_0", arm_paths[j])
        env.follow_path("youbot_0", return_paths[j])
        env.follow_move_arm_to_joint_pose_path("youbot_0", drop_paths[j])
        env.drop_object("youbot_0", obj_name)


    print("Total execution time:--- %s seconds ---" % (time.time() - start_time))
    
    #===== end executaion =====

    # DP
    # pick_rounds_index_list = [[9], [2, 11], [1], [3], [10, 12], [0, 5], [6]]
    # pick_rounds_pose_list = [[Pose(Point(-0.683016193367,-0.425243096452,0),Quaternion(0,0,0,0)),],[Pose(Point(0.472523801307,-1.75371351518,0),Quaternion(0,0,0,0)),Pose(Point(-0.0258034938946,-2.2349652171,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.631459353839,-1.33854507522,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.338779258296,-0.0935720981184,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.280279736286,-0.448332669034,0),Quaternion(0,0,0,0)),Pose(Point(-0.94868945199,-1.74457276598,0),Quaternion(0,0,0,0)),],[Pose(Point(0.85785154938,-1.18506542227,0),Quaternion(0,0,0,0)),Pose(Point(1.13647644996,-1.66429931096,0),Quaternion(0,0,0,0)),],[Pose(Point(0.741259927745,-0.180241800908,0),Quaternion(0,0,0,0)),],]
    # Greedy
    # pick_rounds_index_list = [[3, 10], [6, 5], [9, 2], [0], [1], [11], [12]]
    # pick_rounds_pose_list = [[Pose(Point(-0.337779258296,-0.0925720981184,0),Quaternion(0,0,0,0)),Pose(Point(-0.673112148581,-0.414992878439,0),Quaternion(0,0,0,0)),],[Pose(Point(0.742259927745,-0.179241800908,0),Quaternion(0,0,0,0)),Pose(Point(1.26465516555,-1.65681052557,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.685037924917,-0.419289319079,0),Quaternion(0,0,0,0)),Pose(Point(0.27066011026,-1.87491735555,0),Quaternion(0,0,0,0)),],[Pose(Point(0.85885154938,-1.18406542227,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.630459353839,-1.33754507522,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.36849904939,-1.85849184156,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.929863789521,-1.75251212121,0),Quaternion(0,0,0,0)),],]
    # pick_rounds_index_list = [[6, 5], [9, 2], [0], [1], [11], [12]]
    # pick_rounds_pose_list = [[Pose(Point(0.742259927745,-0.179241800908,0),Quaternion(0,0,0,0)),Pose(Point(1.26465516555,-1.65681052557,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.685037924917,-0.419289319079,0),Quaternion(0,0,0,0)),Pose(Point(0.27066011026,-1.87491735555,0),Quaternion(0,0,0,0)),],[Pose(Point(0.85885154938,-1.18406542227,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.630459353839,-1.33754507522,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.36849904939,-1.85849184156,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.929863789521,-1.75251212121,0),Quaternion(0,0,0,0)),],]


    # env.combined_move_base_and_arm("youbot_0", arm_drop_joint, rest_base_pose)

    # loaded = False
    # loaded_obj_name = ''
    # result = Queue.Queue()   
    # env.copy_env()
    # next_obj_name = "obj_" + str(pick_rounds_index_list[0][0])
    # next_obj_pick_pose = pick_rounds_pose_list[0][0]
    # get_next_grasp(env, next_obj_name, next_obj_pick_pose, result)

    # for i, pick_index_list in enumerate(pick_rounds_index_list):
    #     for j, index in enumerate(pick_index_list):
    #         print("pick " + str(index))
    #         obj_name = "obj_" + str(index)
            
    #         # next obj
    #         if j < len(pick_index_list) - 1:
    #             next_obj_name = "obj_" + str(pick_rounds_index_list[i][j+1])
    #             next_obj_pick_pose = pick_rounds_pose_list[i][j+1]
    #         else:
    #             if i < len(pick_rounds_index_list) - 1:
    #                 next_obj_name = "obj_" + str(pick_rounds_index_list[i+1][0])
    #                 next_obj_pick_pose = pick_rounds_pose_list[i+1][0]
            
    #         if not (i == 0 and j == 0):
    #             grasp.join()
    #         grasp = threading.Thread(target=get_next_grasp, args=(env, next_obj_name, next_obj_pick_pose, result))
    #         grasp.start()

    #         grasp_plan = result.get()
    #         target_base_pose = grasp_plan.final_base_pose
    #         pick_joint_value = [grasp_plan.q1, grasp_plan.q2, grasp_plan.q3, grasp_plan.q4, grasp_plan.q5]
    #         pre_pick_joint_value = [grasp_plan.q1_pre, grasp_plan.q2_pre, grasp_plan.q3_pre, grasp_plan.q4_pre, grasp_plan.q5_pre]    
            
    #         if loaded:
    #             env.combined_move_base_and_arm_drop_pick("youbot_0", loaded_obj_name, arm_drop_joint, pre_pick_joint_value, target_base_pose)
    #             loaded = False
    #         else:
    #             env.combined_move_base_and_arm_pick("youbot_0", pre_pick_joint_value, target_base_pose)
    #             loaded = True
    #             loaded_obj_name = obj_name

    #         env.pick_object_from_prev("youbot_0", pick_joint_value, pre_pick_joint_value)
    #         env.update_env(obj_name)

    #         if j == len(pick_index_list) - 1:
    #             env.combined_move_base_and_arm_drop("youbot_0", obj_name, arm_drop_joint, rest_base_pose)
    #             loaded = False

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