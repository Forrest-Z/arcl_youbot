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


def compute_path_distance(path):
    distance = 0

    for i in range(len(path) - 1):
        distance += math.sqrt((path[i][0] - path[i+1][0]) ** 2 + (path[i][1] - path[i+1][1]) ** 2)

    return distance


if __name__ == "__main__":
    rospy.init_node("single_youbot_pick_demo")


    # env = app_util.YoubotEnvironment(-1.5, 1.5, -3.0, 1.0)    
    env = app_util.YoubotEnvironment(-3.2, 3.2, 1.0, 6.8, 'youbot_0', 0)
    my_path = os.path.abspath(os.path.dirname(__file__))

    
    env_name = '10'
    # import object list from file
    env.import_obj_from_file(os.path.join(my_path, "scatter/new_12_" + env_name + ".txt"))
    # export object
    # env.create_environment(12, 12)
    # env.export_obj_to_file(os.path.join(my_path, "scatter/new_12_" + env_name + ".txt"))
    

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
    rest_base_pose.orientation.z = 0.7135624
    rest_base_pose.orientation.w = 0.7006335

    arm_drop_joint = [math.radians(169), math.radians(60), math.radians(-190), math.radians(50), math.radians(82)]
    print('=========')
    env_obj_list = deepcopy(env.object_list)
    del env_obj_list['wall']


    # Calculation
    # DP = TSP_DP.DP_solver(env_obj_list)
    # DP.experiment()
    
    # # Generate DP Lists
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
    # # Finish Generating DP Lists

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

    # print('Finish ' + env_name)
    # Finish Generate Greedy Lists

    # test01 DP
    # pick_rounds_index_list = [[7, 2], [3, 5], [6, 4], [0, 11], [9], [10], [8, 1]]
    # pick_rounds_pose_list = [[Pose(Point(1.81515345001,1.41234095156,0),Quaternion(0,0,0,0)),Pose(Point(1.3438517512,1.70425866687,0),Quaternion(0,0,0,0)),],[Pose(Point(1.33750504537,3.34686499243,0),Quaternion(0,0,0,0)),Pose(Point(2.21126065153,3.3468509107,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.549152946529,4.01425531944,0),Quaternion(0,0,0,0)),Pose(Point(-1.59543148346,3.04583602012,0),Quaternion(0,0,0,0)),],[Pose(Point(-1.74547666843,3.50626427205,0),Quaternion(0,0,0,0)),Pose(Point(-1.43375321691,5.74084630534,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.104940592471,2.24144073971,0),Quaternion(0,0,0,0)),],[Pose(Point(0.579542122736,2.94541995417,0),Quaternion(0,0,0,0)),],[Pose(Point(-2.09984021033,4.45100873091,0),Quaternion(0,0,0,0)),Pose(Point(0.287309598889,5.38540767775,0),Quaternion(0,0,0,0)),],]

    # test01 Greedy
    # pick_rounds_index_list = [[2, 7], [9, 0], [4, 6], [10, 1], [3, 5], [8], [11]]
    # pick_rounds_pose_list = [[Pose(Point(0.713218424464,1.34036691542,0),Quaternion(0,0,0,0)),Pose(Point(1.72577829399,1.69216820295,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.106903553999,2.24229771667,0),Quaternion(0,0,0,0)),Pose(Point(-1.57966886113,3.65167773491,0),Quaternion(0,0,0,0)),],[Pose(Point(-1.80455327925,1.86664309695,0),Quaternion(0,0,0,0)),Pose(Point(-0.843498050689,4.08253833585,0),Quaternion(0,0,0,0)),],[Pose(Point(0.580542122736,2.94641995417,0),Quaternion(0,0,0,0)),Pose(Point(0.699684211895,5.04576880904,0),Quaternion(0,0,0,0)),],[Pose(Point(1.33850504537,3.34786499243,0),Quaternion(0,0,0,0)),Pose(Point(2.19184887867,3.70415594983,0),Quaternion(0,0,0,0)),],[Pose(Point(-2.09884021033,4.45200873091,0),Quaternion(0,0,0,0)),],[Pose(Point(-1.22962963218,5.73632653401,0),Quaternion(0,0,0,0)),],]

    # test02 DP
    # pick_rounds_index_list = [[11], [8], [7, 2], [9, 0], [1, 10], [6, 5], [3, 4]]
    # pick_rounds_pose_list = [[Pose(Point(0.394214912751,1.6263256541,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.66250735324,2.89460522378,0),Quaternion(0,0,0,0)),],[Pose(Point(-2.19662620009,4.87265013835,0),Quaternion(0,0,0,0)),Pose(Point(-0.645433434968,6.04674328872,0),Quaternion(0,0,0,0)),],[Pose(Point(2.19978332047,3.00398093032,0),Quaternion(0,0,0,0)),Pose(Point(1.92414812536,1.76609266169,0),Quaternion(0,0,0,0)),],[Pose(Point(-1.63826321673,5.49125247898,0),Quaternion(0,0,0,0)),Pose(Point(-0.436900226748,4.96812798464,0),Quaternion(0,0,0,0)),],[Pose(Point(1.26420063296,2.93943945067,0),Quaternion(0,0,0,0)),Pose(Point(-1.15588731509,2.50824551948,0),Quaternion(0,0,0,0)),],[Pose(Point(2.33551850754,4.83357691938,0),Quaternion(0,0,0,0)),Pose(Point(1.92049723045,4.86483841289,0),Quaternion(0,0,0,0)),],]
    
    # test02 Greedy
    # pick_rounds_index_list = [[11, 0], [5, 6], [8, 1], [9, 2], [10], [4, 3], [7]]
    # pick_rounds_pose_list = [[Pose(Point(0.395214912751,1.6273256541,0),Quaternion(0,0,0,0)),Pose(Point(1.40903048554,1.56086850468,0),Quaternion(0,0,0,0)),],[Pose(Point(-1.32806943454,2.01942775531,0),Quaternion(0,0,0,0)),Pose(Point(1.0254291717,3.20477248931,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.66150735324,2.89560522378,0),Quaternion(0,0,0,0)),Pose(Point(-1.6049207936,5.50331944721,0),Quaternion(0,0,0,0)),],[Pose(Point(2.20078332047,3.00498093032,0),Quaternion(0,0,0,0)),Pose(Point(0.317373476154,5.94168847441,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.076850074596,4.29115755118,0),Quaternion(0,0,0,0)),],[Pose(Point(1.41522386607,4.23913730345,0),Quaternion(0,0,0,0)),Pose(Point(2.00410260918,5.11942291196,0),Quaternion(0,0,0,0)),],[Pose(Point(-2.19562620009,4.87365013835,0),Quaternion(0,0,0,0)),],]

    # test03 DP
    # pick_rounds_index_list = [[10, 3], [8, 11], [7, 5], [0, 4], [6, 9], [1, 2]]
    # pick_rounds_pose_list = [[Pose(Point(-2.36336232281,1.51539280538,0),Quaternion(0,0,0,0)),Pose(Point(-2.72567837718,2.20576001824,0),Quaternion(0,0,0,0)),],[Pose(Point(-2.38702244783,4.73280035847,0),Quaternion(0,0,0,0)),Pose(Point(-2.38702244783,4.73280035847,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.65199407712,1.50990235196,0),Quaternion(0,0,0,0)),Pose(Point(0.85438793292,3.13276351821,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.783790772925,4.5713286382,0),Quaternion(0,0,0,0)),Pose(Point(-0.973114986242,4.02739646886,0),Quaternion(0,0,0,0)),],[Pose(Point(0.613599329652,1.48045635041,0),Quaternion(0,0,0,0)),Pose(Point(2.20292178062,2.38507244624,0),Quaternion(0,0,0,0)),],[Pose(Point(1.83048715527,4.11958275823,0),Quaternion(0,0,0,0)),Pose(Point(1.83048715527,4.11958275823,0),Quaternion(0,0,0,0)),],]

    # test03 Greedy
    # pick_rounds_index_list = [[6, 5], [7, 3], [10, 8], [9, 2], [4, 0], [1], [11]]
    # pick_rounds_pose_list = [[Pose(Point(0.614599329652,1.48145635041,0),Quaternion(0,0,0,0)),Pose(Point(1.08623140832,2.97379637935,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.65099407712,1.51090235196,0),Quaternion(0,0,0,0)),Pose(Point(-2.32358265375,2.48258978591,0),Quaternion(0,0,0,0)),],[Pose(Point(-2.27158582963,1.15848955576,0),Quaternion(0,0,0,0)),Pose(Point(-2.58783049576,3.81343072749,0),Quaternion(0,0,0,0)),],[Pose(Point(2.30032411677,2.23097088957,0),Quaternion(0,0,0,0)),Pose(Point(2.13116306695,4.10212848435,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.905885974981,3.18790655371,0),Quaternion(0,0,0,0)),Pose(Point(-0.90691934894,4.56923470398,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.00260485625211,3.79714567101,0),Quaternion(0,0,0,0)),],[Pose(Point(-2.38602244783,4.73380035847,0),Quaternion(0,0,0,0)),],]

    # test04 DP
    # pick_rounds_index_list = [[10, 9], [4, 11], [5, 7], [3], [6], [1], [8], [2, 0]]
    # pick_rounds_pose_list = [[Pose(Point(-0.460543032872,2.35271974539,0),Quaternion(0,0,0,0)),Pose(Point(-0.75827695251,3.87371652877,0),Quaternion(0,0,0,0)),],[Pose(Point(-2.69402055194,4.66602906589,0),Quaternion(0,0,0,0)),Pose(Point(-2.24531612727,5.2810257138,0),Quaternion(0,0,0,0)),],[Pose(Point(0.71659119022,2.04155408675,0),Quaternion(0,0,0,0)),Pose(Point(0.685475062793,4.71025615581,0),Quaternion(0,0,0,0)),],[Pose(Point(1.80421803006,3.39798820939,0),Quaternion(0,0,0,0)),],[Pose(Point(2.26368867847,1.87217191614,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.0805118398821,3.36792250658,0),Quaternion(0,0,0,0)),],[Pose(Point(-1.10383146973,1.47987966212,0),Quaternion(0,0,0,0)),],[Pose(Point(1.72498543581,5.06592150166,0),Quaternion(0,0,0,0)),Pose(Point(-1.29459756471,3.47236054474,0),Quaternion(0,0,0,0)),],]

    # test04 Greedy
    # pick_rounds_index_list = [[10, 5], [8], [6], [0, 11], [1], [3], [9], [7], [2, 4]]
    # pick_rounds_pose_list = [[Pose(Point(-0.170798392727,1.78244835684,0),Quaternion(0,0,0,0)),Pose(Point(0.440209349642,2.35516142136,0),Quaternion(0,0,0,0)),],[Pose(Point(-1.10283146973,1.48087966212,0),Quaternion(0,0,0,0)),],[Pose(Point(2.26468867847,1.87317191614,0),Quaternion(0,0,0,0)),],[Pose(Point(-1.46108472924,2.86039962053,0),Quaternion(0,0,0,0)),Pose(Point(-1.59235542832,4.94027119185,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.0795118398821,3.36892250658,0),Quaternion(0,0,0,0)),],[Pose(Point(1.80521803006,3.39898820939,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.75727695251,3.87471652877,0),Quaternion(0,0,0,0)),],[Pose(Point(0.599427880603,4.71342325247,0),Quaternion(0,0,0,0)),],[Pose(Point(2.2072469857,4.84542448508,0),Quaternion(0,0,0,0)),Pose(Point(-2.43015531684,5.49817363052,0),Quaternion(0,0,0,0)),],]

    # test05 DP
    # pick_rounds_index_list = [[10, 7], [4], [11, 9], [2], [8, 6], [5, 3], [1, 0]]
    # pick_rounds_pose_list = [[Pose(Point(-2.00663430089,3.71707330454,0),Quaternion(0,0,0,0)),Pose(Point(-0.500243989529,4.21752962323,0),Quaternion(0,0,0,0)),],[Pose(Point(-1.52963436983,2.09030377547,0),Quaternion(0,0,0,0)),],[Pose(Point(-1.30618014471,5.84247674397,0),Quaternion(0,0,0,0)),Pose(Point(0.416275450619,5.00455642795,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.913330117563,1.41046337719,0),Quaternion(0,0,0,0)),],[Pose(Point(2.27286808544,4.68832666911,0),Quaternion(0,0,0,0)),Pose(Point(0.551759737732,6.09190171916,0),Quaternion(0,0,0,0)),],[Pose(Point(0.498843480347,2.13416639486,0),Quaternion(0,0,0,0)),Pose(Point(1.31668095058,5.6330619687,0),Quaternion(0,0,0,0)),],[Pose(Point(1.78854772278,2.76226924679,0),Quaternion(0,0,0,0)),Pose(Point(-0.636598982679,4.46404887348,0),Quaternion(0,0,0,0)),],]

    # test05 Greedy
    # pick_rounds_index_list = [[2, 5], [4, 9], [1, 8], [7, 10], [0], [3], [6], [11]]    
    # pick_rounds_pose_list = [[Pose(Point(-0.912330117563,1.41146337719,0),Quaternion(0,0,0,0)),Pose(Point(0.501005726238,2.3138403173,0),Quaternion(0,0,0,0)),],[Pose(Point(-1.52863436983,2.09130377547,0),Quaternion(0,0,0,0)),Pose(Point(0.65068782937,4.10661383291,0),Quaternion(0,0,0,0)),],[Pose(Point(1.78954772278,2.76326924679,0),Quaternion(0,0,0,0)),Pose(Point(2.36726617628,4.65669619477,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.0631151144717,3.81919708441,0),Quaternion(0,0,0,0)),Pose(Point(-1.78021946545,4.11297249802,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.927507510973,4.23863207598,0),Quaternion(0,0,0,0)),],[Pose(Point(1.31768095058,5.6340619687,0),Quaternion(0,0,0,0)),],[Pose(Point(0.161476284961,5.86521109782,0),Quaternion(0,0,0,0)),],[Pose(Point(-1.30518014471,5.84347674397,0),Quaternion(0,0,0,0)),],]

    # test06 DP
    # pick_rounds_index_list = [[7, 2], [10, 1], [9], [6, 4], [0, 5], [3, 8], [11]]
    # pick_rounds_pose_list = [[Pose(Point(1.72354698731,2.77159060848,0),Quaternion(0,0,0,0)),Pose(Point(2.23152719457,4.74626063296,0),Quaternion(0,0,0,0)),],[Pose(Point(0.112027618271,2.97686497542,0),Quaternion(0,0,0,0)),Pose(Point(0.204955546254,4.57949528652,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.342600165809,1.20031436478,0),Quaternion(0,0,0,0)),],[Pose(Point(0.613555932773,3.80782567598,0),Quaternion(0,0,0,0)),Pose(Point(-0.174051797108,4.27940230394,0),Quaternion(0,0,0,0)),],[Pose(Point(-1.36677852399,5.11343378402,0),Quaternion(0,0,0,0)),Pose(Point(-0.0181001252346,6.06902546959,0),Quaternion(0,0,0,0)),],[Pose(Point(-1.59112129764,2.54729954284,0),Quaternion(0,0,0,0)),Pose(Point(0.508792807648,2.36253376775,0),Quaternion(0,0,0,0)),],[Pose(Point(-1.47695270577,1.27608464513,0),Quaternion(0,0,0,0)),],]
    
    # test06 Greedy
    # pick_rounds_index_list = [[9, 1], [11, 0], [8, 3], [10], [7, 2], [6, 4], [5]]
    # pick_rounds_pose_list = [[Pose(Point(-0.341600165809,1.20131436478,0),Quaternion(0,0,0,0)),Pose(Point(0.147951598687,4.58828227499,0),Quaternion(0,0,0,0)),],[Pose(Point(-1.47595270577,1.27708464513,0),Quaternion(0,0,0,0)),Pose(Point(-1.511833348,5.10127830598,0),Quaternion(0,0,0,0)),],[Pose(Point(0.765296414876,1.86683536631,0),Quaternion(0,0,0,0)),Pose(Point(-1.3994255374,2.81421134784,0),Quaternion(0,0,0,0)),],[Pose(Point(0.377816194134,2.81338079693,0),Quaternion(0,0,0,0)),],[Pose(Point(1.9191977306,2.39889023907,0),Quaternion(0,0,0,0)),Pose(Point(2.28553906863,4.74136673421,0),Quaternion(0,0,0,0)),],[Pose(Point(0.614555932773,3.80882567598,0),Quaternion(0,0,0,0)),Pose(Point(-0.173051797108,4.28040230394,0),Quaternion(0,0,0,0)),],[Pose(Point(0.359588988212,5.79008141936,0),Quaternion(0,0,0,0)),],]
    
    # test07 DP
    # pick_rounds_index_list = [[7, 3], [6, 4], [5], [8, 2], [9, 1], [0, 10], [11]]
    # pick_rounds_pose_list = [[Pose(Point(-0.592853620949,3.02185334237,0),Quaternion(0,0,0,0)),Pose(Point(-1.17753580868,2.71522798281,0),Quaternion(0,0,0,0)),],[Pose(Point(1.99637166766,3.42998471583,0),Quaternion(0,0,0,0)),Pose(Point(0.496052534263,4.84551688368,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.0891349114299,1.58224238256,0),Quaternion(0,0,0,0)),],[Pose(Point(-1.01882417313,5.22620717306,0),Quaternion(0,0,0,0)),Pose(Point(0.971220533162,5.69130106785,0),Quaternion(0,0,0,0)),],[Pose(Point(0.867664079136,3.46749184749,0),Quaternion(0,0,0,0)),Pose(Point(1.22452658723,2.76133761317,0),Quaternion(0,0,0,0)),],[Pose(Point(-2.26780542969,4.4975830868,0),Quaternion(0,0,0,0)),Pose(Point(-2.33865801611,3.79656814648,0),Quaternion(0,0,0,0)),],[Pose(Point(-2.14806238681,1.33624570202,0),Quaternion(0,0,0,0)),],]
    
    # test07 Greedy
    # pick_rounds_index_list =[[5, 3], [1, 9], [11, 0], [7, 4], [10], [6, 2], [8]]
    # pick_rounds_pose_list = [[Pose(Point(-0.0881349114299,1.58324238256,0),Quaternion(0,0,0,0)),Pose(Point(-1.00504844761,2.25111009979,0),Quaternion(0,0,0,0)),],[Pose(Point(1.10882311393,1.96813396882,0),Quaternion(0,0,0,0)),Pose(Point(1.0673423768,3.46609787211,0),Quaternion(0,0,0,0)),],[Pose(Point(-2.14706238681,1.33724570202,0),Quaternion(0,0,0,0)),Pose(Point(-2.47568814294,4.45280700509,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.480060855369,2.99569052038,0),Quaternion(0,0,0,0)),Pose(Point(-0.0337879171098,4.6722093923,0),Quaternion(0,0,0,0)),],[Pose(Point(-2.04758326204,3.01467615413,0),Quaternion(0,0,0,0)),],[Pose(Point(1.99737166766,3.43098471583,0),Quaternion(0,0,0,0)),Pose(Point(1.97070800825,5.25482944357,0),Quaternion(0,0,0,0)),],[Pose(Point(-1.01782417313,5.22720717306,0),Quaternion(0,0,0,0)),],]
    
    # test08 DP
    # pick_rounds_index_list = [[8, 3], [4, 7], [11, 0], [1, 10], [6, 5], [2, 9]]
    # pick_rounds_pose_list = [[Pose(Point(-1.94598412576,1.47362759804,0),Quaternion(0,0,0,0)),Pose(Point(-1.86721270196,3.34662682341,0),Quaternion(0,0,0,0)),],[Pose(Point(1.86493341656,2.49802521727,0),Quaternion(0,0,0,0)),Pose(Point(0.966771621992,4.75794215662,0),Quaternion(0,0,0,0)),],[Pose(Point(0.024667617481,1.72506365439,0),Quaternion(0,0,0,0)),Pose(Point(0.86658542104,1.83651367779,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.798658789979,5.38006782065,0),Quaternion(0,0,0,0)),Pose(Point(-2.06882619239,5.13871035361,0),Quaternion(0,0,0,0)),],[Pose(Point(1.43811778845,3.43234803733,0),Quaternion(0,0,0,0)),Pose(Point(0.856028464103,3.48247062537,0),Quaternion(0,0,0,0)),],[Pose(Point(-1.69726928869,2.53664414763,0),Quaternion(0,0,0,0)),Pose(Point(1.30264982342,5.37410768636,0),Quaternion(0,0,0,0)),],]

    # test08 Greedy
    # pick_rounds_index_list = [[0, 4], [11], [8, 2], [5, 6], [3, 1], [7], [10], [9]]
    # pick_rounds_pose_list = [[Pose(Point(1.18516214701,1.15464035694,0),Quaternion(0,0,0,0)),Pose(Point(1.94554368132,2.45199716246,0),Quaternion(0,0,0,0)),],[Pose(Point(0.025667617481,1.72606365439,0),Quaternion(0,0,0,0)),],[Pose(Point(-1.94498412576,1.47462759804,0),Quaternion(0,0,0,0)),Pose(Point(-2.26392411955,2.47537777968,0),Quaternion(0,0,0,0)),],[Pose(Point(0.406757896511,2.87700980704,0),Quaternion(0,0,0,0)),Pose(Point(1.21508752235,3.64926581542,0),Quaternion(0,0,0,0)),],[Pose(Point(-1.57544470683,3.37410360945,0),Quaternion(0,0,0,0)),Pose(Point(-1.03363749613,5.41267214976,0),Quaternion(0,0,0,0)),],[Pose(Point(0.22736111442,4.66327502216,0),Quaternion(0,0,0,0)),],[Pose(Point(-2.25318591685,4.54657544745,0),Quaternion(0,0,0,0)),],[Pose(Point(1.52463755032,5.22372923881,0),Quaternion(0,0,0,0)),],]

    # test09 DP
    # pick_rounds_index_list = [[3, 8], [7, 4], [2, 9], [10, 1], [0, 11], [6, 5]]
    # pick_rounds_pose_list = [[Pose(Point(-1.11481952042,4.01474265865,0),Quaternion(0,0,0,0)),Pose(Point(0.590373384992,5.26024525943,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.933234151761,2.90410494389,0),Quaternion(0,0,0,0)),Pose(Point(-1.93031158741,5.75548580005,0),Quaternion(0,0,0,0)),],[Pose(Point(1.27492508859,2.30674470561,0),Quaternion(0,0,0,0)),Pose(Point(1.51736146189,2.2464719186,0),Quaternion(0,0,0,0)),],[Pose(Point(0.082917199255,3.49031627102,0),Quaternion(0,0,0,0)),Pose(Point(-0.781043500965,1.83153548802,0),Quaternion(0,0,0,0)),],[Pose(Point(1.78214853082,3.80667142112,0),Quaternion(0,0,0,0)),Pose(Point(2.21133331707,5.40881248976,0),Quaternion(0,0,0,0)),],[Pose(Point(-2.02353927673,2.16542724771,0),Quaternion(0,0,0,0)),Pose(Point(-0.61319609758,4.97872515212,0),Quaternion(0,0,0,0)),],]

    # test09 Greedy
    # pick_rounds_index_list = [[1, 6], [9, 2], [7, 3], [10, 0], [5, 4], [8], [11]]
    # pick_rounds_pose_list = [[Pose(Point(-0.709189471692,1.0769983867,0),Quaternion(0,0,0,0)),Pose(Point(-1.97419622138,2.21986693528,0),Quaternion(0,0,0,0)),],[Pose(Point(1.32250605039,1.03208498567,0),Quaternion(0,0,0,0)),Pose(Point(1.53852174707,2.25756826087,0),Quaternion(0,0,0,0)),],[Pose(Point(-1.27987916938,2.73864515659,0),Quaternion(0,0,0,0)),Pose(Point(-1.66514457284,3.81111307953,0),Quaternion(0,0,0,0)),],[Pose(Point(0.083917199255,3.49131627102,0),Quaternion(0,0,0,0)),Pose(Point(1.54006712875,4.13438494411,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.340129747411,4.90604010425,0),Quaternion(0,0,0,0)),Pose(Point(-1.50123673367,6.13015501512,0),Quaternion(0,0,0,0)),],[Pose(Point(0.906156685398,5.01669191799,0),Quaternion(0,0,0,0)),],[Pose(Point(2.1404796198,5.43110391861,0),Quaternion(0,0,0,0)),],]

    # test10 DP
    # pick_rounds_index_list = [[8, 3], [9], [2, 1], [11, 0], [7, 4], [5, 6], [10]]
    # pick_rounds_pose_list = [[Pose(Point(2.06012786779,4.38845467558,0),Quaternion(0,0,0,0)),Pose(Point(-0.283366430847,5.82898954999,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.106633814463,1.81053031132,0),Quaternion(0,0,0,0)),],[Pose(Point(-1.98245523384,4.99671625016,0),Quaternion(0,0,0,0)),Pose(Point(-1.92459732076,4.89369775468,0),Quaternion(0,0,0,0)),],[Pose(Point(1.12304016773,5.56135880135,0),Quaternion(0,0,0,0)),Pose(Point(0.11971245678,3.87172798092,0),Quaternion(0,0,0,0)),],[Pose(Point(1.69515559687,3.34421287608,0),Quaternion(0,0,0,0)),Pose(Point(-0.895403355364,2.64891155414,0),Quaternion(0,0,0,0)),],[Pose(Point(0.892369121232,3.66905030168,0),Quaternion(0,0,0,0)),Pose(Point(2.09176555361,2.47767904339,0),Quaternion(0,0,0,0)),],[Pose(Point(0.571216387072,1.02877358027,0),Quaternion(0,0,0,0)),],]

    # test10 Greedy
    pick_rounds_index_list = [[10, 0], [9, 1], [4, 5], [6, 3], [7, 2], [8], [11]]
    pick_rounds_pose_list = [[Pose(Point(0.572216387072,1.02977358027,0),Quaternion(0,0,0,0)),Pose(Point(0.0871951676616,3.09284488766,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.105633814463,1.81153031132,0),Quaternion(0,0,0,0)),Pose(Point(-1.50269300467,4.15050830637,0),Quaternion(0,0,0,0)),],[Pose(Point(-1.09702328887,2.11306374447,0),Quaternion(0,0,0,0)),Pose(Point(0.598675941237,3.90034517528,0),Quaternion(0,0,0,0)),],[Pose(Point(2.01695479541,1.83417175077,0),Quaternion(0,0,0,0)),Pose(Point(-0.415198262497,5.65591023522,0),Quaternion(0,0,0,0)),],[Pose(Point(2.0109448358,3.07682000901,0),Quaternion(0,0,0,0)),Pose(Point(-1.58590452149,5.51689991292,0),Quaternion(0,0,0,0)),],[Pose(Point(2.06112786779,4.38945467558,0),Quaternion(0,0,0,0)),],[Pose(Point(1.12404016773,5.56235880135,0),Quaternion(0,0,0,0)),],]

    # env.move_arm_to_joint_pose("youbot_0", arm_drop_joint)

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

    #         # pick_joint_value = [env.grasp_plan_result.q1, env.grasp_plan_result.q2, env.grasp_plan_result.q3, env.grasp_plan_result.q4, env.grasp_plan_result.q5]
    #         # pre_pick_joint_value = [env.grasp_plan_result.q1_pre, env.grasp_plan_result.q2_pre, env.grasp_plan_result.q3_pre, env.grasp_plan_result.q4_pre, env.grasp_plan_result.q5_pre]   
    #         # one_round_pick.append(pick_joint_value)
    #         # one_round_pre.append(pre_pick_joint_value)
    #         # arm_pick_path = env.get_pick_object_path('youbot_0', pick_joint_value, pre_pick_joint_value, target_base_pose, arm_drop_joint)
    #         # one_round_arm_pick_path.append(arm_pick_path)
    #         env.update_env(obj_name)
    #         # state, drop_path = env.get_move_arm_to_joint_pose_path('youbot_0', target_base_pose, pre_pick_joint_value, arm_drop_joint)
    #         # if state == False: 
    #         #     print("move_arm_to_joint_return_false")
    #         #     exit(-1)
    #         # one_round_drop_path.append(drop_path)

    #     return_path, _ = env.get_path('youbot_0', start_pose, rest_base_pose)

    #     paths.append(one_round_path)
    #     return_paths.append(return_path)
    #     pick_joint_values.append(one_round_pick)
    #     pre_pick_joint_values.append(one_round_pre)
    #     arm_paths.append(one_round_arm_pick_path)
    #     drop_paths.append(one_round_drop_path)

    # # save data
    # with open('new_12_01_greedy.pkl', 'wb') as output:
    #     pickle.dump((paths, return_paths, pick_joint_values, pre_pick_joint_values, arm_paths, drop_paths), output, pickle.HIGHEST_PROTOCOL)
    # ===== pre-compute =====

    # ===== executaion =====
    # read data
    # with open('new_12_09_greedy.pkl', 'rb') as input:
    #     paths, return_paths, pick_joint_values, pre_pick_joint_values, arm_paths, drop_paths = pickle.load(input)

    # for i, pick_index_list in enumerate(pick_rounds_index_list):
    #     for j, index in enumerate(pick_index_list):
    #         print("pick " + str(index))
    #         obj_name = "obj_" + str(index)
    #         env.follow_path("youbot_0", paths[i][j])
            
    #         # env.follow_pick_object_path("youbot_0", arm_paths[i][j])
    #         # env.follow_move_arm_to_joint_pose_path("youbot_0", drop_paths[i][j])
    #         env.drop_object("youbot_0", obj_name)

    #     env.follow_path("youbot_0", return_paths[i])
    # ===== executaion =====

    # ===== compute =====
    # total_distance = 0

    # with open('new_12_10_greedy.pkl', 'rb') as input:
    #     paths, return_paths, pick_joint_values, pre_pick_joint_values, arm_paths, drop_paths = pickle.load(input)

    # for i, pick_index_list in enumerate(pick_rounds_index_list):
    #     for j, index in enumerate(pick_index_list):
    #         total_distance += compute_path_distance(paths[i][j])
    #     total_distance += compute_path_distance(return_paths[i])

    # print(total_distance)
    # ===== compute =====



    # for i, pick_index_list in enumerate(pick_rounds_index_list):

    #     for j, index in enumerate(pick_index_list):
    #         print("pick " + str(index))
    #         obj_name = "obj_" + str(index)
    #         env.send_grasp_action(env.planning_scene_msg, obj_name, env.reserved_planning_scene_msg.scene_object_list[env.obj_name_to_index_dict[obj_name]].object_pose, " ", "cube", pick_rounds_pose_list[i][j], True)
    #         target_base_pose = env.grasp_plan_result.final_base_pose
    #         env.move_to_target("youbot_0", target_base_pose)
            
    #         pick_joint_value = [env.grasp_plan_result.q1, env.grasp_plan_result.q2, env.grasp_plan_result.q3, env.grasp_plan_result.q4, env.grasp_plan_result.q5]
    #         pre_pick_joint_value = [env.grasp_plan_result.q1_pre, env.grasp_plan_result.q2_pre, env.grasp_plan_result.q3_pre, env.grasp_plan_result.q4_pre, env.grasp_plan_result.q5_pre]    
    #         # env.pick_object("youbot_0", pick_joint_value, pre_pick_joint_value)
    #         env.update_env(obj_name)
    #         # env.move_arm_to_joint_pose("youbot_0", arm_drop_joint)
    #         env.drop_object("youbot_0", obj_name)

    #     env.move_to_target("youbot_0", rest_base_pose)

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
