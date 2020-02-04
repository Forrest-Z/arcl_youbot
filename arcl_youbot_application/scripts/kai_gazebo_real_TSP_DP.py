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


base_pose = Pose()

def position_callback(self, data):
    global base_pose
    base_pose.position.x = data.point.x
    base_pose.position.y = data.point.y

if __name__ == "__main__":
    rospy.init_node("single_youbot_pick_demo")
    rospy.Subscriber('/youbot_0/robot/pose', PointStamped, position_callback)

    
    
    
    # env = app_util.YoubotEnvironment(-1.5, 1.5, -3.0, 1.0)    
    env = app_util.YoubotEnvironment(-3.2, 3.2, 1.0, 6.8, 'youbot_0', 0)
    my_path = os.path.abspath(os.path.dirname(__file__))

    
    #import object list from file
    # env.import_obj_from_file(os.path.join(my_path, "scatter/new_12.txt"))


    
    # export object
    # env.create_environment(12, 12)
    # env.export_obj_to_file(os.path.join(my_path, "scatter/new_12_05.txt"))
    
    env.object_list = {'obj_9': [(-0.7384098156046233, -1.0396410612672276), (-0.7532900066292016, -1.0767702938888701), (-0.5147346870351472, -1.1723755212217861), (-0.49985449601056886, -1.1352462886001435)], 
    'obj_3': [(0.8681416502412339, -1.3298824568574468), (0.8897227300617275, -1.2962037247883578), (0.6893342742506484, -1.1677962998564204), (0.6677531944301548, -1.2014750319255094)], 
    'obj_2': [(-0.6103882592350371, -1.3523035966600205), (-0.6482229549393665, -1.3393214627460297), (-0.7082653242915742, -1.5143069303785537), (-0.6704306285872448, -1.5272890642925445)], 
    'obj_1': [(0.6404971576972914, -0.9485426187551826), (0.624945858132787, -0.9116894359638003), (0.40659075009384626, -1.0038308858834892), (0.42214204965835067, -1.0406840686748715)], 
    'obj_7': [(-1.0500325893982907, -0.204718788691302), (-1.0224167219502436, -0.17578154858289008), (-1.1287610793486575, -0.07429323571131641), (-1.1563769467967047, -0.10323047581972833)],
    'obj_6': [(-0.8967840249173443, -0.24952558153545096), (-0.8570117762190462, -0.24526316045339824), (-0.8857831185229023, 0.02319951826011374), (-0.9255553672212004, 0.018937097178061024)], 
    'obj_5': [(0.4863222365387207, -1.389403936246166), (0.5227808064606629, -1.4058591080546536), (0.591892528056311, -1.252733114382496), (0.5554339581343688, -1.2362779425740085)], 
    'obj_4': [(-0.7672793580585098, -1.2911032135537945), (-0.8049820797942624, -1.2777426456872552), (-0.8550842092937851, -1.4191278521963275), (-0.8173814875580325, -1.432488420062867)], 
    'obj_12': [(0.7707273046161855, 0.03395589672776249), (0.8103686708622331, 0.03930024721652463), (0.7843149622295176, 0.2325519076660058), (0.74467359598347, 0.22720755717724367)], 
    'obj_11': [(0.7656518559753612, -0.2765270004628717), (0.8056381553012718, -0.27548016417257803), (0.8004301447570606, -0.07654832502617245), (0.7604438454311501, -0.07759516131646613)]}

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

    # test01 DP
    # pick_rounds_index_list = [[7], [2], [8, 3], [6, 9], [4, 1], [0, 10], [11, 5]]
    # pick_rounds_pose_list = [[Pose(Point(-1.78615398398,1.97445096182,0),Quaternion(0,0,0,0)),],[Pose(Point(1.67148493575,4.62082743612,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.253960395213,2.62870244346,0),Quaternion(0,0,0,0)),Pose(Point(0.49865911551,4.69498719299,0),Quaternion(0,0,0,0)),],[Pose(Point(2.69807608809,4.36016202211,0),Quaternion(0,0,0,0)),Pose(Point(2.08977933794,4.87066522389,0),Quaternion(0,0,0,0)),],[Pose(Point(-1.51654328104,3.86612464136,0),Quaternion(0,0,0,0)),Pose(Point(-1.60433902118,4.16621310223,0),Quaternion(0,0,0,0)),],[Pose(Point(2.56233679521,3.21358061961,0),Quaternion(0,0,0,0)),Pose(Point(0.156686223061,4.38104390015,0),Quaternion(0,0,0,0)),],[Pose(Point(1.38456999337,1.28528101564,0),Quaternion(0,0,0,0)),Pose(Point(-2.40840519644,3.05042207717,0),Quaternion(0,0,0,0))]]

    # test01 Greedy
    # pick_rounds_index_list = [[11, 8], [7, 4], [5, 10], [0], [9, 6], [1], [2], [3]]
    # pick_rounds_pose_list = [[Pose(Point(1.38556999337,1.28628101564,0),Quaternion(0,0,0,0)),Pose(Point(0.0511292686526,2.80164047031,0),Quaternion(0,0,0,0)),],[Pose(Point(-1.78515398398,1.97545096182,0),Quaternion(0,0,0,0)),Pose(Point(-1.97662416306,3.96337256007,0),Quaternion(0,0,0,0)),],[Pose(Point(-2.52805093003,2.87376899381,0),Quaternion(0,0,0,0)),Pose(Point(-0.556982957819,4.20564669577,0),Quaternion(0,0,0,0)),],[Pose(Point(2.56333679521,3.21458061961,0),Quaternion(0,0,0,0)),],[Pose(Point(0.8153165657,4.43913099744,0),Quaternion(0,0,0,0)),Pose(Point(2.62128766943,4.42276034135,0),Quaternion(0,0,0,0)),],[Pose(Point(-1.68495417345,4.12300837552,0),Quaternion(0,0,0,0)),],[Pose(Point(1.67248493575,4.62182743612,0),Quaternion(0,0,0,0)),],[Pose(Point(0.629651658824,4.65680646034,0),Quaternion(0,0,0,0))]]

    # test02 DP
    # pick_rounds_index_list = [[8, 10], [2, 4], [3], [6, 5], [1], [9], [0, 7], [11]]
    # pick_rounds_pose_list = [[Pose(Point(1.33458761699,3.39477459032,0),Quaternion(0,0,0,0)),Pose(Point(-1.12081931074,4.96855616488,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.58246295617,0.984517434197,0),Quaternion(0,0,0,0)),Pose(Point(-0.821858857621,1.38618188465,0),Quaternion(0,0,0,0)),],[Pose(Point(2.17523999594,2.92263517055,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.237455456703,1.91182356571,0),Quaternion(0,0,0,0)),Pose(Point(-1.71652510022,3.13768709172,0),Quaternion(0,0,0,0)),],[Pose(Point(-2.06802702393,1.8356138129,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.447791733115,2.60788887677,0),Quaternion(0,0,0,0)),],[Pose(Point(-1.45498173641,4.18264179764,0),Quaternion(0,0,0,0)),Pose(Point(-1.70251238751,4.6378402703,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.187517471666,3.73327064142,0),Quaternion(0,0,0,0))]]

    # test02 Greedy
    # pick_rounds_index_list = [[2, 4], [6, 5], [9, 10], [1], [3], [8], [11], [7, 0]]
    # pick_rounds_pose_list = [[Pose(Point(-0.58146295617,0.985517434197,0),Quaternion(0,0,0,0)),Pose(Point(-1.40277913913,1.50533129288,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.236455456703,1.91282356571,0),Quaternion(0,0,0,0)),Pose(Point(-1.71566379295,3.13843978581,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.446791733115,2.60888887677,0),Quaternion(0,0,0,0)),Pose(Point(-1.39210072447,4.73281979936,0),Quaternion(0,0,0,0)),],[Pose(Point(-2.06702702393,1.8366138129,0),Quaternion(0,0,0,0)),],[Pose(Point(2.12891813178,2.96452508614,0),Quaternion(0,0,0,0)),],[Pose(Point(1.18885679904,3.49190464811,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.512144550687,3.85967332063,0),Quaternion(0,0,0,0)),],[Pose(Point(-1.61518652139,4.15450106549,0),Quaternion(0,0,0,0)),Pose(Point(-1.20487679918,4.40948761767,0),Quaternion(0,0,0,0))]]
    
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
    # pick_rounds_index_list = [[4, 2], [3], [6, 9], [1, 11], [10], [0, 8], [5, 7]]
    # pick_rounds_pose_list = [[Pose(Point(0.87289546037,3.97790896624,0),Quaternion(0,0,0,0)),Pose(Point(1.30177587504,5.63682813265,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.913330117561,1.41046337719,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.0641151144716,3.81819708441,0),Quaternion(0,0,0,0)),Pose(Point(-0.625721121917,4.48732402761,0),Quaternion(0,0,0,0)),],[Pose(Point(-1.55611907349,2.88254250622,0),Quaternion(0,0,0,0)),Pose(Point(-2.00663430089,3.71707330455,0),Quaternion(0,0,0,0)),],[Pose(Point(-1.30618014471,5.84247674397,0),Quaternion(0,0,0,0)),],[Pose(Point(1.09324510486,1.93827630362,0),Quaternion(0,0,0,0)),Pose(Point(1.73715600798,2.80131557965,0),Quaternion(0,0,0,0)),],[Pose(Point(2.27286808544,4.68832666911,0),Quaternion(0,0,0,0)),Pose(Point(0.551759737732,6.09190171916,0),Quaternion(0,0,0,0)),],]

    # test05 Greedy
    # pick_rounds_index_list = [[3, 11], [0, 8], [1, 4], [6, 9], [5, 7], [2], [10]]
    # pick_rounds_pose_list = [[Pose(Point(-0.912330117561,1.41146337719,0),Quaternion(0,0,0,0)),Pose(Point(-1.89877130166,3.7996451983,0),Quaternion(0,0,0,0)),],[Pose(Point(0.759842265951,2.04500949712,0),Quaternion(0,0,0,0)),Pose(Point(1.65567957439,2.9010817949,0),Quaternion(0,0,0,0)),],[Pose(Point(-1.52863436983,2.09130377548,0),Quaternion(0,0,0,0)),Pose(Point(0.650687829368,4.10661383291,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.0631151144716,3.81919708441,0),Quaternion(0,0,0,0)),Pose(Point(-0.624721121917,4.48832402761,0),Quaternion(0,0,0,0)),],[Pose(Point(2.27386808544,4.68932666911,0),Quaternion(0,0,0,0)),Pose(Point(0.505244787478,6.0217881293,0),Quaternion(0,0,0,0)),],[Pose(Point(1.31768095058,5.6340619687,0),Quaternion(0,0,0,0)),],[Pose(Point(-1.30518014471,5.84347674397,0),Quaternion(0,0,0,0)),],]

    # DP
    pick_rounds_index_list = [[1, 4], [3], [9, 7], [0, 2], [11]]
    pick_rounds_pose_list = [[Pose(Point(-0.914791571956,-1.23610458707,0),Quaternion(0,0,0,0)),Pose(Point(-1.26156095048,-1.3406888417,0),Quaternion(0,0,0,0)),],[Pose(Point(0.688094169922,-0.640470002986,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.238207818572,-0.4707302894,0),Quaternion(0,0,0,0)),Pose(Point(-0.724025674537,-0.239715612576,0),Quaternion(0,0,0,0)),],[Pose(Point(0.377376805186,-1.2425973245,0),Quaternion(0,0,0,0)),Pose(Point(0.124605874977,-1.64542909091,0),Quaternion(0,0,0,0)),],[Pose(Point(0.349563695275,-0.0498377792525,0),Quaternion(0,0,0,0)),],]
    # Greedy
    # pick_rounds_index_list = [[11, 7], [9, 2], [3, 4], [0], [1]]
    # pick_rounds_pose_list = [[Pose(Point(0.350563695275,-0.0488377792525,0),Quaternion(0,0,0,0)),Pose(Point(-0.482370217199,0.0843160078989,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.092780059743,-0.4685054472,0),Quaternion(0,0,0,0)),Pose(Point(-0.231817872356,-1.21077754286,0),Quaternion(0,0,0,0)),],[Pose(Point(0.689094169922,-0.639470002986,0),Quaternion(0,0,0,0)),Pose(Point(-0.982388536478,-0.936504418969,0),Quaternion(0,0,0,0)),],[Pose(Point(0.434110713487,-1.22137406108,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.913791571956,-1.23510458707,0),Quaternion(0,0,0,0)),],]
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


    env.combined_move_base_and_arm("youbot_0", arm_drop_joint, rest_base_pose)
    loaded = False
    loaded_obj_name = ''
    for i, pick_index_list in enumerate(pick_rounds_index_list):

        for j, index in enumerate(pick_index_list):
            print("pick " + str(index))
            obj_name = "obj_" + str(index)
            env.send_grasp_action(env.planning_scene_msg, obj_name, env.reserved_planning_scene_msg.scene_object_list[env.obj_name_to_index_dict[obj_name]].object_pose, " ", "cube", pick_rounds_pose_list[i][j], True)
            target_base_pose = env.grasp_plan_result.final_base_pose
            pick_joint_value = [env.grasp_plan_result.q1, env.grasp_plan_result.q2, env.grasp_plan_result.q3, env.grasp_plan_result.q4, env.grasp_plan_result.q5]
            pre_pick_joint_value = [env.grasp_plan_result.q1_pre, env.grasp_plan_result.q2_pre, env.grasp_plan_result.q3_pre, env.grasp_plan_result.q4_pre, env.grasp_plan_result.q5_pre]    
            
            if loaded:
                env.combined_move_base_and_arm_drop_pick("youbot_0", loaded_obj_name, arm_drop_joint, pre_pick_joint_value, target_base_pose)
                loaded = False
            else:
                env.combined_move_base_and_arm_pick("youbot_0", pre_pick_joint_value, target_base_pose)
                loaded = True
                loaded_obj_name = obj_name

            env.pick_object_from_prev("youbot_0", pick_joint_value, pre_pick_joint_value)
            env.update_env(obj_name)

            if j == len(pick_index_list) - 1:
                env.combined_move_base_and_arm_drop("youbot_0", obj_name, arm_drop_joint, rest_base_pose)
                loaded = False