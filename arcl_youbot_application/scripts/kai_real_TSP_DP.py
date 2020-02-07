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

base_pose = Pose()

def position_callback(self, data):
    global base_pose
    base_pose.position.x = data.point.x
    base_pose.position.y = data.point.y

if __name__ == "__main__":
    rospy.init_node("single_youbot_pick_demo")
    rospy.Subscriber('/youbot_0/robot/pose', PointStamped, position_callback)

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


    # Calculation
    env_obj_list = copy.deepcopy(env.object_list)
    DP = TSP_DP.DP_solver(env_obj_list)
    DP.experiment()
    
    # # Generate DP Lists
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
    # # Finish Generating DP Lists

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

    # DP
    # pick_rounds_index_list = [[4], [2, 10], [11, 5], [9, 3], [7, 8], [0], [1]]
    # pick_rounds_pose_list = [[Pose(Point(-1.05625121057,-0.616028618248,0),Quaternion(0,0,0,0)),],[Pose(Point(0.213718271621,-1.03639375863,0),Quaternion(0,0,0,0)),Pose(Point(0.360496555365,-1.43625118697,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.976072958552,-0.948464292816,0),Quaternion(0,0,0,0)),Pose(Point(-1.05257721248,-1.02280458927,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.0196509190422,-0.444175127916,0),Quaternion(0,0,0,0)),Pose(Point(-0.18983864183,-0.836230665043,0),Quaternion(0,0,0,0)),],[Pose(Point(0.617243972906,-0.534148463288,0),Quaternion(0,0,0,0)),Pose(Point(0.836986233474,-0.900456330995,0),Quaternion(0,0,0,0)),],[Pose(Point(0.193936922847,-0.270637124346,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.407467226301,-1.46466500069,0),Quaternion(0,0,0,0)),],]
    # Greedy
    # pick_rounds_index_list = [[0, 9], [3, 12], [4, 10], [7, 8], [2, 1], [6, 11], [5]]
    # pick_rounds_pose_list = [[Pose(Point(0.194936922847,-0.269637124346,0),Quaternion(0,0,0,0)),Pose(Point(0.418833850295,-0.658252911265,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.331794029076,-0.400564339607,0),Quaternion(0,0,0,0)),Pose(Point(-0.639952311994,-0.727235047379,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.550830873639,-0.108999222646,0),Quaternion(0,0,0,0)),Pose(Point(-0.41901285757,-1.12270663266,0),Quaternion(0,0,0,0)),],[Pose(Point(0.618243972906,-0.533148463288,0),Quaternion(0,0,0,0)),Pose(Point(0.450263633077,-0.742495714026,0),Quaternion(0,0,0,0)),],[Pose(Point(0.214718271621,-1.03539375863,0),Quaternion(0,0,0,0)),Pose(Point(-1.20857274301,-1.24257189093,0),Quaternion(0,0,0,0)),],[Pose(Point(-0.489278263723,-1.06928027696,0),Quaternion(0,0,0,0)),Pose(Point(-0.886305903588,-1.31967441356,0),Quaternion(0,0,0,0)),],[Pose(Point(-1.05157721248,-1.02180458927,0),Quaternion(0,0,0,0)),],]

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