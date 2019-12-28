import rospy
import arcl_youbot_application.application_util as app_util
import arcl_youbot_planner.arm_planner.arm_util as arm_util
from arcl_youbot_msgs.msg import PolygonArray
from geometry_msgs.msg import Pose, Polygon, Point32
from std_msgs.msg import String
from std_msgs.msg import UInt8
import os.path
import threading
import time
import signal, sys

def ctrl_c_handler(signal, frame):
    """Gracefully quit the infrared_pub node"""
    print("\nCaught ctrl-c! Stopping node.")
    sys.exit()

def thread_test(env, name, target, rest):
    env.thread_move_to_target(name, target)
    time.sleep(1)
    env.thread_move_to_target(name, rest)


if __name__ == "__main__":
    signal.signal(signal.SIGINT, ctrl_c_handler)
    rospy.init_node("single_youbot_pick_demo")
    env = app_util.YoubotEnvironment(-1.5, 1.5, -3.0, 1.0)    
    # env = app_util.YoubotEnvironment(-2.5, 2.5, 0.0, 5.0)
    env.mode = 0
    #import object list from file
    my_path = os.path.abspath(os.path.dirname(__file__))
    env.import_obj_from_file(os.path.join(my_path, "scatter/new_11.txt"))

    env.generate_obj_in_gazebo()

    rest_base_pose1 = Pose()
    rest_base_pose1.position.x = -1.0
    rest_base_pose1.position.y = -2.0
    rest_base_pose1.position.z = 0.1
    rest_base_pose1.orientation.x = 0
    rest_base_pose1.orientation.y = 0
    rest_base_pose1.orientation.z = 0
    rest_base_pose1.orientation.w = 1

    rest_base_pose2 = Pose()
    rest_base_pose2.position.x = -1.0
    rest_base_pose2.position.y = -1.0
    rest_base_pose2.position.z = 0.1
    rest_base_pose2.orientation.x = 0
    rest_base_pose2.orientation.y = 0
    rest_base_pose2.orientation.z = 0
    rest_base_pose2.orientation.w = 1

    rest_base_pose3 = Pose()
    rest_base_pose3.position.x = -1.0
    rest_base_pose3.position.y = -3.0
    rest_base_pose3.position.z = 0.1
    rest_base_pose3.orientation.x = 0
    rest_base_pose3.orientation.y = 0
    rest_base_pose3.orientation.z = 0
    rest_base_pose3.orientation.w = 1

    pick_obj_seq1 = ['obj_5']
    pick_obj_seq2 = ['obj_2']
    pick_obj_seq3 = ['obj_0']
    youbot_names = ['youbot_0', 'youbot_1', 'youbot_2']
    rest_base_poses = [rest_base_pose1, rest_base_pose2, rest_base_pose3]

    obs_pub = rospy.Publisher('/rvo2/obstacles', PolygonArray, queue_size=1)
    time.sleep(2)
    obstacles = env.object_list.values()
    msg = PolygonArray()
    msg.polygons = []
    for ob in obstacles:
        polygon = Polygon()
        polygon.points = []
        for p in ob:
            polygon.points.append(Point32(x=p[0], y=p[1], z=0))
        msg.polygons.append(polygon)
    obs_pub.publish(msg)

    for index in range(len(pick_obj_seq1)):
        env.send_grasp_action(env.planning_scene_msg, pick_obj_seq1[index], env.planning_scene_msg.scene_object_list[env.obj_name_to_index_dict[pick_obj_seq1[index]]].object_pose, " ", "cube", rest_base_pose1, True)
        target_base_pose1 = env.grasp_plan_result.final_base_pose   
        pick_joint_value1 = [env.grasp_plan_result.q1, env.grasp_plan_result.q2, env.grasp_plan_result.q3, env.grasp_plan_result.q4, env.grasp_plan_result.q5]
        pre_pick_joint_value1 = [env.grasp_plan_result.q1_pre, env.grasp_plan_result.q2_pre, env.grasp_plan_result.q3_pre, env.grasp_plan_result.q4_pre, env.grasp_plan_result.q5_pre]   
        
        env.send_grasp_action(env.planning_scene_msg, pick_obj_seq2[index], env.planning_scene_msg.scene_object_list[env.obj_name_to_index_dict[pick_obj_seq2[index]]].object_pose, " ", "cube", rest_base_pose1, True)
        target_base_pose2 = env.grasp_plan_result.final_base_pose 
        pick_joint_value2 = [env.grasp_plan_result.q1, env.grasp_plan_result.q2, env.grasp_plan_result.q3, env.grasp_plan_result.q4, env.grasp_plan_result.q5]
        pre_pick_joint_value2 = [env.grasp_plan_result.q1_pre, env.grasp_plan_result.q2_pre, env.grasp_plan_result.q3_pre, env.grasp_plan_result.q4_pre, env.grasp_plan_result.q5_pre]  
        
        env.send_grasp_action(env.planning_scene_msg, pick_obj_seq3[index], env.planning_scene_msg.scene_object_list[env.obj_name_to_index_dict[pick_obj_seq3[index]]].object_pose, " ", "cube", rest_base_pose1, True)
        target_base_pose3 = env.grasp_plan_result.final_base_pose 
        pick_joint_value3 = [env.grasp_plan_result.q1, env.grasp_plan_result.q2, env.grasp_plan_result.q3, env.grasp_plan_result.q4, env.grasp_plan_result.q5]
        pre_pick_joint_value3 = [env.grasp_plan_result.q1_pre, env.grasp_plan_result.q2_pre, env.grasp_plan_result.q3_pre, env.grasp_plan_result.q4_pre, env.grasp_plan_result.q5_pre]  


        thr1 = threading.Thread(target=env.thread_move_to_target, args=("youbot_0", target_base_pose1)) 
        thr2 = threading.Thread(target=env.thread_move_to_target, args=("youbot_1", target_base_pose2)) 
        thr3 = threading.Thread(target=env.thread_move_to_target, args=("youbot_2", target_base_pose3)) 
        # thr3 = threading.Thread(target=env.thread_move_to_target, args=("youbot_2", rest_base_pose3)) 

        # env.multi_move_to_target(youbot_names, {youbot_names[0]: target_base_pose1, youbot_names[1]: target_base_pose2, youbot_names[2]: target_base_pose3})
        # env.multi_move_to_target(youbot_names, {youbot_names[0]: target_base_pose1, youbot_names[1]: rest_base_pose2, youbot_names[2]: rest_base_pose3})

        # thr1 = threading.Thread(target=env.pick_object, args=("youbot_0", pick_joint_value1, pre_pick_joint_value1)) 
        # thr2 = threading.Thread(target=env.pick_object, args=("youbot_1", pick_joint_value2, pre_pick_joint_value2)) 
        # thr3 = threading.Thread(target=env.pick_object, args=("youbot_2", pick_joint_value3, pre_pick_joint_value3)) 
        thr1.start()
        thr2.start()
        thr3.start()
        thr1.join()
        thr2.join()
        thr3.join()
        # env.pick_object("youbot_0", pick_joint_value1, pre_pick_joint_value1)
        # env.pick_object("youbot_1", pick_joint_value2, pre_pick_joint_value2)

        # env.update_env('obj_2')
        # env.update_env('obj_1')
        # env.update_env('obj_5')
        print("back!!!!!!!!!!!!!!!!!!")
        # env.multi_move_to_target(youbot_names, {youbot_names[0]: rest_base_pose1, youbot_names[1]: rest_base_pose2, youbot_names[2]: rest_base_pose3})
        thr1 = threading.Thread(target=env.thread_move_to_target, args=("youbot_0", rest_base_pose1)) 
        thr2 = threading.Thread(target=env.thread_move_to_target, args=("youbot_1", rest_base_pose2)) 
        thr3 = threading.Thread(target=env.thread_move_to_target, args=("youbot_2", rest_base_pose3)) 
        thr1.start()
        thr2.start()
        thr3.start()
        thr1.join()
        thr2.join()
        thr3.join()


    print("Done!") 