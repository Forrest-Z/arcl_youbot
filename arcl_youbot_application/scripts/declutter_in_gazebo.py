import rospy
import arcl_youbot_application.application_util as app_util
import arcl_youbot_planner.arm_planner.arm_util as arm_util
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from std_msgs.msg import UInt8
import os.path
import math

if __name__ == "__main__":
    rospy.init_node("single_youbot_pick_demo")
    #env = app_util.YoubotEnvironment(-1.5, 1.5, -3.0, 1.0)    
    env = app_util.YoubotEnvironment(-2.5, 2.5, 0.0, 5.0)
    env.mode = 0
    #import object list from file
    my_path = os.path.abspath(os.path.dirname(__file__))
    env.import_obj_from_file(os.path.join(my_path, "scatter/new_10.txt"))
    #env.create_scene(20, 5)
    #env.export_obj_to_file(os.path.join(my_path, "scatter/new.txt"))
    pick_index_list = [8, 9, 4, 0, 7, 5, 6, 2, 1, 3]
    # pick_index_list = [0, 6, 4, 5, 2, 1, 3]
    # pick_index_list = [5, 3, 4, 1, 0, 2]
    # pick_index_list = [3, 4, 1, 0, 2]
    # pick_index_list = [3, 1, 0, 2]
    # pick_index_list = [0, 1]
    # pick_index_list = [0]


    #spawn the objects in gazebo, and generate the planningscene msg 
    env.generate_obj_in_gazebo()

    rest_base_pose = Pose()
    rest_base_pose.position.x = 0
    rest_base_pose.position.y = -0.5
    rest_base_pose.position.z = 0.1
    rest_base_pose.orientation.x = 0
    rest_base_pose.orientation.y = 0
    rest_base_pose.orientation.z = 0.7071068
    rest_base_pose.orientation.w = 0.7071068   

    target_base_pose = Pose()
    target_base_pose.position.x = 0.2
    target_base_pose.position.y = 1
    target_base_pose.position.z = 0.1      
    target_base_pose.orientation.x = 0
    target_base_pose.orientation.y = 0
    target_base_pose.orientation.z = 0
    target_base_pose.orientation.w = 1

    reserved_object_list = {}
    for obj_name, test_obj  in env.object_list.iteritems():

        reserved_object_list[obj_name] = test_obj
    arm_up_joint = [202/180.0*math.pi, 65/180.0*math.pi, -146 / 180.0 * math.pi, 102.5 / 180.0 * math.pi, 172 / 180.0 * math.pi]
    env.move_arm_to_joint_pose("youbot_0", arm_up_joint)
    for index in pick_index_list:
        print("pick " + str(index))
        obj_name = "obj_" + str(index)
        env.send_grasp_action(env.planning_scene_msg, "obj_" + str(index), env.reserved_planning_scene_msg.scene_object_list[env.obj_name_to_index_dict[obj_name]].object_pose, " ", "cube", rest_base_pose, True)
        target_base_pose = env.grasp_plan_result.final_base_pose
        env.move_to_target("youbot_0", target_base_pose)
        
        pick_joint_value = [env.grasp_plan_result.q1, env.grasp_plan_result.q2, env.grasp_plan_result.q3, env.grasp_plan_result.q4, env.grasp_plan_result.q5]
        pre_pick_joint_value = [env.grasp_plan_result.q1_pre, env.grasp_plan_result.q2_pre, env.grasp_plan_result.q3_pre, env.grasp_plan_result.q4_pre, env.grasp_plan_result.q5_pre]    
        env.pick_object("youbot_0", pick_joint_value, pre_pick_joint_value)
        env.update_env(reserved_object_list[obj_name])

        env.move_to_target("youbot_0", rest_base_pose)
        env.drop_object('obj_' + str(index))
        env.move_arm_to_joint_pose("youbot_0", arm_up_joint)
        # raw_input("wait")