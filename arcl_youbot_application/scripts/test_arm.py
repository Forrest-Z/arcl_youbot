import rospy
import arcl_youbot_application.application_util as app_util
import arcl_youbot_planner.arm_planner.arm_util as arm_util
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from std_msgs.msg import UInt8
import os.path
import math
import numpy as np

if __name__ == "__main__":
    rospy.init_node("reachibility")
    #env = app_util.YoubotEnvironment(-1.5, 1.5, -3.0, 1.0)    
    env = app_util.YoubotEnvironment(-2.5, 2.5, 0.0, 5.0)
    env.mode = 0
    #import object list from file
    my_path = os.path.abspath(os.path.dirname(__file__))
    # env.import_obj_from_file(os.path.join(my_path, "scatter/new_1.txt"))
    # #env.create_scene(20, 5)
    # #env.export_obj_to_file(os.path.join(my_path, "scatter/new.txt"))
    # # pick_index_list = [8, 9, 4, 0, 7, 5, 6, 2, 1, 3]
    # # pick_index_list = [0, 6, 4, 5, 2, 1, 3]
    # # pick_index_list = [5, 3, 4, 1, 0, 2]
    # # pick_index_list = [3, 4, 1, 0, 2]
    # # pick_index_list = [3, 1, 0, 2]
    # # pick_index_list = [0, 1]
    # pick_index_list = [0]


    # #spawn the objects in gazebo, and generate the planningscene msg 
    # env.generate_obj_in_gazebo()

    # rest_base_pose = Pose()
    # rest_base_pose.position.x = 0
    # rest_base_pose.position.y = -0.5
    # rest_base_pose.position.z = 0.1
    # rest_base_pose.orientation.x = 0
    # rest_base_pose.orientation.y = 0
    # rest_base_pose.orientation.z = 0.7071068
    # rest_base_pose.orientation.w = 0.7071068   

    # target_base_pose = Pose()
    # target_base_pose.position.x = 0.2
    # target_base_pose.position.y = 1
    # target_base_pose.position.z = 0.1      
    # target_base_pose.orientation.x = 0
    # target_base_pose.orientation.y = 0
    # target_base_pose.orientation.z = 0
    # target_base_pose.orientation.w = 1

    # reserved_object_list = []
    # for test_obj in env.object_list:
    #     reserved_object_list.append(test_obj)
    arm_up_joint = [202/180.0*math.pi, 65/180.0*math.pi, -146 / 180.0 * math.pi, 102.5 / 180.0 * math.pi, 172 / 180.0 * math.pi]
    # env.move_arm_to_joint("youbot_0", arm_up_joint)
    # sample_num = 1000
    #while True:
        # jnt_pos = []
        # for jnt in range(5):
        #     jnt_pos.append( np.random.uniform(arm_util.MIN_JOINT_POS[jnt] - arm_util.JOINT_OFFSET[jnt] ,arm_util.MAX_JOINT_POS[jnt] - arm_util.JOINT_OFFSET[jnt]))
        # jnt_pos = [5.282255649566651, 1.2318736314788064, -2.0795466899871577, 1.6788692474365234, 1.9565308094024658]
    jnt_pos = [3.0895373821258536, 1.6700667142879224, -1.026131987569805, 2.8564229011535645, 1.8807144165039062]
    env.move_arm_to_joint_pose("youbot_0", jnt_pos)
    rospy.spin()
    # env.move_arm_to_joint_pose("youbot_0", arm_up_joint)
    # for index in range(sample_num):

        # pick_joint_value = [env.grasp_plan_result.q1, env.grasp_plan_result.q2, env.grasp_plan_result.q3, env.grasp_plan_result.q4, env.grasp_plan_result.q5]
        # pre_pick_joint_value = [env.grasp_plan_result.q1_pre, env.grasp_plan_result.q2_pre, env.grasp_plan_result.q3_pre, env.grasp_plan_result.q4_pre, env.grasp_plan_result.q5_pre]    
        # env.pick_object("youbot_0", pick_joint_value, pre_pick_joint_value)
        # env.update_env(reserved_object_list[index])


        # env.move_to_target("youbot_0", rest_base_pose)
        # env.drop_object('obj_' + str(index))
        # env.move_arm_to_joint("youbot_0", arm_up_joint)

        #(5.282255649566651, 1.2318736314788064, -2.0795466899871577, 1.6788692474365234, 1.9565308094024658)
        #(3.0895373821258536, 1.6700667142879224, -1.026131987569805, 2.8564229011535645, 1.8807144165039062)