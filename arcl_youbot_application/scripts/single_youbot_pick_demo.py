import rospy
import arcl_youbot_application.application_util as app_util
import arcl_youbot_planner.arm_planner.arm_util as arm_util
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from std_msgs.msg import UInt8
import os.path
from arcl_youbot_planner.base_planner.velocity_controller import VelocityController

if __name__ == "__main__":
    rospy.init_node("single_youbot_pick_demo")
    env = app_util.YoubotEnvironment(-20, 20, -20, 20)

    #import object list from file
<<<<<<< HEAD
    env.import_obj_from_file("/home/wei/Desktop/scatter/10_0.txt")
=======
    my_path = os.path.abspath(os.path.dirname(__file__))
    env.import_obj_from_file(os.path.join(my_path, "scatter/10_0.txt"))
>>>>>>> 1c10199c47d327b89b473c1277259f94bd0e604f

    #spawn the objects in gazebo, and generate the planningscene msg 
    env.generate_obj_in_gazebo()

    rest_base_pose = Pose()
    rest_base_pose.position.x = 0
    rest_base_pose.position.y = -0.5
    rest_base_pose.position.z = 0.1
    rest_base_pose.orientation.x = 0
    rest_base_pose.orientation.y = 0
    rest_base_pose.orientation.z = 0
    rest_base_pose.orientation.w = 1

    #arm_util.set_gripper_width("youbot", 0.04)
    # arm_util.set_gripper_width("youbot", 0.0)
    # arm_util.set_gripper_width("youbot", 0.06)
    # arm_util.set_gripper_width("youbot", 0.0)
    env.send_grasp_action(env.planning_scene_msg, "obj_7", env.planning_scene_msg.scene_object_list[7].object_pose, " ", "cube", rest_base_pose, True)

    target_base_pose = env.grasp_plan_result.final_base_pose
    print(target_base_pose)
    env.move_to_target("youbot_0", target_base_pose)
    
    pick_joint_value = [env.grasp_plan_result.q1, env.grasp_plan_result.q2, env.grasp_plan_result.q3, env.grasp_plan_result.q4, env.grasp_plan_result.q5]
    pre_pick_joint_value = [env.grasp_plan_result.q1_pre, env.grasp_plan_result.q2_pre, env.grasp_plan_result.q3_pre, env.grasp_plan_result.q4_pre, env.grasp_plan_result.q5_pre]    
    env.pick_object("youbot_0", pick_joint_value, pre_pick_joint_value)
