import rospy
import arcl_youbot_application.application_util as app_util
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from std_msgs.msg import UInt8

if __name__ == "__main__":
    rospy.init_node("single_youbot_pick_demo")
    env = app_util.YoubotEnvironment(-20, 20, -20, 20)

    #import object list from file
    env.import_obj_from_file("/home/wei/Desktop/scatter/10_0.txt")

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


    env.send_grasp_action(env.planning_scene_msg, "obj_7", env.planning_scene_msg.scene_object_list[7].object_pose, " ", "cube", rest_base_pose, True)

    target_base_pose = env.grasp_plan_result.final_base_pose

    env.move_to_target("youbot", target_base_pose)
    
    pick_joint_value = [env.grasp_plan_result.q1, env.grasp_plan_result.q2, env.grasp_plan_result.q3, env.grasp_plan_result.q4, env.grasp_plan_result.q5]
    pre_pick_joint_value = [env.grasp_plan_result.pre_q1, env.grasp_plan_result.pre_q2, env.grasp_plan_result.pre_q3, env.grasp_plan_result.pre_q4, env.grasp_plan_result.pre_q5]    
    env.pick_object("youbot", pick_joint_value, pre_pick_joint_value)
