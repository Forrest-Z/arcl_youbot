import rospy
import arcl_youbot_application.application_util as app_util
import arcl_youbot_planner.base_planner.base_util as base_util
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_msgs.msg import UInt8
import os.path
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_about_axis, quaternion_from_matrix
import math
import copy

offset = 0.43
offset_last_step = 0.3
first_height = 0.0
second_height = 0.05
load_height = 0.05
unload_height = 0.02


UNLOAD_HEIGHT = [0.0, 0.085, 0.17]
LOAD_HEIGHT = [0.03, 0.115, 0.2]

def load(env, obj_name, height):
    # object
    obj_data = rospy.wait_for_message('/vrpn_client_node/' + obj_name + '/pose', PoseStamped)
    target_base_pose = obj_data.pose
    q = (obj_data.pose.orientation.x, obj_data.pose.orientation.y, obj_data.pose.orientation.z, obj_data.pose.orientation.w)
    obj_roll, obj_pitch, obj_yaw = euler_from_quaternion(q) 

    # base
    base_data = rospy.wait_for_message('/vrpn_client_node/' + "youbot_2" + '/pose', PoseStamped)
    current_base_pose = base_data.pose
    q = (base_data.pose.orientation.x, base_data.pose.orientation.y, base_data.pose.orientation.z, base_data.pose.orientation.w)
    base_roll, base_pitch, base_yaw = euler_from_quaternion(q) 

    # adjust pose
    diff_yaw = base_yaw - obj_yaw
    if abs(diff_yaw) > math.pi:
        if obj_yaw > 0:
            obj_yaw -= math.pi
        else:
            obj_yaw += math.pi
    target_base_pose.position.x -= offset * math.cos(obj_yaw)
    target_base_pose.position.y -= offset * math.sin(obj_yaw)
    q = quaternion_from_euler(obj_roll, obj_pitch, obj_yaw)
    target_base_pose.orientation.x = q[0]
    target_base_pose.orientation.y = q[1]
    target_base_pose.orientation.z = q[2]
    target_base_pose.orientation.w = q[3]

    last_step = copy.deepcopy(target_base_pose)
    last_step.position.x -= offset_last_step * math.cos(obj_yaw)
    last_step.position.y -= offset_last_step * math.sin(obj_yaw)

    env.move_to_target("youbot_2", last_step)
    if height == 0.0:
        env.update_env_del(obj_name)
        # TODO: else, we need to add a new object
    env.set_forklift_position('youbot_2', height)
    env.move_to_target("youbot_2", target_base_pose)
    env.set_forklift_position('youbot_2', height + load_height)

    return target_base_pose


def unload_on_object(env, obj_name, height):
    # object
    obj_data = rospy.wait_for_message('/vrpn_client_node/' + obj_name + '/pose', PoseStamped)
    target_base_pose = obj_data.pose
    q = (obj_data.pose.orientation.x, obj_data.pose.orientation.y, obj_data.pose.orientation.z, obj_data.pose.orientation.w)
    obj_roll, obj_pitch, obj_yaw = euler_from_quaternion(q) 

    # base
    base_data = rospy.wait_for_message('/vrpn_client_node/' + "youbot_2" + '/pose', PoseStamped)
    current_base_pose = base_data.pose
    q = (base_data.pose.orientation.x, base_data.pose.orientation.y, base_data.pose.orientation.z, base_data.pose.orientation.w)
    base_roll, base_pitch, base_yaw = euler_from_quaternion(q) 

    # adjust pose
    diff_yaw = base_yaw - obj_yaw
    if abs(diff_yaw) > math.pi:
        if obj_yaw > 0:
            obj_yaw -= math.pi
        else:
            obj_yaw += math.pi
    target_base_pose.position.x -= offset * math.cos(obj_yaw)
    target_base_pose.position.y -= offset * math.sin(obj_yaw)
    q = quaternion_from_euler(obj_roll, obj_pitch, obj_yaw)
    target_base_pose.orientation.x = q[0]
    target_base_pose.orientation.y = q[1]
    target_base_pose.orientation.z = q[2]
    target_base_pose.orientation.w = q[3]

    last_step = copy.deepcopy(target_base_pose)
    last_step.position.x -= offset_last_step * math.cos(obj_yaw)
    last_step.position.y -= offset_last_step * math.sin(obj_yaw)

    env.move_to_target("youbot_2", last_step)
    if height == 0.0:
        env.update_env_del(obj_name)
        # TODO: else, we need to add a new object
    env.set_forklift_position('youbot_2', height + unload_height)
    env.move_to_target("youbot_2", target_base_pose)
    env.set_forklift_position('youbot_2', height - unload_height)
    env.move_to_target("youbot_2", last_step)

def unload_on_pose(env, end_pose):
    last_step = copy.deepcopy(end_pose)
    q = (last_step.orientation.x, last_step.orientation.y, last_step.orientation.z, last_step.orientation.w)
    _, _, base_yaw = euler_from_quaternion(q) 
    last_step.position.x -= offset_last_step * math.cos(base_yaw)
    last_step.position.y -= offset_last_step * math.sin(base_yaw)

    env.move_to_target("youbot_2", end_pose)
    env.set_forklift_position('youbot_2', first_height)
    env.move_to_target("youbot_2", last_step)

if __name__ == "__main__":
    rospy.init_node("forklift_demo")
    env = app_util.YoubotEnvironment(-1.5, 1.5, -3.0, 1.0)    
    env.mode = 1
    # env.import_obj_from_optitrack_forklift(['obj_50', 'obj_51'])

    rest_base_pose = Pose()
    rest_base_pose.position.x = 0
    rest_base_pose.position.y = 0
    rest_base_pose.position.z = 0.1
    rest_base_pose.orientation.x = 0
    rest_base_pose.orientation.y = 0
    rest_base_pose.orientation.z = -0.7071068
    rest_base_pose.orientation.w = 0.7071068

    target_base_pose = Pose()
    target_base_pose.position.x = 0
    target_base_pose.position.y = -0.5
    target_base_pose.position.z = 0.1
    target_base_pose.orientation.x = 0
    target_base_pose.orientation.y = 0
    target_base_pose.orientation.z = -0.7071068
    target_base_pose.orientation.w = 0.7071068

    # TASK: lift target 1 (two levels), then, put target 1 on the top of target 2 (one level)

    # load target 1
    # end_pose = load(env, 'obj_51', first_height)
    # # unload target 1 on the top of target 2
    # unload_on_object(env, 'obj_50', second_height)

    # # load all of them back to target_base_pose
    # env.update_env_add('obj_51')
    # load(env, 'obj_51', first_height)
    # unload_on_pose(env, end_pose)

    # env.move_to_target("youbot_2", rest_base_pose)
    env.set_forklift_position('youbot_2', 0.0)
