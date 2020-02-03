import rospy
import math
import pybullet as p
import time
import pybullet_data
import numpy as np
import actionlib
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from sensor_msgs.msg import JointState
from arcl_youbot_msgs.msg import SetGripperAction, SetGripperGoal, MoveToJointPoseGoal

JOINT_1_INDEX = 8
JOINT_2_INDEX = 9
JOINT_3_INDEX = 10
JOINT_4_INDEX = 11
JOINT_5_INDEX = 12

SAMPLE_NUM = 500
PATH_INTERPOLATE_NUM=10
MIN_JOINT_POS = [
    -2.9395372288,
    -1.124398163,
    -2.4783710285,
    -1.7668361332,
    -2.8913271881]
MAX_JOINT_POS = [
    2.8905337534,
    1.4835265078,
    2.532469254,
    1.6402432236,
    2.6396457936]
JOINT_OFFSET = [
    -2.949606436,
    -1.1344673701987218,
     2.5481772172532176,
    -1.7889600250839740,
    -3.0019466477485340
    ]

ARM_JOINT_NUM = 5

current_path_finished = False


def execute_path_callback(goal_state, result):
    current_path_finished = True

#change youbot arm joint value from MIN-MAX range to actual joint limit range
def subtract_offset(joint_value):
    if len(joint_value) != ARM_JOINT_NUM:
        raise Exception('arm_util.subtract_offset: input joint_value length not equal to ARM_JOINT_NUM')

    for jnt_index in range(len(joint_value)):
        if joint_value[jnt_index] >= MIN_JOINT_POS[jnt_index] and joint_value[jnt_index] <= MAX_JOINT_POS[jnt_index]:
            joint_value[jnt_index] -= JOINT_OFFSET[jnt_index]
        else:
            raise Exception('arm_util.subtract_offset: input joint_value exceeds limit, input: {}'.format(joint_value))

def set_arm_joint_pos(joint_vector, pybullet_client, youbot_id):
    pybullet_client.resetJointState(youbot_id, JOINT_1_INDEX, joint_vector[0])
    pybullet_client.resetJointState(youbot_id, JOINT_2_INDEX, joint_vector[1])
    pybullet_client.resetJointState(youbot_id, JOINT_3_INDEX, joint_vector[2])
    pybullet_client.resetJointState(youbot_id, JOINT_4_INDEX, joint_vector[3])
    pybullet_client.resetJointState(youbot_id, JOINT_5_INDEX, joint_vector[4])

def find_nearest_neighbor(query, joint_mat, neighbor_num):
    sample_num = joint_mat.shape[1]
    index = 0
    n_index = 0
    dist_arr = np.zeros([sample_num])
    while index < sample_num:
        dist = (joint_mat[:,index] - query)*(joint_mat[:, index] - query)
        dist_arr[index] = dist.sum()
        index += 1
    dist_index = np.argpartition(dist_arr, neighbor_num)
    return dist_index[:neighbor_num]

def set_gripper_width(youbot_name, width, mode=0):
    if mode == 0:
        client = actionlib.SimpleActionClient(youbot_name + "/gazebo/arm_1/set_gripper" , SetGripperAction)
    else:
        client = actionlib.SimpleActionClient(youbot_name + "/arm_1/set_gripper" , SetGripperAction)
    client.wait_for_server()
    goal = SetGripperGoal()
    
    goal.gripper_width = width
    goal.is_relative = False
    client.send_goal_and_wait(goal, rospy.Duration.from_sec(2.0), rospy.Duration.from_sec(2.0))
    # client.wait_for_result(rospy.Duration.from_sec(10.0))

def execute_path(youbot_name, final_path):
    client = actionlib.SimpleActionClient(youbot_name + '/arm_1/follow_joint_trajectory', FollowJointTrajectoryAction)

    print("wait for server")
    print(youbot_name + '/arm_1/follow_joint_trajectory')
    client.wait_for_server()
    goal = FollowJointTrajectoryGoal()
    begin_time = 0
    for pt_index in range(len(final_path)):
        if pt_index < (len(final_path)-1):
            for interpolate_index in range(PATH_INTERPOLATE_NUM):
                traj_pt = JointTrajectoryPoint()
                current_pt = final_path[pt_index]
                next_pt = final_path[pt_index+1]
                for index, j in  zip(range(ARM_JOINT_NUM), current_pt):
                    traj_pt.positions.append(j + JOINT_OFFSET[index] + interpolate_index * (next_pt[index] - current_pt[index]) / PATH_INTERPOLATE_NUM)
                begin_time += 0.2
                traj_pt.time_from_start = rospy.Duration.from_sec(begin_time)
                goal.trajectory.points.append(traj_pt)
        else:
            traj_pt = JointTrajectoryPoint()
            for index, j in  zip(range(ARM_JOINT_NUM),final_path[pt_index]):
                traj_pt.positions.append(j + JOINT_OFFSET[index])
            begin_time += 0.2
            traj_pt.time_from_start = rospy.Duration.from_sec(begin_time)
            goal.trajectory.points.append(traj_pt)
    goal.goal_time_tolerance = rospy.Duration.from_sec(10)
    # print(goal)
    client.send_goal_and_wait(goal)
    print("finished current path")

def execute_path_without_wait(youbot_name, final_path):
    client = actionlib.SimpleActionClient(youbot_name + '/arm_1/follow_joint_trajectory', FollowJointTrajectoryAction)
    current_path_finished = False
    print("wait for server")
    print(youbot_name + '/arm_1/follow_joint_trajectory')
    client.wait_for_server()
    goal = FollowJointTrajectoryGoal()
    begin_time = 0
    for pt_index in range(len(final_path)):
        if pt_index < (len(final_path)-1):
            for interpolate_index in range(PATH_INTERPOLATE_NUM):
                traj_pt = JointTrajectoryPoint()
                current_pt = final_path[pt_index]
                next_pt = final_path[pt_index+1]
                for index, j in  zip(range(ARM_JOINT_NUM), current_pt):
                    traj_pt.positions.append(j + JOINT_OFFSET[index] + interpolate_index * (next_pt[index] - current_pt[index]) / PATH_INTERPOLATE_NUM)
                begin_time += 0.2
                traj_pt.time_from_start = rospy.Duration.from_sec(begin_time)
                goal.trajectory.points.append(traj_pt)
        else:
            traj_pt = JointTrajectoryPoint()
            for index, j in  zip(range(ARM_JOINT_NUM),final_path[pt_index]):
                traj_pt.positions.append(j + JOINT_OFFSET[index])
            begin_time += 0.2
            traj_pt.time_from_start = rospy.Duration.from_sec(begin_time)
            goal.trajectory.points.append(traj_pt)
    goal.goal_time_tolerance = rospy.Duration.from_sec(10)
    # print(goal)
    client.send_goal_and_wait(goal)
    print("finished current path")


class ArmController():
    """ 
    This is used to publish velocity
    """

    def __init__(self, youbot_name, mode):
        self.mode = mode
        self.is_joint_state_received = False
        self.youbot_name = youbot_name
        self.current_joint_state_ = []
        if mode == 0:
            rospy.Subscriber(youbot_name + '/gazebo/joint_states', JointState, self.arm_joint_state_callback, [youbot_name])
        else:
            rospy.Subscriber(youbot_name + '/arm_1/joint_states', JointState, self.arm_joint_state_callback, [youbot_name])

    def arm_joint_state_callback(self, data, args):
        """ Gazebo: callback to receive the current youbot position
        """
        self.current_joint_state_ = []
        self.current_joint_state_.append(data.position[0])
        self.current_joint_state_.append(data.position[1])
        self.current_joint_state_.append(data.position[2])
        self.current_joint_state_.append(data.position[3])
        self.current_joint_state_.append(data.position[4])
        self.is_joint_state_received = True





#return the joint position in the actual range (0,0,-5,0,0) to (5,5,0, 5, 5)
    def get_current_joint_pos(self):
        while self.is_joint_state_received == False:
            pass
        self.is_joint_state_received = False
        return self.current_joint_state_
        
        