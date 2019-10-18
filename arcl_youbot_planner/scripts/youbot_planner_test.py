import rospy
import math
import pybullet as p
import time
import numpy as np
import pybullet_data
import arcl_youbot_planner.arm_planner.prmstar as prmstar
import arcl_youbot_planner.arm_planner.arm_util as arm_util
import control_msgs
import rospy
import actionlib
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
SAMPLE_NUM = 500
PATH_INTERPOLATE_NUM=10
MIN_JOINT_POS = [-2.9395372288,
    -1.124398163,
    -2.4783710285,
    -1.7668361332,
    -2.8913271881]
MAX_JOINT_POS = [2.8905337534,
    1.4835265078,
    2.532469254,
    1.6402432236,
    2.6396457936]
JOINT_OFFSET = [-2.9395372288,
    -1.124398163,
    2.532469254,
    -1.7668361332,
    -2.8913271881]
# JOINT_OFFSET = [ -2.9395372288,
#     -1.1244673701987218,
#      2.5281772172532176,
#     -1.7689600250839740,
#     -2.8919466477485340]
ARM_JOINT_NUM = 5

if __name__ == "__main__":
    rospy.init_node('arcl_youbot_planner_test')
    physicsClient = p.connect(p.DIRECT)#or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
    p.setGravity(0,0,-10)
    prmstar = prmstar.PRMStarPlanner(p, "/home/wei/catkin_youbot_ws/src/luh_youbot_description/robots/youbot_0.urdf")
    #prmstar.build_roadmap()
    start = np.zeros(ARM_JOINT_NUM)
    #for jnt in range(ARM_JOINT_NUM):
    # arm_util.set_arm_joint_pos(joint_mat[:, sample_index], p, self.robot_id)
     #   start[jnt] = np.random.uniform(MIN_JOINT_POS[jnt] - JOINT_OFFSET[jnt] ,MAX_JOINT_POS[jnt] - JOINT_OFFSET[jnt])
    goal = np.zeros(ARM_JOINT_NUM)
    # for jnt in range(ARM_JOINT_NUM):
    #     goal[jnt] = np.random.uniform(MIN_JOINT_POS[jnt] - JOINT_OFFSET[jnt] ,MAX_JOINT_POS[jnt] - JOINT_OFFSET[jnt])  
    goal[1] += 0.5 
    # goal[0] += 0.2
    goal[2] -= 0.3

    [final_path, final_cost] = prmstar.path_plan(tuple(start), tuple(goal))
    client = actionlib.SimpleActionClient('/arm_1/follow_joint_trajectory', FollowJointTrajectoryAction)
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
    print("detect the server")
    print(goal)
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(10.0))
    rospy.spin()


