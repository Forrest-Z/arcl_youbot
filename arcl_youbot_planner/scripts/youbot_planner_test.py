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


if __name__ == "__main__":
    rospy.init_node('arcl_youbot_planner_test')
    physicsClient = p.connect(p.DIRECT)#or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
    p.setGravity(0,0,-10)
    prmstar = prmstar.PRMStarPlanner(p, "/home/wei/catkin_youbot_ws/src/luh_youbot_description/robots/youbot_0.urdf")
    #prmstar.build_roadmap()
    start = np.zeros(arm_util.ARM_JOINT_NUM)
    #for jnt in range(ARM_JOINT_NUM):
    # arm_util.set_arm_joint_pos(joint_mat[:, sample_index], p, self.robot_id)
     #   start[jnt] = np.random.uniform(MIN_JOINT_POS[jnt] - JOINT_OFFSET[jnt] ,MAX_JOINT_POS[jnt] - JOINT_OFFSET[jnt])
    goal = np.zeros(arm_util.ARM_JOINT_NUM)
    # for jnt in range(ARM_JOINT_NUM):
    #     goal[jnt] = np.random.uniform(MIN_JOINT_POS[jnt] - JOINT_OFFSET[jnt] ,MAX_JOINT_POS[jnt] - JOINT_OFFSET[jnt])  
    goal[1] += 0.5 
    # goal[0] += 0.2
    goal[2] -= 0.3

    [final_path, final_cost] = prmstar.path_plan(tuple(start), tuple(goal))
    arm_util.execute_path(final_path, '/arm_1/follow_joint_trajectory')

    
    rospy.spin()


