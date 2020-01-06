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
    env = app_util.YoubotEnvironment(-1.5, 1.5, -3.0, 1.0)    
    # env = app_util.YoubotEnvironment(-2.5, 2.5, 0.0, 5.0)
    env.mode = 1
    #import object list from file
    my_path = os.path.abspath(os.path.dirname(__file__))
    

    # reserved_object_list = []
    # for test_obj in env.object_list:
    #     reserved_object_list.append(test_obj)
    arm_up_joint = [202/180.0*math.pi, 65/180.0*math.pi, -146 / 180.0 * math.pi, 102.5 / 180.0 * math.pi, 172 / 180.0 * math.pi]
    arm_up_joint_1 = [155/180.0*math.pi, 85/180.0*math.pi, -146 / 180.0 * math.pi, 102.5 / 180.0 * math.pi, 172 / 180.0 * math.pi]
    arm_up_joint_2 = [202/180.0*math.pi, 25/180.0*math.pi, -86 / 180.0 * math.pi, 102.5 / 180.0 * math.pi, 172 / 180.0 * math.pi]
    arm_up_joint_3 = [202/180.0*math.pi, 65/180.0*math.pi, -146 / 180.0 * math.pi, 76.5 / 180.0 * math.pi, 172 / 180.0 * math.pi]
    arm_up_joint_4 = [202/180.0*math.pi, 65/180.0*math.pi, -146 / 180.0 * math.pi, 102.5 / 180.0 * math.pi, 172 / 180.0 * math.pi]
    arm_up_joint_5 = [202/180.0*math.pi, 120/180.0*math.pi, -80 / 180.0 * math.pi, 102.5 / 180.0 * math.pi, 121 / 180.0 * math.pi]
    arm_up_joint_6 = [202/180.0*math.pi, 65/180.0*math.pi, -146 / 180.0 * math.pi, 102.5 / 180.0 * math.pi, 172 / 180.0 * math.pi]

    # env.move_arm_to_joint_pose("youbot_0", arm_up_joint)
    # sample_num = 1000
    #while True:
        # jnt_pos = []
        # for jnt in range(5):
        #     jnt_pos.append( np.random.uniform(arm_util.MIN_JOINT_POS[jnt] - arm_util.JOINT_OFFSET[jnt] ,arm_util.MAX_JOINT_POS[jnt] - arm_util.JOINT_OFFSET[jnt]))
        # jnt_pos = [5.282255649566651, 1.2318736314788064, -2.0795466899871577, 1.6788692474365234, 1.9565308094024658]
    jnt_pos = [3.0895373821258536, 1.1700667142879224, -1.026131987569805, 2.8564229011535645, 1.8807144165039062]
    
    # arm_util.set_gripper_width("youbot_0", 0.06, 1)
    
    
    env.move_arm_to_joint_pose_old("youbot_0", arm_up_joint)
    env.move_arm_to_joint_pose_old("youbot_0", arm_up_joint_1)
    env.move_arm_to_joint_pose_old("youbot_0", arm_up_joint_2)
    env.move_arm_to_joint_pose_old("youbot_0", arm_up_joint_3)
    env.move_arm_to_joint_pose_old("youbot_0", arm_up_joint_4)
    env.move_arm_to_joint_pose_old("youbot_0", arm_up_joint_5)
    env.move_arm_to_joint_pose_old("youbot_0", arm_up_joint_6)


    rospy.spin()
   