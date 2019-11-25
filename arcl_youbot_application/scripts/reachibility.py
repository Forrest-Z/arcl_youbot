import rospy
import arcl_youbot_application.application_util as app_util
import arcl_youbot_planner.arm_planner.arm_util as arm_util
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from std_msgs.msg import UInt8
import os.path
import math

if __name__ == "__main__":
    rospy.init_node("reachibility")
    #env = app_util.YoubotEnvironment(-1.5, 1.5, -3.0, 1.0)    
    env = app_util.YoubotEnvironment(-2.5, 2.5, 0.0, 5.0)
    env.mode = 0
    #import object list from file
    my_path = os.path.abspath(os.path.dirname(__file__))
    
    env.plot_reachibility()
    