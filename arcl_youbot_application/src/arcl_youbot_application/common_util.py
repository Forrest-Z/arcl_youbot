#!/usr/bin/env python
import rospy
import math
import pybullet as p
import time
import numpy as np
import pybullet_data
import arcl_youbot_planner.arm_planner.arm_util as arm_util
import arcl_youbot_planner.base_planner.base_util as base_util
# import control_msgs.msg.FollowJointTrajectoryActionGoal
from geometry_msgs.msg import Twist 
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String
from std_msgs.msg import UInt8
import arcl_youbot_planner.arm_planner.astar 
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_about_axis, quaternion_from_matrix
import scipy.spatial
import string
import actionlib
import StringIO
from arcl_youbot_application.msg import ManipulationAction, ManipulationGoal
from arcl_youbot_application.msg import PlanningSceneMsg
from arcl_youbot_application.msg import SceneObjectMsg
import arcl_youbot_planner.base_planner.visgraph as vg
# import arcl_youbot_planner.arm_planner.prmstar as prmstar


class Object():
    def __init__(self):
        pass
        



def get_yaw_from_polygon(obj):
    long_x = obj[0][0] - obj[len(obj)-1][0]
    long_y = obj[0][1] - obj[len(obj)-1][1]
    long_length = math.sqrt(long_x*long_x + long_y*long_y)

    short_x = obj[0][0] - obj[1][0]
    short_y = obj[0][1] - obj[1][1]
    short_length = math.sqrt(short_x*short_x + short_y*short_y)
        
    temp_length = 0
    if long_length < short_length:
        long_x = short_x
        long_y = short_y
        temp_length = long_length
        long_length = short_length
        short_length = temp_length
    return math.atan(float(long_y/long_x))

#return information from a cube, defined as [[x0,y0], [x1,y1], [x2,y2], [x3,y3]]
#return (size, position, quaternion)
def get_info_from_cube(obj):    
    long_x = obj[0][0] - obj[len(obj)-1][0]
    long_y = obj[0][1] - obj[len(obj)-1][1]
    long_length = math.sqrt(long_x*long_x + long_y*long_y)

    short_x = obj[0][0] - obj[1][0]
    short_y = obj[0][1] - obj[1][1]
    short_length = math.sqrt(short_x*short_x + short_y*short_y)
    
    temp_length = 0
    if long_length < short_length:
        temp_length = long_length
        long_length = short_length
        short_length = temp_length

    center_x = 0
    center_y = 0
    for pt in obj:
        center_x += pt[0]
        center_y += pt[1]
    center_x = center_x / len(obj)
    center_y = center_y / len(obj)
    yaw = get_yaw_from_polygon(obj)

    rotation_matrix = np.array((
    (0, -math.sin(yaw), -math.cos(yaw), 0.0),
    (0, math.cos(yaw), -math.sin(yaw), 0.0),
    (1, 0, 0, 0.0),
    (0.0, 0.0, 0.0, 1.0)), dtype=np.float64)
    q = quaternion_from_matrix(rotation_matrix)
    qx = q[0]
    qy = q[1]
    qz = q[2]
    qw = q[3]
    short_length = 0.0376
    size = [short_length, short_length,long_length]
    position = [center_x, center_y, short_length / 2.0]
    quaternion = [qx, qy, qz, qw]
    return size, position, quaternion

def spawnCuboid(size, position, quaternion, color, object_name):
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    mass = 0.1
    dx = size[0]
    dy = size[1]
    dz = size[2]
    
    try:
        spawn_model = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)


        ss = StringIO.StringIO()
        #print >> ss, "<robot><link name='box'><inertial><origin xyz='0 0 0' /><mass value='0.3' /><inertia  ixx='0.002625115' ixy='0.0'  ixz='0.0'  iyy='0.002625115'  iyz='0.0'  izz='0.000023563' /></inertial><visual><origin xyz='0 0 0'/><geometry><box size='0.0376 0.0376 0.56' /></geometry></visual><collision><geometry><box size='0.0376 0.0376 0.56' /></geometry></collision></link><gazebo reference='box'><maxVel>0.0</maxVel><kp>10000.0</kp><kd>1.0</kd><minDepth>0.001</minDepth><mu1>200.0</mu1><mu2>200.0</mu2><material>Gazebo/Red</material></gazebo></robot>"
       
  
        print >> ss, "<robot><link name='box'><inertial><origin xyz='0 0 0' /><mass value='"
        print >> ss, str(mass) 
        print >> ss, "' />"
        print >> ss, "<inertia  ixx='" 
        print >> ss, str((1./12)*(dy*dy + dz*dz)*0.1) 
        print >> ss, "' ixy='0.0'  ixz='0.0'  iyy='"
        print >> ss, str((1./12)*(dx*dx + dz*dz)*0.1) 
        print >> ss, "'  iyz='0.0'  izz='"  
        print >> ss, str((1./12)*(dx*dx + dy*dy)*0.1) 
        print >> ss, "' /></inertial><visual><origin xyz='0 0 0'/><geometry>"
        print >> ss, "<box size='" 
        print >> ss, str(dx) + " " + str(dy) + " " + str(dz) + "' /></geometry></visual>"
        print >> ss, "<collision><geometry><box size='" + str(dx) + " " + str(dy) + " " + str(dz) + "' /></geometry>"
        print >> ss, "</collision></link>"
        print >> ss, "<gazebo reference='box'><maxVel>0.0</maxVel><kp>10000.0</kp><kd>1.0</kd><minDepth>0.001</minDepth><mu1>200.0</mu1>"
        print >> ss, "<mu2>200.0</mu2><material>" + str(color) + "</material></gazebo></robot>"

        #print ss.getvalue()

        model_xml = ss.getvalue()
       # model_xml = "<robot name = '1'><link name='my_link><visual><origin xyz='0 0 0' rpy='0 0 0' /><geometry><box size='1 1 1' /></geometry><material name='Cyan'><color rgba='0 1.0 1.0 1.0'/></material></visual></link><gazebo reference='box'><maxVel>0.0</maxVel><kp>10000.0</kp><kd>1.0</kd><minDepth>0.001</minDepth><mu1>200.0</mu1><mu2>200.0</mu2><material>Gazebo/Red</material></gazebo></robot>"
        #file_xml = open("/home/wei/catkin_youbot_ws/src/luh_youbot_description/robots/youbot_0.urdf")
        #model_xml = file_xml.read()
        reference_frame = "world"
        initial_pose = Pose()
        initial_pose.position.x = position[0]
        initial_pose.position.y = position[1]
        initial_pose.position.z = position[2]
        initial_pose.orientation.x = quaternion[0]
        initial_pose.orientation.y = quaternion[1]
        initial_pose.orientation.z = quaternion[2]
        initial_pose.orientation.w = quaternion[3]
                
        res = spawn_model(object_name, model_xml, " ", initial_pose, reference_frame)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
