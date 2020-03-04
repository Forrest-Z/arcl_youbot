#!/usr/bin/env python
import rospy
import math
import pybullet as p
import time
import numpy as np
import pybullet_data
# import control_msgs.msg.FollowJointTrajectoryActionGoal
from geometry_msgs.msg import Twist 
from gazebo_msgs.srv import SpawnModel, SetModelState
from gazebo_msgs.msg import ModelState
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
from shapely.geometry import Polygon, LinearRing, LineString, MultiPolygon
from shapely.ops import unary_union
import random

# import arcl_youbot_planner.arm_planner.prmstar as prmstar

OBJECT_WIDTH = 0.035
OBJECT_LENGTH_MAX = 0.35
OBJECT_LENGTH_MIN = 0.15
USE_GAZEBO = 0
USE_OPTI = 1
CUBE_SIZE_DICT = {'obj_0': (0.04,0.04,0.242), 'obj_1': (0.04,0.04,0.237), 'obj_2': (0.04,0.04,0.185), 'obj_3': (0.04,0.04,0.238), 'obj_4': (0.04,0.04,0.15), 'obj_5': (0.04,0.04,0.168), 
                    'obj_6': (0.04,0.04,0.27), 'obj_7': (0.04,0.04,0.147), 'obj_8': (0.04,0.04,0.134), 'obj_9': (0.04,0.04,0.257), 'obj_10': (0.04,0.04,0.178), 
                    'obj_11': (0.04,0.04,0.199), 'obj_12': (0.04,0.04,0.195), 'obj_13': (0.04,0.04,0.257), 'obj_14': (0.04,0.04,0.257)}



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
        print >> ss, "<mu2>200.0</mu2><material>" + str(color) + "</material><gravity>0</gravity></gazebo></robot>"

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

def generate_poly(center_x, center_y, yaw, length, width):
    # x = length
    # y = width
    # p0 = (center_x + math.cos(yaw)*x - math.sin(yaw)*y, center_y - (math.sin(yaw)*x + math.cos(yaw)*y))
    # x = -length
    # y = width
    # p1 = (center_x + math.cos(yaw)*x - math.sin(yaw)*y, center_y - (math.sin(yaw)*x + math.cos(yaw)*y))
    # x = length
    # y = -width
    # p2 = (center_x + math.cos(yaw)*x - math.sin(yaw)*y, center_y - (math.sin(yaw)*x + math.cos(yaw)*y))
    # x = -length
    # y = -width
    # p3 = (center_x + math.cos(yaw)*x - math.sin(yaw)*y, center_y - (math.sin(yaw)*x + math.cos(yaw)*y))
    
    x = length
    y = width
    p0 = (center_x + math.cos(yaw)*x - math.sin(yaw)*y, center_y + (math.sin(yaw)*x + math.cos(yaw)*y))
    x = length
    y = -width
    p1 = (center_x + math.cos(yaw)*x - math.sin(yaw)*y, center_y + (math.sin(yaw)*x + math.cos(yaw)*y))
    x = -length
    y = -width
    p2 = (center_x + math.cos(yaw)*x - math.sin(yaw)*y, center_y + (math.sin(yaw)*x + math.cos(yaw)*y))
    x = -length
    y = width
    p3 = (center_x + math.cos(yaw)*x - math.sin(yaw)*y, center_y + (math.sin(yaw)*x + math.cos(yaw)*y))
    po = [p0, p1, p2, p3]
    po.reverse()
    poly = Polygon(po)
    return poly



		




def add_near_poly(same_cluster_objs, created_objs, x_min, x_max, y_min, y_max, boundary_padding):
    union_multi_polygon = MultiPolygon(same_cluster_objs)
    convex_hull_poly = union_multi_polygon.convex_hull
    close_pt = convex_hull_poly.centroid
    is_valid = False
    new_poly = None
    while not is_valid:
        is_valid = True
        direction = random.random() * 3.14159
        far_pt = (2500 * math.cos(direction) + close_pt.x, 2500 * math.sin(direction) + close_pt.y)
        cross_line = LineString([close_pt, far_pt])
        intersections = convex_hull_poly.intersection(cross_line)
        if not isinstance(intersections, LineString):
            center_pt = list(intersections[0].coords[0])
        else:
            center_pt = list(intersections.coords[0])
        center_pt[0] += boundary_padding * math.cos(direction)
        center_pt[1] += boundary_padding * math.sin(direction)
        yaw = random.random() * 3.14159
        length = random.random() * (OBJECT_LENGTH_MAX - OBJECT_LENGTH_MIN) + OBJECT_LENGTH_MIN
        width = OBJECT_WIDTH
        tp = generate_poly(center_pt[0], center_pt[1], yaw, length, width)
        for existing_obj in created_objs:
            if tp.intersects(existing_obj):
                is_valid = False
                break
        
        if not is_valid:
            continue
        
        new_poly = tp
    return new_poly 

def set_obj_pose(obj_name, pose):
    # change an object's pose in gazebo
    # INPUT: obj_name, string
    #        pose, geometry_msgs/Pose

    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_obj_pose = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        target_model_state = ModelState()
        target_model_state.model_name = obj_name
        target_model_state.reference_frame = "world"
        target_model_state.pose.position.x = pose.position.x
        target_model_state.pose.position.y = pose.position.y
        target_model_state.pose.position.z = pose.position.z
        target_model_state.pose.orientation.x = pose.orientation.x
        target_model_state.pose.orientation.y = pose.orientation.y
        target_model_state.pose.orientation.z = pose.orientation.z
        target_model_state.pose.orientation.w = pose.orientation.w
        set_obj_pose(target_model_state)
        
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e