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
from shapely.geometry import Polygon, LinearRing, LineString
from shapely.ops import unary_union

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


# def generate_polygon():
#     result.outer().clear();
# 	Point_2 upper_right, upper_left, down_right, down_left;
# 	double x, y;
# 	double length = diff_length / 2.0;  double width = diff_width / 2.0;
# 	x = length; y = width;
# 	if (std::abs(yaw - 1.57) < 0.001) {
# 		/*  
# 		upper_right.set<0>(center_x + cos(yaw)*x - sin(yaw)*y);
# 		upper_right.set<1>(center_y + sin(yaw)*x + cos(yaw)*y);
# 		x = -length; y = width;
# 		upper_left.set<0>(center_x + cos(yaw)*x - sin(yaw)*y);
# 		upper_left.set<1>(center_y + sin(yaw)*x + cos(yaw)*y);
# 		x = length; y = -width;
# 		down_right.set<0>(center_x + cos(yaw)*x - sin(yaw)*y);
# 		down_right.set<1>(center_y + sin(yaw)*x + cos(yaw)*y);
# 		x = -length; y = -width;
# 		down_left.set<0>(center_x + cos(yaw)*x - sin(yaw)*y);
# 		down_left.set<1>(center_y + sin(yaw)*x + cos(yaw)*y);
# 		*/
# 		upper_right.set<0>(center_x + y);
# 		upper_right.set<1>(center_y - x);

# 		upper_left.set<0>(center_x - y);
# 		upper_left.set<1>(center_y - x);

# 		down_right.set<0>(center_x + y);
# 		down_right.set<1>(center_y + x);

# 		down_left.set<0>(center_x - y);
# 		down_left.set<1>(center_y + x);
# 	}
# 	else if (std::abs(yaw - 0) < 0.0001) {
# 		upper_right.set<0>(center_x + x);
# 		upper_right.set<1>(center_y - y);
		
# 		upper_left.set<0>(center_x - x);
# 		upper_left.set<1>(center_y - y);
		
# 		down_right.set<0>(center_x + x);
# 		down_right.set<1>(center_y + y);

# 		down_left.set<0>(center_x - x);
# 		down_left.set<1>(center_y + y);
# 	}
# 	else {
# 		upper_right.set<0>(center_x + cos(yaw)*x - sin(yaw)*y);
# 		upper_right.set<1>(center_y - (sin(yaw)*x + cos(yaw)*y));
# 		x = -length; y = width;
# 		upper_left.set<0>(center_x + cos(yaw)*x - sin(yaw)*y);
# 		upper_left.set<1>(center_y - (sin(yaw)*x + cos(yaw)*y));
# 		x = length; y = -width;
# 		down_right.set<0>(center_x + cos(yaw)*x - sin(yaw)*y);
# 		down_right.set<1>(center_y - (sin(yaw)*x + cos(yaw)*y));
# 		x = -length; y = -width;
# 		down_left.set<0>(center_x + cos(yaw)*x - sin(yaw)*y);
# 		down_left.set<1>(center_y - (sin(yaw)*x + cos(yaw)*y));
# 	}
# 	bg::append(result.outer(), down_left);
# 	bg::append(result.outer(), upper_left);
# 	bg::append(result.outer(), upper_right);
# 	bg::append(result.outer(), down_right);
	
# 	bg::correct(result);
# }



# def 


# Polygon_2 MainWindow::addNewNearPoly(std::vector<Polygon_2> exist_polys, std::vector<Polygon_2> all_created_polys) {
# 	double yaw;
# 	double length = 145;
# 	double obj_between_dist = 80;
# 	//srand(time(NULL));
# 	double width = 30;
# 	Polygon_2 new_poly;
# 	std::uniform_int_distribution<int> distribution(0, 10000);
# 	std::random_device rd;
# 	std::mt19937 engine(rd());
# #ifdef ALL_FLAT
# 	yaw = 0;
# #else
# 	if (distribution(engine) / 10000. < 0.5) {
# 		yaw = 1.57;
# 	}
# 	else {
# 		yaw = 0;
# 	}
# #endif
# #ifdef NOT_AXIS_ALIGNED
# 	yaw = distribution(engine) / 10000. * 3.14;
# #endif
# 	Segment_2 border_1, border_2, border_3, border_4;
# #ifdef BIG_ENV
# 	border_1.first = Point_2(0, 0);
# 	border_1.second = Point_2(2000, 0);
# 	border_2.first = Point_2(2000, 0);     
# 	border_2.second = Point_2(2000, 2000);
# 	border_3.first = Point_2(2000, 2000);
# 	border_3.second = Point_2(0, 2000);
# 	border_4.first = Point_2(0, 2000);
# 	border_4.second = Point_2(0, 0);

# #endif

# #ifdef SMALL_ENV
# 	border_1.first = Point_2(0, 0);
# 	border_1.second = Point_2(1000, 0);
# 	border_2.first = Point_2(1000, 0);
# 	border_2.second = Point_2(1000, 1000);
# 	border_3.first = Point_2(1000, 1000);
# 	border_3.second = Point_2(0, 1000);
# 	border_4.first = Point_2(0, 1000);
# 	border_4.second = Point_2(0, 0);
# #endif
# 	std::vector<Polygon_2> final_outer;
# 	if (exist_polys.size() == 1) {
		
# 			Point_2 upper_left, upper_right, down_left, down_right;
# 			Polygon_2 outer_poly;
# 			// upper_left = exist_polys[0].outer()[1];
# 			// upper_right = exist_polys[0].outer()[2];
# 			// down_right = exist_polys[0].outer()[3];
# 			// down_left = exist_polys[0].outer()[0];
# 			getBoundingPoly(exist_polys[0], upper_left, upper_right, down_left, down_right, obj_between_dist);
# 			// if (yaw > 0) {
# 			// 	upper_left.set<0>(upper_left.get<0>() - obj_between_dist - width / 2);
# 			// 	upper_left.set<1>(upper_left.get<1>() - obj_between_dist - length / 2);
# 			// 	upper_right.set<0>(upper_right.get<0>() + obj_between_dist + width / 2);
# 			// 	upper_right.set<1>(upper_right.get<1>() - obj_between_dist - length / 2);
# 			// 	down_right.set<0>(down_right.get<0>() + obj_between_dist + width / 2);
# 			// 	down_right.set<1>(down_right.get<1>() + obj_between_dist + length / 2);
# 			// 	down_left.set<0>(down_left.get<0>() - obj_between_dist - width / 2);
# 			// 	down_left.set<1>(down_left.get<1>() + obj_between_dist + length / 2);
# 			// }
# 			// else if (yaw == 0) {
# 			// 	upper_left.set<0>(upper_left.get<0>() - obj_between_dist - length / 2);
# 			// 	upper_left.set<1>(upper_left.get<1>() - obj_between_dist - width / 2);
# 			// 	upper_right.set<0>(upper_right.get<0>() + obj_between_dist + length / 2);
# 			// 	upper_right.set<1>(upper_right.get<1>() - obj_between_dist - width / 2);
# 			// 	down_right.set<0>(down_right.get<0>() + obj_between_dist + length / 2);
# 			// 	down_right.set<1>(down_right.get<1>() + obj_between_dist + width / 2);
# 			// 	down_left.set<0>(down_left.get<0>() - obj_between_dist - length / 2);
# 			// 	down_left.set<1>(down_left.get<1>() + obj_between_dist + width / 2);
# 			// }
# 			bg::append(outer_poly.outer(), upper_left);
# 			bg::append(outer_poly.outer(), upper_right);
# 			bg::append(outer_poly.outer(), down_right);
# 			bg::append(outer_poly.outer(), down_left);
# 			bg::correct(outer_poly);

# 			bool is_valid = false;
# 			while (!is_valid) {
# 				new_poly.outer().clear();
# 				is_valid = true;
# 				double rotation = distribution(engine) / 10000. *(3.14 + 3.14) - 3.14;
# 				Point_2 origin;
# 				origin.set<0>((upper_left.get<0>() + upper_right.get<0>()) / 2);
# 				origin.set<1>((upper_left.get<1>() + down_left.get<1>()) / 2);
# 				Point_2 far_pt;
# 				far_pt.set<0>(500 * cos(rotation) + origin.get<0>());
# 				far_pt.set<1>(500 * sin(rotation) + origin.get<1>());
# 				Linestring_2 inter;
# 				bg::append(inter, origin);
# 				bg::append(inter, far_pt);
# 				std::vector<Point_2> inter_pt_list;
# 				bg::intersection(outer_poly, inter, inter_pt_list);
# 				if (inter_pt_list.size() == 1) {
# 					Point_2 inter_pt = inter_pt_list[0];
# 					#ifdef DIFFERENT_SIZE
# 						double different_length = (distribution(engine) / 10000. * 2.5 + 1) * OBJ_LENGTH;
# 						double different_width = OBJ_WIDTH;
# 						generatePoly(new_poly, inter_pt.get<0>(), inter_pt.get<1>(), yaw, different_length, different_width);

# 					#else
# 						generatePoly(new_poly, inter_pt.get<0>(), inter_pt.get<1>(), yaw);
# 					#endif
# 					// if (yaw > 0) {
# 					// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() - width / 2, inter_pt.get<1>() + length / 2));
# 					// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() - width / 2, inter_pt.get<1>() - length / 2));
# 					// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() + width / 2, inter_pt.get<1>() - length / 2));
# 					// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() + width / 2, inter_pt.get<1>() + length / 2));

# 					// 	bg::correct(new_poly);
# 					// }
# 					// else if (yaw == 0) {
# 					// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() - length / 2, inter_pt.get<1>() + width / 2));
# 					// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() - length / 2, inter_pt.get<1>() - width / 2));
# 					// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() + length / 2, inter_pt.get<1>() - width / 2));
# 					// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() + length / 2, inter_pt.get<1>() + width / 2));

# 					// 	bg::correct(new_poly);
# 					// }
# 					if (bg::intersects(new_poly, exist_polys[0])) {
# 						is_valid = false;
# 					}
# 					if (bg::intersects(new_poly, border_1) || bg::intersects(new_poly, border_2) || bg::intersects(new_poly, border_3) || bg::intersects(new_poly, border_4)) {
# 						is_valid = false;
# 					}
# #ifndef STACK
# 					for (int p = 0; p < all_created_polys.size(); p++) {
# 						if (bg::intersects(all_created_polys[p], new_poly)) {
# 							is_valid = false;
# 							break;
# 						}
# 					}
# #endif
# #ifdef SMALL_ENV
# 					if (bg::distance(new_poly, Point_2(500, 1000)) < 40) {
# 						is_valid = false;
# 					}
# #endif
# #ifdef BIG_ENV
# 					if (bg::distance(new_poly, Point_2(1000, 2000)) < 40) {
# 						is_valid = false;
# 					}
# #endif
# #ifdef BIG_ENV
# 					if (new_poly.outer()[0].get<1>() > 2000 || new_poly.outer()[0].get<1>() < 0 || new_poly.outer()[0].get<0>() > 2000 || new_poly.outer()[0].get<0>() < 0) {
# 						is_valid = false;
# 					}
# #endif
# #ifdef SMALL_ENV
# 					if (new_poly.outer()[0].get<1>() > 1000 || new_poly.outer()[0].get<1>() < 0 || new_poly.outer()[0].get<0>() > 1000 || new_poly.outer()[0].get<0>() < 0) {
# 						is_valid = false;
# 					}
# #endif
# 				}
# 				else {
# 					std::cout << "error: intersection output more than 1 pts" << std::endl;
# 				}
# 			}
# 	}
# 	else if (exist_polys.size() == 2) {
# 		Point_2 upper_left, upper_right, down_left, down_right;
# 		Polygon_2 outer_poly_1, outer_poly_2;

# 		getBoundingPoly(exist_polys[0], upper_left, upper_right, down_left, down_right, obj_between_dist);

# 		// upper_left = exist_polys[0].outer()[1];
# 		// upper_right = exist_polys[0].outer()[2];
# 		// down_right = exist_polys[0].outer()[3];
# 		// down_left = exist_polys[0].outer()[0];
# 		// if (yaw > 0) {
# 		// 	upper_left.set<0>(upper_left.get<0>() - obj_between_dist - width / 2);
# 		// 	upper_left.set<1>(upper_left.get<1>() - obj_between_dist - length / 2);
# 		// 	upper_right.set<0>(upper_right.get<0>() + obj_between_dist + width / 2);
# 		// 	upper_right.set<1>(upper_right.get<1>() - obj_between_dist - length / 2);
# 		// 	down_right.set<0>(down_right.get<0>() + obj_between_dist + width / 2);
# 		// 	down_right.set<1>(down_right.get<1>() + obj_between_dist + length / 2);
# 		// 	down_left.set<0>(down_left.get<0>() - obj_between_dist - width / 2);
# 		// 	down_left.set<1>(down_left.get<1>() + obj_between_dist + length / 2);
# 		// }
# 		// else if (yaw == 0) {
# 		// 	upper_left.set<0>(upper_left.get<0>() - obj_between_dist - length / 2);
# 		// 	upper_left.set<1>(upper_left.get<1>() - obj_between_dist - width / 2);
# 		// 	upper_right.set<0>(upper_right.get<0>() + obj_between_dist + length / 2);
# 		// 	upper_right.set<1>(upper_right.get<1>() - obj_between_dist - width / 2);
# 		// 	down_right.set<0>(down_right.get<0>() + obj_between_dist + length / 2);
# 		// 	down_right.set<1>(down_right.get<1>() + obj_between_dist + width / 2);
# 		// 	down_left.set<0>(down_left.get<0>() - obj_between_dist - length / 2);
# 		// 	down_left.set<1>(down_left.get<1>() + obj_between_dist + width / 2);
# 		// }
# 		bg::append(outer_poly_1.outer(), upper_left);
# 		bg::append(outer_poly_1.outer(), upper_right);
# 		bg::append(outer_poly_1.outer(), down_right);
# 		bg::append(outer_poly_1.outer(), down_left);
# 		bg::correct(outer_poly_1);


# 		getBoundingPoly(exist_polys[1], upper_left, upper_right, down_left, down_right, obj_between_dist);

# 		// upper_left = exist_polys[1].outer()[1];
# 		// upper_right = exist_polys[1].outer()[2];
# 		// down_right = exist_polys[1].outer()[3];
# 		// down_left = exist_polys[1].outer()[0];
# 		// if (yaw > 0) {
# 		// 	upper_left.set<0>(upper_left.get<0>() - obj_between_dist - width / 2);
# 		// 	upper_left.set<1>(upper_left.get<1>() - obj_between_dist - length / 2);
# 		// 	upper_right.set<0>(upper_right.get<0>() + obj_between_dist + width / 2);
# 		// 	upper_right.set<1>(upper_right.get<1>() - obj_between_dist - length / 2);
# 		// 	down_right.set<0>(down_right.get<0>() + obj_between_dist + width / 2);
# 		// 	down_right.set<1>(down_right.get<1>() + obj_between_dist + length / 2);
# 		// 	down_left.set<0>(down_left.get<0>() - obj_between_dist - width / 2);
# 		// 	down_left.set<1>(down_left.get<1>() + obj_between_dist + length / 2);
# 		// }
# 		// else if (yaw == 0) {
# 		// 	upper_left.set<0>(upper_left.get<0>() - obj_between_dist - length / 2);
# 		// 	upper_left.set<1>(upper_left.get<1>() - obj_between_dist - width / 2);
# 		// 	upper_right.set<0>(upper_right.get<0>() + obj_between_dist + length / 2);
# 		// 	upper_right.set<1>(upper_right.get<1>() - obj_between_dist - width / 2);
# 		// 	down_right.set<0>(down_right.get<0>() + obj_between_dist + length / 2);
# 		// 	down_right.set<1>(down_right.get<1>() + obj_between_dist + width / 2);
# 		// 	down_left.set<0>(down_left.get<0>() - obj_between_dist - length / 2);
# 		// 	down_left.set<1>(down_left.get<1>() + obj_between_dist + width / 2);
# 		// }
# 		bg::append(outer_poly_2.outer(), upper_left);
# 		bg::append(outer_poly_2.outer(), upper_right);
# 		bg::append(outer_poly_2.outer(), down_right);
# 		bg::append(outer_poly_2.outer(), down_left);
# 		bg::correct(outer_poly_2);

# 		std::vector<Polygon_2> union_polys;
# 		bg::union_(outer_poly_1, outer_poly_2, union_polys);
# 		if (union_polys.size() == 1) {
# 			bool is_valid = false;
# 			while(!is_valid){
				
# 			double rotation = distribution(engine) / 10000. *(3.14 + 3.14) - 3.14;
# 			Point_2 origin;
# 			origin.set<0>((upper_left.get<0>() + upper_right.get<0>()) / 2);
# 			origin.set<1>((upper_left.get<1>() + down_left.get<1>()) / 2);
# 			Point_2 far_pt;
# 			far_pt.set<0>(2500 * cos(rotation) + origin.get<0>());
# 			far_pt.set<1>(2500 * sin(rotation) + origin.get<1>());
# 			Linestring_2 inter;
# 			bg::append(inter, origin);
# 			bg::append(inter, far_pt);
# 			std::vector<Point_2> inter_pt_list;
# 			bg::intersection(union_polys[0], inter, inter_pt_list);
# 			is_valid = true;

# 			for (int k = 0; k < inter_pt_list.size(); k++) {
# 				Point_2 inter_pt = inter_pt_list[k];
# 				new_poly.outer().clear();
# 				// if (yaw > 0) {
# 				// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() - width / 2, inter_pt.get<1>() + length / 2));
# 				// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() - width / 2, inter_pt.get<1>() - length / 2));
# 				// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() + width / 2, inter_pt.get<1>() - length / 2));
# 				// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() + width / 2, inter_pt.get<1>() + length / 2));

# 				// 	bg::correct(new_poly);
# 				// }
# 				// else if (yaw == 0) {
# 				// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() - length / 2, inter_pt.get<1>() + width / 2));
# 				// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() - length / 2, inter_pt.get<1>() - width / 2));
# 				// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() + length / 2, inter_pt.get<1>() - width / 2));
# 				// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() + length / 2, inter_pt.get<1>() + width / 2));

# 				// 	bg::correct(new_poly);
# 				// }
# 				#ifdef DIFFERENT_SIZE
# 					double different_length = (distribution(engine) / 10000. * 2.5 + 1) * OBJ_LENGTH;
# 					double different_width = OBJ_WIDTH;
# 					generatePoly(new_poly, inter_pt.get<0>(), inter_pt.get<1>(), yaw, different_length, different_width);

# 				#else
# 					generatePoly(new_poly, inter_pt.get<0>(), inter_pt.get<1>(), yaw);
# 				#endif

# 				if (bg::intersects(new_poly, border_1) || bg::intersects(new_poly, border_2) || bg::intersects(new_poly, border_3) || bg::intersects(new_poly, border_4)) {
# 					is_valid = false;
# 				}
# #ifndef STACK
# 				for (int t = 0; t < exist_polys.size(); t++) {
# 					if (bg::intersects(exist_polys[t], new_poly)) {
# 						is_valid = false;
# 						break;
# 					}
# 					if (isTwoPolyTooClose(exist_polys[t], new_poly)) {
# 						is_valid = false;
# 						break;
# 					}

# 				}
# 				for (int p = 0; p < all_created_polys.size(); p++) {
# 					if (bg::intersects(all_created_polys[p], new_poly)) {
# 						is_valid = false;
# 						break;
# 					}
# 				}
# #endif
# #ifdef SMALL_ENV
# 				if (bg::distance(new_poly, Point_2(500, 1000)) < 40) {
# 					is_valid = false;
# 				}
# #endif
# #ifdef BIG_ENV
# 				if (bg::distance(new_poly, Point_2(1000, 2000)) < 40) {
# 					is_valid = false;
# 				}
# #endif
# #ifdef BIG_ENV
# 				if (new_poly.outer()[0].get<1>() > 2000 || new_poly.outer()[0].get<1>() < 0 || new_poly.outer()[0].get<0>() > 2000 || new_poly.outer()[0].get<0>() < 0) {
# 					is_valid = false;
# 				}
# #endif
# #ifdef SMALL_ENV
# 				if (new_poly.outer()[0].get<1>() > 1000 || new_poly.outer()[0].get<1>() < 0 || new_poly.outer()[0].get<0>() > 1000 || new_poly.outer()[0].get<0>() < 0) {
# 					is_valid = false;
# 				}
# #endif
# 				if (is_valid) {
# 					break;
# 				}
# 			}
# 			}
			
			
			
			
		
# 		}
# 		else {
# 			std::cout << "error(2 polys): union_ returns more than 1 polygons" << std::endl;
# 		}


# 	}
# 	else {
# 		Point_2 upper_left, upper_right, down_left, down_right;
		
# 		Polygon_2 outer_poly_1, outer_poly_2;
# 		Polygon_2 current_poly;
# 		std::vector<Polygon_2> outer_poly_vector;
# 		for (int k = 0; k < exist_polys.size(); k++) {
# 			current_poly.outer().clear();
# 			getBoundingPoly(exist_polys[k], upper_left, upper_right, down_left, down_right, obj_between_dist);

# 			// upper_left = exist_polys[k].outer()[1];
# 			// upper_right = exist_polys[k].outer()[2];
# 			// down_right = exist_polys[k].outer()[3];
# 			// down_left = exist_polys[k].outer()[0];
# 			// if (yaw > 0) {
# 			// 	upper_left.set<0>(upper_left.get<0>() - obj_between_dist - width / 2);
# 			// 	upper_left.set<1>(upper_left.get<1>() - obj_between_dist - length / 2);
# 			// 	upper_right.set<0>(upper_right.get<0>() + obj_between_dist + width / 2);
# 			// 	upper_right.set<1>(upper_right.get<1>() - obj_between_dist - length / 2);
# 			// 	down_right.set<0>(down_right.get<0>() + obj_between_dist + width / 2);
# 			// 	down_right.set<1>(down_right.get<1>() + obj_between_dist + length / 2);
# 			// 	down_left.set<0>(down_left.get<0>() - obj_between_dist - width / 2);
# 			// 	down_left.set<1>(down_left.get<1>() + obj_between_dist + length / 2);
# 			// }
# 			// else if (yaw == 0) {
# 			// 	upper_left.set<0>(upper_left.get<0>() - obj_between_dist - length / 2);
# 			// 	upper_left.set<1>(upper_left.get<1>() - obj_between_dist - width / 2);
# 			// 	upper_right.set<0>(upper_right.get<0>() + obj_between_dist + length / 2);
# 			// 	upper_right.set<1>(upper_right.get<1>() - obj_between_dist - width / 2);
# 			// 	down_right.set<0>(down_right.get<0>() + obj_between_dist + length / 2);
# 			// 	down_right.set<1>(down_right.get<1>() + obj_between_dist + width / 2);
# 			// 	down_left.set<0>(down_left.get<0>() - obj_between_dist - length / 2);
# 			// 	down_left.set<1>(down_left.get<1>() + obj_between_dist + width / 2);
# 			// }
# 			bg::append(current_poly.outer(), upper_left);
# 			bg::append(current_poly.outer(), upper_right);
# 			bg::append(current_poly.outer(), down_right);
# 			bg::append(current_poly.outer(), down_left);
# 			bg::correct(current_poly);
# 			outer_poly_vector.push_back(current_poly);
# 		}
# 		Polygon_2 first = outer_poly_vector.back();
# 		outer_poly_vector.pop_back();
# 		Polygon_2 second;
# 		std::vector<Polygon_2> union_polys;
# 		while (outer_poly_vector.size() > 0) {
# 			second = outer_poly_vector.back();
# 			bg::union_(first, second, union_polys);
# 			first = union_polys[0];
# 			outer_poly_vector.pop_back();
# 		}
# 		bool is_valid = false;
# 		while (!is_valid) {

# 			double rotation = distribution(engine) / 10000. *(3.14 + 3.14) - 3.14;
			
# 			//srand(rand()%10000);
# 			Point_2 origin;
# 			origin.set<0>((upper_left.get<0>() + upper_right.get<0>()) / 2);
# 			origin.set<1>((upper_left.get<1>() + down_left.get<1>()) / 2);
# 			Point_2 far_pt;
# 			far_pt.set<0>(2500 * cos(rotation) + origin.get<0>());
# 			far_pt.set<1>(2500 * sin(rotation) + origin.get<1>());
# 			Linestring_2 inter;
# 			bg::append(inter, origin);
# 			bg::append(inter, far_pt);
# 			std::vector<Point_2> inter_pt_list;
# 			bg::intersection(first, inter, inter_pt_list);

# 			for (int k = 0; k < inter_pt_list.size(); k++) {
# 				is_valid = true;
# 				Point_2 inter_pt = inter_pt_list[k];
# 				new_poly.outer().clear();
# 				// if (yaw > 0) {
# 				// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() - width / 2, inter_pt.get<1>() + length / 2));
# 				// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() - width / 2, inter_pt.get<1>() - length / 2));
# 				// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() + width / 2, inter_pt.get<1>() - length / 2));
# 				// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() + width / 2, inter_pt.get<1>() + length / 2));

# 				// 	bg::correct(new_poly);
# 				// }
# 				// else if (yaw == 0) {
# 				// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() - length / 2, inter_pt.get<1>() + width / 2));
# 				// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() - length / 2, inter_pt.get<1>() - width / 2));
# 				// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() + length / 2, inter_pt.get<1>() - width / 2));
# 				// 	bg::append(new_poly.outer(), Point_2(inter_pt.get<0>() + length / 2, inter_pt.get<1>() + width / 2));

# 				// 	bg::correct(new_poly);
# 				// }

# 				#ifdef DIFFERENT_SIZE
# 					double different_length = (distribution(engine) / 10000. * 2.5 + 1) * OBJ_LENGTH;
# 					double different_width = OBJ_WIDTH;
# 					generatePoly(new_poly, inter_pt.get<0>(), inter_pt.get<1>(), yaw, different_length, different_width);

# 				#else
# 					generatePoly(new_poly, inter_pt.get<0>(), inter_pt.get<1>(), yaw);
# 				#endif
# 				if (bg::intersects(new_poly, border_1) || bg::intersects(new_poly, border_2) || bg::intersects(new_poly, border_3) || bg::intersects(new_poly, border_4)) {
# 					is_valid = false;
# 				}
# #ifndef STACK
# 				for (int t = 0; t < exist_polys.size(); t++) {
# 					if (bg::intersects(exist_polys[t], new_poly)) {
# 						is_valid = false;
# 						break;
# 					}
# 					if (isTwoPolyTooClose(exist_polys[t], new_poly)) {
# 						is_valid = false;
# 						break;
# 					}

# 				}
# 				for (int p = 0; p < all_created_polys.size(); p++) {
# 					if (bg::intersects(all_created_polys[p], new_poly)) {
# 						is_valid = false;
# 						break;
# 					}
# 				}
# #endif
# #ifdef SMALL_ENV
# 				if (bg::distance(new_poly, Point_2(500, 1000)) < 40) {
# 					is_valid = false;
# 				}
# #endif
# #ifdef BIG_ENV
# 				if (bg::distance(new_poly, Point_2(2500, 5000)) < 40) {
# 					is_valid = false;
# 				}
# #endif
# #ifdef BIG_ENV
# 				if (new_poly.outer()[0].get<1>() > 5000 || new_poly.outer()[0].get<1>() < 0 || new_poly.outer()[0].get<0>() > 5000 || new_poly.outer()[0].get<0>() < 0) {
# 					is_valid = false;
# 				}
# #endif
# #ifdef SMALL_ENV
# 				if (new_poly.outer()[0].get<1>() > 1000 || new_poly.outer()[0].get<1>() < 0 || new_poly.outer()[0].get<0>() > 1000 || new_poly.outer()[0].get<0>() < 0) {
# 					is_valid = false;
# 				}
# #endif
# 				if (is_valid) {
# 					break;
# 				}
# 			}
# 		}
		

# 	}

# 	return new_poly;
# }
