import rospy
import math
import pybullet as p
import time
import pybullet_data
import numpy as np
import actionlib
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from arcl_youbot_msgs.msg import MoveBaseAction, MoveBaseGoal
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion


#youbot_name: youbot
def get_youbot_base_pose2d(youbot_name):
    data = rospy.wait_for_message('gazebo/model_states', ModelStates)
    current_pose = [0,0,0]
    youbot_index = 0
    for name, data_index in zip(data.name, range(len(data.name))):
        if name == youbot_name:
            youbot_index = data_index


    current_pose[0] = data.pose[youbot_index].position.x
    current_pose[1] = data.pose[youbot_index].position.y
    q = (data.pose[youbot_index].orientation.x,
             data.pose[youbot_index].orientation.y,
             data.pose[youbot_index].orientation.z,
             data.pose[youbot_index].orientation.w)
    (roll, pitch, yaw) = euler_from_quaternion(q)
    current_pose[2] = yaw
    return current_pose

#youbot_name: youbot
def get_youbot_base_pose(youbot_name):
    data = rospy.wait_for_message('gazebo/model_states', ModelStates)
    current_position = [0,0,0]
    current_orientation = [0,0,0,1]
    youbot_index = 0
    for name, data_index in zip(data.name, range(len(data.name))):
        if name == youbot_name:
            youbot_index = data_index


    current_position[0] = data.pose[youbot_index].position.x
    current_position[1] = data.pose[youbot_index].position.y
    current_position[2] = data.pose[youbot_index].position.z
    current_orientation[0] = data.pose[youbot_index].orientation.x
    current_orientation[1] = data.pose[youbot_index].orientation.y
    current_orientation[2] = data.pose[youbot_index].orientation.z
    current_orientation[3] = data.pose[youbot_index].orientation.w    
    return current_position, current_orientation

# base_action_name:   "youbot_base/move"
# work with base_controller.cpp updatePositionMode()
def execute_path(youbot_name, final_path, base_action_name):
    client = actionlib.SimpleActionClient(youbot_name + base_action_name, MoveBaseAction)
    client.wait_for_server()
    goal = MoveBaseGoal()
    begin_time = 0
    path = final_path[:]
    path.append((-111.0, -111.0, 0.0))
    for pt_index in range(len(path) - 1):
        goal.x = path[pt_index][0]
        goal.y = path[pt_index][1]
        goal.theta = path[pt_index][2]
        goal.next_x = path[pt_index+1][0]
        goal.next_y = path[pt_index+1][1]
        goal.next_theta = path[pt_index+1][2]
        print('Path', pt_index)
        print(goal.x, goal.y, goal.theta, goal.next_x, goal.next_y, goal.next_theta)
        client.send_goal_and_wait(goal, rospy.Duration.from_sec(10.0), rospy.Duration.from_sec(12.0))
        # client.wait_for_result(rospy.Duration.from_sec(10.0)) 

# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
from velocity_controller import VelocityController
from geometry_msgs.msg import Twist

# work with velocity_controller.py
def execute_path_vel_pub(youbot_name, final_path):
    vel_pub = rospy.Publisher('/' + youbot_name + '/robot/cmd_vel', Twist, queue_size=1)
    vc = VelocityController(youbot_name)
    vc.set_path(final_path)
    loop_rate = rospy.Rate(100)
    path_completed = False
    
    while not rospy.is_shutdown() and not path_completed: 
        msg = vc.get_velocity()  
        vel_pub.publish(msg)
        if msg.linear.x == 0.0 and msg.linear.y == 0.0 and msg.angular.z == 0.0:
            path_completed = True
        loop_rate.sleep()
# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

# ==================== visibility graph ====================
import arcl_youbot_planner.base_planner.visgraph as vg
from arcl_youbot_planner.base_planner.visgraph.visible_vertices import edge_distance
import commands
from shapely.geometry import Polygon, LinearRing, LineString
from shapely.ops import unary_union

YOUBOT_SHORT_RADIUS = 0.2  # in meters
YOUBOT_LONG_RADIUS = 0.32  # in meters
OFFSET = 0.1

TEST = False


def vg_find_path(start_pos, goal_pos, start_heading, goal_heading, obstacles):
    """
    Assume obstacles are represent by a list of in-order points
    """

    # ===== create free configuration space =====
    cpu_cores = int(commands.getstatusoutput('cat /proc/cpuinfo | grep processor | wc -l')[1])
    # enlarged obstacles based on YOUBOT_SHORT_RADIUS, join_style=2 flat, join_style=1 round
    dilated_obstacles = [Polygon(obs).buffer(YOUBOT_SHORT_RADIUS, join_style=2, mitre_limit=1.5) for obs in obstacles]
    # enlarged obstacles based on YOUBOT_LONG_RADIUS, join_style=2 flat, join_style=1 round
    dilated_large_obstacles = [Polygon(obs).buffer(YOUBOT_LONG_RADIUS, join_style=2, mitre_limit=1.5) for obs in obstacles]
    # union dialted obstacles
    union_dilated_obstacles = unary_union(dilated_obstacles)
    # union dialted large obstacles
    union_dilated_large_obstacles = unary_union(dilated_large_obstacles)
    # unary_union returns Polygon if only one polygon, otherwise, it returns MultiPolygon
    if not isinstance(union_dilated_obstacles, Polygon):
        # LinearRing: implicitly closed by copying the first tuple to the last index
        polygons = [[vg.Point(x, y) for x, y in dilated_obs.exterior.coords[:-1]] for dilated_obs in union_dilated_obstacles]
    else:
        polygons = [[vg.Point(x, y) for x, y in union_dilated_obstacles.exterior.coords]]

    # ===== get path from visibility graph =====
    g = vg.VisGraph()
    if TEST:
        g.build(polygons)
    else:
        g.build(polygons, workers=cpu_cores)
    path = g.shortest_path(vg.Point(start_pos[0], start_pos[1]), vg.Point(goal_pos[0], goal_pos[1]))

    # ===== change orientation of youbot so it can fit in this graph =====
    current_heading = start_heading
    path_with_heading = []
    for i in range(len(path) - 1):
        # add path
        path_with_heading.append((path[i].x, path[i].y, current_heading))

        # generate a line from current position to next
        line = LineString([(path[i].x, path[i].y), (path[i+1].x, path[i+1].y)])

        # find intersections & adjust orientation
        intersections = line.intersection(union_dilated_large_obstacles)
        if not intersections.is_empty:
            if not isinstance(intersections, LineString):
                intersection = intersections[0].coords[0]
            else:
                intersection = intersections.coords[0]
            vector = (path[i+1].x - path[i].x, path[i+1].y - path[i].y)
            current_heading = math.atan2(vector[1], vector[0])
            # if the vector starts inside a polygon (shapely returns intersection == path[i])
            if intersection == (path[i].x, path[i].y):
                path_with_heading.append((path[i].x, path[i].y, current_heading))
            # if the intersection comes before next path
            else:
                vector_norm = math.sqrt(vector[0]**2 + vector[1]**2)
                norm_vector = (vector[0] / vector_norm, vector[1] / vector_norm)
                offset_vecotr = (norm_vector[0] * OFFSET, norm_vector[1] * OFFSET)
                intersection_vector = (intersection[0] - path[i].x, intersection[1] - path[i].y)
                offset_intersection = (intersection[0] - offset_vecotr[0], intersection[1] - offset_vecotr[1])
                if abs(intersection_vector[0]) + abs(intersection_vector[1]) > abs(offset_vecotr[0]) + abs(offset_vecotr[1]):
                    path_with_heading.append((offset_intersection[0], offset_intersection[1], current_heading))
                else:
                    path_with_heading.append((path[i].x, path[i].y, current_heading))

    # back up 0.1 meters
    if abs(goal_heading - current_heading) > math.pi / 4:
        goal_pos_back_x = path[-1].x - math.cos(goal_heading) * OFFSET
        goal_pos_back_y = path[-1].y - math.sin(goal_heading) * OFFSET

        path_with_heading.append((path[-1].x, path[-1].y, current_heading))
        path_with_heading.append((goal_pos_back_x, goal_pos_back_y, goal_heading))
        path_with_heading.append((path[-1].x, path[-1].y, goal_heading))
    else:
        path_with_heading.append((path[-1].x, path[-1].y, current_heading))
        path_with_heading.append((path[-1].x, path[-1].y, goal_heading))

    return path_with_heading, g

def add_orientation(path, start_heading, goal_heading):
    """
    Assume the difference between start_heading and goal_heading is less than pi
    """

    # Calculate the distance between two points
    step_distance = []
    prev_point = path[0]
    for point in path[1:]:
        step_distance.append(edge_distance(prev_point, point))
        prev_point = point
    path_distance = sum(step_distance)

    # total rotation
    delta_rotation = goal_heading - start_heading

    # create list of path with heading
    current_heading = start_heading
    path_with_heading = [(path[0].x, path[0].y, current_heading)]
    for i in range(1, len(path)):
        current_heading += delta_rotation * step_distance[i-1] / path_distance
        path_with_heading.append((path[i].x, path[i].y, current_heading))

    return path_with_heading

def plot_vg_path(obstacles, path, g):
    from matplotlib import pyplot

    fig = pyplot.figure()
    ax = fig.subplots()

    # plot obstacles
    for o in obstacles:
        plot_line(ax, LinearRing(o))
          
    # plot visgraph
    for edge in g.visgraph.edges:
        x = [edge.p1.x, edge.p2.x]
        y = [edge.p1.y, edge.p2.y]
        plot_edge(ax, x, y, color='#6699cc', linewidth=1.5)

    # plot dilated obstacles
    for polygon in g.graph.polygons.values():
        for edge in polygon:
            x = [edge.p1.x, edge.p2.x]
            y = [edge.p1.y, edge.p2.y]
            plot_edge(ax, x, y, color='black', linewidth=2)

    # plot points
    for p in g.graph.get_points():
        ax.plot(p.x, p.y, marker='x', markersize=10, color='black')

    # plot path
    x = []
    y = []
    for point in path:
        x.append(point[0])
        y.append(point[1])
    plot_edge(ax, x, y, color='red', linewidth=0.5)

    # plot orientation
    for point in path:
        ax.plot(point[0], point[1], marker=(2, 1, math.degrees(point[2])-90), markersize=20, linestyle='None')

    # # plot offset
    # from shapely.figures import plot_line as plotline
    # for l in pl:
    #     plotline(ax, l, linewidth=1)
    # for r in pr:
    #     plotline(ax, r, linewidth=1)

    # # plot intersection
    # for l in plp:
    #     ax.plot(l[0], l[1], marker='o', markersize=10, linestyle='None')
    # for r in prp:
    #     ax.plot(r[0], r[1], marker='o', markersize=10, linestyle='None')
    
    pyplot.show()

def plot_line(ax, ob, color='#BDC3C7', zorder=1, linewidth=5, alpha=1):
    x, y = ob.xy
    ax.plot(x, y, color=color, linewidth=linewidth, solid_capstyle='round', zorder=zorder, alpha=alpha)
    
def plot_edge(ax, x, y, color='gray', zorder=1, linewidth=1, alpha=1):
    ax.plot(x, y, color=color, linewidth=linewidth, solid_capstyle='round', zorder=zorder, alpha=alpha)


# ==================== visibility graph ====================



# import sys, signal
# def signal_handler(signal, frame):
#     print("\nprogram exiting gracefully")
#     sys.exit(0)

# signal.signal(signal.SIGINT, signal_handler)

# from geometry_msgs.msg import Twist
# from pid import PID

# def position_to_velocity(path_with_heading):
#     print("From position to velocity:")

#     pid_x = PID(Kp=4.0, Ki=0.0, Kd=-0.1, output_limits=(-0.1, 0.1))
#     pid_y = PID(Kp=4.0, Ki=0.0, Kd=-0.1, output_limits=(-0.1, 0.1))
#     pid_theta = PID(Kp=2.0, Ki=0.0, Kd=-0.05, output_limits=(-0.1, 0.1))
#     vel_pub = rospy.Publisher('robot/cmd_vel', Twist, queue_size=1)
#     loop_rate = rospy.Rate(100)

#     i = 0
#     while i < len(path_with_heading):
#         # TODO: filter error
#         current_pos = get_youbot_base_pose2d("youbot")
#         target_pos = path_with_heading[i]
#         error_x = target_pos[0] - current_pos[0]
#         error_y = target_pos[1] - current_pos[1]
#         error_yaw = target_pos[2] - current_pos[2]
#         msg = Twist()
#         msg.linear.x = pid_x(error_x)
#         msg.linear.y = pid_y(error_y)
#         msg.angular.z = pid_theta(error_yaw)
#         vel_pub.publish(msg)
#         print(i, "current:",
#                  "{:.2f}".format(current_pos[0]), 
#                  "{:.2f}".format(current_pos[1]),
#                  "{:.2f}".format(current_pos[2]),
#                  "target:",
#                  "{:.2f}".format(target_pos[0]),
#                  "{:.2f}".format(target_pos[1]),
#                  "{:.2f}".format(target_pos[2]))
#         print(msg.linear.x, msg.linear.y, msg.angular.z)
        
#         if abs(error_x) + abs(error_y) < 0.1 and abs(error_yaw) < 0.174533:
#             i += 1
#         loop_rate.sleep()




if __name__ == "__main__":
    start_pos = (10, 2)
    goal_pos = (4, 9)
    obstacles = [[(1, 1), (2, 1), (2, 4), (1, 4)],
                 [(1.5, 5), (2.5, 5), (2.5, 6), (1.5, 6)],
                 [(3, 8), (5, 9), (4.5, 9.5), (2.8, 8.2)],
                 [(5, 3), (6, 3), (6, 4), (6, 5)],
                 [(7, 4), (8, 4), (8, 10), (7, 10)],
                 [(10, 5), (12, 5), (12, 7), (10, 7)]]
    start_heading = 0
    goal_heading = math.pi / 2
    path_with_heading, g = vg_find_path(start_pos, goal_pos, start_heading, goal_heading, obstacles)

    plot_vg_path(obstacles, path_with_heading, g)

    print("Path:")
    print(path_with_heading)

    rospy.init_node('robot_cmd_vel_publisher')
    # position_to_velocity(path_with_heading)
    
    