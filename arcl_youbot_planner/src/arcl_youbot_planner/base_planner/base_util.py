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



import arcl_youbot_planner.base_planner.visgraph as vg
from arcl_youbot_planner.base_planner.visgraph.visible_vertices import edge_distance
import commands
from shapely.geometry import Polygon, LinearRing
from shapely.ops import unary_union
import math

YOUBOT_RADIUS = 0.32  # in meters
TEST = False





#youbot_name: youbot
def get_youbot_base_pose(youbot_name):
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

# base_action_name:   "youbot_base/move"
def execute_path(final_path, base_action_name):
    client = actionlib.SimpleActionClient(base_action_name, MoveBaseAction)
    client.wait_for_server()
    goal = MoveBaseGoal()
    begin_time = 0
    for pt_index in range(len(final_path)):
        goal.x = final_path[pt_index][0]
        goal.y = final_path[pt_index][1]
        goal.theta = final_path[pt_index][2]
        client.send_goal_and_wait(goal, rospy.Duration.from_sec(10.0), rospy.Duration.from_sec(10.0))
        # client.wait_for_result(rospy.Duration.from_sec(10.0)) 
        
        















def vg_find_path(start_pos, goal_pos, obstacles):
    """
    Assume obstacles are represent by a list of in-order points
    """

    # multi-threads
    cpu_cores = int(commands.getstatusoutput('cat /proc/cpuinfo | grep processor | wc -l')[1])

    # enlarged obstacles based on YOUBOT_RADIUS, join_style=2 flat, join_style=1 round
    dilated_obstacles = [Polygon(obs).buffer(YOUBOT_RADIUS, join_style=2, mitre_limit=1.5) for obs in obstacles]

    # union dialted obstacles
    union_dilated_obstacles = unary_union(dilated_obstacles)

    # unary_union returns Polygon if only one polygon, otherwise, it returns MultiPolygon
    if not isinstance(union_dilated_obstacles, Polygon):
        # LinearRing: implicitly closed by copying the first tuple to the last index
        polygons = [[vg.Point(x, y) for x, y in dilated_obs.exterior.coords[:-1]] for dilated_obs in union_dilated_obstacles]
    else:
        polygons = [[vg.Point(x, y) for x, y in union_dilated_obstacles.exterior.coords]]

    g = vg.VisGraph()
    if TEST:
        g.build(polygons)
    else:
        g.build(polygons, workers=cpu_cores)
    path = g.shortest_path(vg.Point(start_pos[0], start_pos[1]), vg.Point(goal_pos[0], goal_pos[1]))

    return path, g

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
        ax.plot(point[0], point[1], marker=(2, 1, -math.degrees(point[2])), markersize=20, linestyle='None')
    
    pyplot.show()

def plot_line(ax, ob, color='#BDC3C7', zorder=1, linewidth=5, alpha=1):
    x, y = ob.xy
    ax.plot(x, y, color=color, linewidth=linewidth, solid_capstyle='round', zorder=zorder, alpha=alpha)
    
def plot_edge(ax, x, y, color='gray', zorder=1, linewidth=1, alpha=1):
    ax.plot(x, y, color=color, linewidth=linewidth, solid_capstyle='round', zorder=zorder, alpha=alpha)



       
