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
from geometry_msgs.msg import PoseStamped
from velocity_controller import VelocityController
from geometry_msgs.msg import Twist
import arcl_youbot_planner.base_planner.visgraph as vg
from arcl_youbot_planner.base_planner.visgraph.visible_vertices import edge_distance
import commands
from shapely.geometry import Polygon, LinearRing, LineString, Point
from shapely.ops import unary_union
import rvo2

import sys, signal
def signal_handler(signal, frame):
    print("\nprogram exiting gracefully")
    sys.exit(0)


LOW_SPEED = 0.005           # smaller than this will be considered not moving
LOW_SPEED_TIMEOUT = 3       # if not moving over seconds, then stop
YOUBOT_SHORT_RADIUS = 0.25  # used to dilated obstacles (width / 2)
YOUBOT_LONG_RADIUS = 0.40   # used to dilated obstacles (diagonal / 2)
ADJUST_DISTANCE = YOUBOT_LONG_RADIUS - YOUBOT_SHORT_RADIUS + 0.2  # used for new_vg_path to decide use path from small or large dilated obstacles
BACK_DISTANCE = 0.1         # buffer distance to the final goal pos
SHORT_ANGLE = -111          # sign to indicate path from small obstacles
NEAR_RATIO = 0.2            # distance ratio from a large pos to small pos
LENGTH_TOL = 1.0          # length of path tolerance

TEST = False                # multi-core acceleration for visibility graph


class BaseController():
    """ 
    This is used to publish velocity
    """

    def __init__(self, youbot_name, mode):
        self.mode = mode
        self.is_pose_received = False
        self.current_pose_2d = [0,0,0]
        self.youbot_name = youbot_name

        self.vel_pub = rospy.Publisher('/' + youbot_name + '/robot/cmd_vel', Twist, queue_size=1)
        if mode == 0:
            rospy.Subscriber('/gazebo/model_states', ModelStates, self.base_pose2d_callback, [youbot_name])

    def base_pose2d_callback(self, data, args):
        """ Gazebo: callback to receive the current youbot position
        """
        youbot_index = 0

        for name, data_index in zip(data.name, range(len(data.name))):
            if name == args[0]:
                youbot_index = data_index
        self.current_pose_2d[0] = data.pose[youbot_index].position.x
        self.current_pose_2d[1] = data.pose[youbot_index].position.y
        q = (data.pose[youbot_index].orientation.x,
                data.pose[youbot_index].orientation.y,
                data.pose[youbot_index].orientation.z,
                data.pose[youbot_index].orientation.w)
        (_, _, yaw) = euler_from_quaternion(q)
        self.current_pose_2d[2] = yaw
        self.is_pose_received = True

    def execute_path_vel_pub(self, final_path):
        """ compute and publish velocity
        """
        vc = VelocityController(self.youbot_name)
        vc.set_path(final_path)
        loop_rate = rospy.Rate(100)
        path_completed = False
        
        # ===== see how the velocity converge =====
        # x_vel_log = []
        # y_vel_log = []
        # =========================================
        record_time = -1
        while not rospy.is_shutdown() and not path_completed: 
            current_pose = self.get_youbot_base_pose2d()
            msg = vc.get_velocity(self.youbot_name, current_pose, self.mode)
            self.vel_pub.publish(msg)
            # x_vel_log.append(msg.linear.x)
            # y_vel_log.append(msg.linear.y)
            if msg.linear.x == 0.0 and msg.linear.y == 0.0 and msg.angular.z == 0.0:
                path_completed = True
            if msg.linear.x < LOW_SPEED and msg.linear.y < LOW_SPEED and msg.angular.z < LOW_SPEED:
                if record_time == -1:
                    record_time = time.time()
                elif time.time() - record_time > LOW_SPEED_TIMEOUT:
                    path_completed = True
            else:
                record_time = time.time()

            loop_rate.sleep()

        # ===== see how the velocity converge =====
        # if mode == 1:
        #     from matplotlib import pyplot

        #     fig = pyplot.figure()
        #     ax = fig.subplots()

        #     ax.plot(range(len(x_vel_log)), x_vel_log, color='blue')
        #     ax.plot(range(len(y_vel_log)), y_vel_log, color='red')

        #     pyplot.show()


    def get_youbot_base_pose2d(self):
        """ get the current youbot position
        """
        if self.mode == 0:
            while self.is_pose_received == False:
                pass
            self.is_pose_received = False
            return self.current_pose_2d
        elif self.mode == 1:
            data = rospy.wait_for_message('/vrpn_client_node/' + self.youbot_name + '/pose', PoseStamped)
            
            current_pose = [0, 0, 0]
            current_pose[0] = data.pose.position.x
            current_pose[1] = data.pose.position.y
            q = (data.pose.orientation.x,
                    data.pose.orientation.y,
                    data.pose.orientation.z,
                    data.pose.orientation.w)
            (_, _, yaw) = euler_from_quaternion(q)
            current_pose[2] = yaw
            return current_pose

    def get_youbot_base_pose(self):
        """ get the current youbot position
        """
        if self.mode == 0:
            while self.is_pose_received == False:
                pass
            self.is_pose_received = False
            return self.current_pose_2d
        elif self.mode == 1:
            data = rospy.wait_for_message('/vrpn_client_node/' + self.youbot_name + '/pose', PoseStamped)
            
            current_pose = data.pose
            return current_pose


class MultiBaseController():
    """ 
    This is used to publish velocity for multi-YouBot system
    """
    def __init__(self, youbot_names, mode):
        self.youbot_names = youbot_names
        self.mode = mode
        self.base_controllers = [None] * len(youbot_names)
        for name in youbot_names:
            self.base_controllers = BaseController(name, mode)

    def pub_all_vel(self, final_paths, object_list):
        # TODO: assume vg_find_large_path for now
        vcs = [None] * len(final_paths)
        paths_completed = [False] * len(final_paths)
        record_times = [-1] * len(final_paths)
        # loop_rate = rospy.Rate(100)
        for index, final_path in enumerate(final_paths):
            vc = VelocityController(self.youbot_names[index])
            vc.set_path(final_path)
            vcs[index] = (vc)

        # timeStep, neighborDist, maxNeighbors, timeHorizon, timeHorizonObst, radius, maxSpeed
        sim = rvo2.PyRVOSimulator(1/100., 2 * YOUBOT_LONG_RADIUS, 5, 1.0, 0.5, YOUBOT_LONG_RADIUS, vcs[0].x_pid._max_output)
        agents = [None] * len(final_paths)
        for index, final_path in enumerate(final_paths):
            current_pose = self.base_controllers[index].get_youbot_base_pose2d()
            a = sim.addAgent((current_pose[0], current_pose[1]))
            agents[index] = (a)
        for obs in object_list:
            sim.addObstacle(obs)
        sim.processObstacles()
        print('Simulation has %i agents and %i obstacle vertices in it.' % (sim.getNumAgents(), sim.getNumObstacleVertices()))
        


        msgs = [None] * len(final_paths)
        while not rospy.is_shutdown() and all(paths_completed): 
            for index, agent in enumerate(agents):
                current_pose = self.base_controllers[index].get_youbot_base_pose2d()
                vel = vc.get_velocity(self.youbot_names[index], current_pose, self.mode)
                if vel.linear.x == 0.0 and vel.linear.y == 0.0 and vel.angular.z == 0.0:
                    paths_completed[index] = True
                if vel.linear.x < LOW_SPEED and vel.linear.y < LOW_SPEED and vel.angular.z < LOW_SPEED:
                    if record_times[index] == -1:
                        record_times[index] = time.time()
                    elif time.time() - record_times[index] > LOW_SPEED_TIMEOUT:
                        paths_completed[index] = True
                else:
                    record_times[index] = time.time()
                msgs[index] = vel

                sim.setAgentPosition(agent, (current_pose[0], current_pose[1]))
                sim.setAgentPrefVelocity(agent, (vel.linear.x, vel.linear.y))
            
            sim.doStep()

            for index, base_controller in enumerate(self.base_controllers):
                vel = sim.getAgentVelocity(agents[index])
                msgs[index].linear.x = vel[0]
                msgs[index].linear.y = vel[1]
                base_controller.vel_pub.publish(msgs[index])

            # TODO: probably we don't need the sleep, because rvo2 takes times
            # loop_rate.sleep()


# ==================== visibility graph ====================
def vg_find_small_path(start_pos, goal_pos, start_heading, goal_heading, obstacles):
    """ Use YOUBOT_SHORT_RADIUS to dilate obstacles and find the path based on it
        We need to care about the rotation, because it might hit obstacles
        assume obstacles are represent by a list of in-order points
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
            if i == 0:
                if abs(start_heading - current_heading) > math.pi / 2:
                    current_heading = current_heading - math.pi
            else:
                if abs(goal_heading - current_heading) > math.pi / 2:
                    current_heading = current_heading - math.pi
            if current_heading > math.pi:
                current_heading -= 2*math.pi
            elif current_heading < -math.pi:
                current_heading += 2*math.pi
            # if the vector starts inside a polygon (shapely returns intersection == path[i])
            if intersection == (path[i].x, path[i].y):
                path_with_heading.append((path[i].x, path[i].y, current_heading))
            # if the intersection comes before next path
            else:
                path_with_heading.append((intersection[0], intersection[1], current_heading))

    if not intersections.is_empty:
        if not isinstance(intersections, LineString):
            intersection = intersections[-2].coords[-1]
        else:
            intersection = intersections.coords[-2]
        # need more time to rotate without collision
        if abs(abs(goal_heading) - abs(current_heading)) > math.pi / 4:
            goal_pos_back_x = intersection[0] - math.cos(current_heading) * BACK_DISTANCE
            goal_pos_back_y = intersection[1] - math.sin(current_heading) * BACK_DISTANCE
            path_with_heading.append((goal_pos_back_x, goal_pos_back_y, current_heading))   
            # path_with_heading.append((intersection[0], intersection[1], goal_heading)) 
        else:
            path_with_heading.append((intersection[0], intersection[1], current_heading))
    path_with_heading.append((path[-1].x, path[-1].y, goal_heading))

    return path_with_heading, g

def vg_find_large_path(start_pos, goal_pos, start_heading, goal_heading, obstacles):
    """ Use YOUBOT_LONG_RADIUS to dilate obstacles and find the path based on it
        We do not need to care about the rotation, because it won't hit obstacles
        Assume obstacles are represent by a list of in-order points
    """

    # ===== create free configuration space =====
    cpu_cores = int(commands.getstatusoutput('cat /proc/cpuinfo | grep processor | wc -l')[1])
    # enlarged obstacles based on YOUBOT_SHORT_RADIUS, join_style=2 flat, join_style=1 round
    dilated_obstacles = [Polygon(obs).buffer(YOUBOT_LONG_RADIUS, join_style=2, mitre_limit=1.5) for obs in obstacles]
    # union dialted obstacles
    union_dilated_obstacles = unary_union(dilated_obstacles)
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
    temp_heading = abs(goal_heading - start_heading)
    total_distance = 0
    for i in range(len(path) - 2):
        total_distance += math.sqrt((path[i+1].x-path[i].x)**2 + (path[i+1].y-path[i].y)**2)
    if temp_heading > math.pi:
        if goal_heading - start_heading > 0:
            delta_heading = -(math.pi*2 - goal_heading + start_heading) / total_distance
        else:
            delta_heading = (math.pi*2 + goal_heading - start_heading) / total_distance
    else:
        delta_heading = (goal_heading - start_heading) / total_distance
    path_with_heading = []
    for i in range(len(path) - 1):
        # add path
        path_with_heading.append((path[i].x, path[i].y, current_heading))
        current_heading += delta_heading * math.sqrt((path[i+1].x-path[i].x)**2 + (path[i+1].y-path[i].y)**2)

    path_with_heading.append((path[-1].x, path[-1].y, goal_heading))

    return path_with_heading, g

def vg_find_combined_path(start_pos, goal_pos, start_heading, goal_heading, obstacles):
    """ Use YOUBOT_LONG_RADIUS and YOUBOT_SHORT_RADIUS to dilate obstacles and find the path based on them,
        we use the large path as the first choice, however, if the small path provides 
        shorter path (evaluate based on LENGTH_CHOOSE and ADJUST_DISTANCE), then, 
        we switch to the small path (The large path may detour).
        We will switch back if small path and large path have merged back again
        
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
    # unary_union returns Polygon if only one polygon, otherwise, it returns MultiPolygon
    if not isinstance(union_dilated_large_obstacles, Polygon):
        # LinearRing: implicitly closed by copying the first tuple to the last index
        large_polygons = [[vg.Point(x, y) for x, y in dilated_obs.exterior.coords[:-1]] for dilated_obs in union_dilated_large_obstacles]
    else:
        large_polygons = [[vg.Point(x, y) for x, y in union_dilated_large_obstacles.exterior.coords]]

    # ===== get path from visibility graph =====
    small_g = vg.VisGraph()
    if TEST:
        small_g.build(polygons)
    else:
        small_g.build(polygons, workers=cpu_cores)
    path = small_g.shortest_path(vg.Point(start_pos[0], start_pos[1]), vg.Point(goal_pos[0], goal_pos[1]))
    print("vg small path", path)

    large_g = vg.VisGraph()
    if TEST:
        large_g.build(large_polygons)
    else:
       large_g.build(large_polygons, workers=cpu_cores)
    large_path = large_g.shortest_path(vg.Point(start_pos[0], start_pos[1]), vg.Point(goal_pos[0], goal_pos[1]))
    print("vg large path", large_path)

    # ===== group two paths =====
    adjust_path = []
    distance_small_path = vg_path_distance(path)
    distance_large_path = vg_path_distance(large_path)
    if distance_large_path > LENGTH_TOL + distance_small_path:
        adjust_path.append([path[0].x, path[0].y, SHORT_ANGLE])  
        if union_dilated_large_obstacles.intersection(Point(large_path[0].x, large_path[0].y)).is_empty:
            li = 1
        else:
            li = 2
        for i in range(1, len(path)):
            if point_distance(path[i], large_path[li]) <= ADJUST_DISTANCE:
                adjust_path.append([large_path[li].x, large_path[li].y, 0])    
                li += 1
            else:
                adjust_path.append([path[i].x, path[i].y, SHORT_ANGLE])
                temp = li
                while li < len(large_path) and point_distance(path[i+1], large_path[li]) > ADJUST_DISTANCE:
                    li += 1
                if li == len(large_path):
                    li = temp
    else:
        for li in range(len(large_path)):
            adjust_path.append([large_path[li].x, large_path[li].y, 0])    
    
    # ===== add heading =====
    # distance
    total_distance = 0
    for i in range(len(adjust_path) - 1):
        if adjust_path[i][-1] != SHORT_ANGLE and adjust_path[i+1][-1] != SHORT_ANGLE:
            total_distance += point_distance(adjust_path[i], adjust_path[i+1])
        elif (adjust_path[i][-1] == SHORT_ANGLE) and (adjust_path[i+1][-1] != SHORT_ANGLE):
            total_distance += point_distance(adjust_path[i], adjust_path[i+1]) * NEAR_RATIO
    if total_distance == 0: total_distance = 1
    # delta rotation
    temp_heading = abs(goal_heading - start_heading)
    if temp_heading > math.pi:
        if goal_heading - start_heading > 0:
            delta_heading = -(math.pi * 2 - goal_heading + start_heading) / total_distance
        else:
            delta_heading = (math.pi * 2 + goal_heading - start_heading) / total_distance
    else:
        delta_heading = (goal_heading - start_heading) / total_distance
    # add heading to large
    current_heading = start_heading
    for i in range(len(adjust_path) - 1):
        if adjust_path[i][-1] != SHORT_ANGLE:
            adjust_path[i][-1] = current_heading
            if adjust_path[i+1][-1] != SHORT_ANGLE:
                current_heading += delta_heading * point_distance(adjust_path[i+1], adjust_path[i])
                if current_heading > math.pi:
                    current_heading -= 2*math.pi
                elif current_heading < -math.pi:
                    current_heading += 2*math.pi
        else:
            if adjust_path[i+1][-1] != SHORT_ANGLE:
                current_heading += delta_heading * point_distance(adjust_path[i+1], adjust_path[i]) * NEAR_RATIO
                if current_heading > math.pi:
                    current_heading -= 2*math.pi
                elif current_heading < -math.pi:
                    current_heading += 2*math.pi
    # add headding to small
    i = 0
    while i < len(adjust_path) - 1:
        if adjust_path[i][-1] == SHORT_ANGLE:
            s = adjust_path[i]
            g = adjust_path[i+1]
            current_heading = compute_heading(s, g, i, adjust_path, goal_heading)
            adjust_path[i][-1] = current_heading
            i += 1
            if g[-1] == SHORT_ANGLE:
                adjust_path.insert(i, [g[0], g[1], current_heading])
            else:
                adjust_path.insert(i, [(g[0] - s[0]) * (1 - NEAR_RATIO) + s[0], (g[1] - s[1]) * (1 - NEAR_RATIO) + s[1], current_heading])
            i += 1
        else:
            if adjust_path[i+1][-1] == SHORT_ANGLE:
                s = adjust_path[i]
                g = adjust_path[i+1]
                current_heading = compute_heading(s, g, i, adjust_path, goal_heading)
                i += 1
                adjust_path.insert(i, [(g[0] - s[0]) * NEAR_RATIO + s[0], (g[1] - s[1]) * NEAR_RATIO + s[1], current_heading])
                i += 1
                adjust_path.insert(i, [g[0], g[1], current_heading])
            i += 1

    adjust_path[-1][-1] = goal_heading

    return adjust_path, small_g, large_g

def compute_heading(s, g, i, adjust_path, goal_heading):
    """ helper for vg_find_combined_path
        compute heading for small path so it won't hit obstacles
    """
    while i < len(adjust_path) and adjust_path[i][-1] == SHORT_ANGLE:
        i += 1
    if i < len(adjust_path):
        target_heading = adjust_path[i][-1]
    else:
        target_heading = goal_heading
    vector = (g[0] - s[0], g[1] - s[1])
    current_heading = math.atan2(vector[1], vector[0])
    if abs(target_heading - current_heading) > math.pi / 2:
        current_heading -= math.pi
    if current_heading > math.pi:
        current_heading -= 2*math.pi
    elif current_heading < -math.pi:
        current_heading += 2*math.pi

    return current_heading

def vg_path_distance(path):
    """ helper for vg_find_combined_path
        compute the distance of a path from VG
    """
    path_distance = 0
    prev_point = path[0]
    for point in path[1:]:
        path_distance += point_distance(prev_point, point)
        prev_point = point
    return path_distance

def point_distance(p, q):
    """ helper for vg_find_combined_path
        compute the distance between two points
    """
    if (isinstance(q, list) or isinstance(q, tuple)) and isinstance(p, list):
        return math.sqrt((p[0] - q[0])**2 + (p[1] - q[1])**2)
    else:
        return math.sqrt((p.x - q.x)**2 + (p.y - q.y)**2)
# ===========================================================

def plot_vg_path(obstacles, path, g, large_g=None):
    from matplotlib import pyplot

    fig = pyplot.figure()
    ax = fig.subplots()

    # plot obstacles
    print(len(obstacles))
    for o in obstacles:
        plot_line(ax, LinearRing(o), linewidth=1.5)
          
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

    # plot dilated obstacles
    for polygon in large_g.graph.polygons.values():
        for edge in polygon:
            x = [edge.p1.x, edge.p2.x]
            y = [edge.p1.y, edge.p2.y]
            plot_edge(ax, x, y, color='blue', linewidth=2)

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

    ax.set_ylim(-5.5, 6.5)
    ax.set_xlim(-4, 4)
    ax.set_aspect("equal")
    
    pyplot.show()

def plot_line(ax, ob, color='#BDC3C7', zorder=1, linewidth=5, alpha=1):
    x, y = ob.xy
    ax.plot(x, y, color=color, linewidth=linewidth, solid_capstyle='round', zorder=zorder, alpha=alpha)
    
def plot_edge(ax, x, y, color='gray', zorder=1, linewidth=1, alpha=1):
    ax.plot(x, y, color=color, linewidth=linewidth, solid_capstyle='round', zorder=zorder, alpha=alpha)


if __name__ == "__main__":
    from arcl_youbot_application.application_util import YoubotEnvironment

    signal.signal(signal.SIGINT, signal_handler)

    # y = YoubotEnvironment(0, 5, 0, 5)
    import time

    start_pos = (4, 2)
    goal_pos = (10, 9)
    obstacles = [[(1, 1), (2, 1), (2, 4), (1, 4)],
                 [(1.5, 4.5), (2.5, 4.5), (2.5, 6), (1.5, 6)],
                 [(3, 8), (5, 9), (4.5, 9.5), (2.8, 8.2)],
                 [(5, 3), (6, 3), (6, 4), (6, 5)],
                 [(7, 4), (8, 4), (8, 10), (7, 10)],
                 [(10, 5), (12, 5), (12, 7), (10, 7)]]
    # obstacles = y.create_scene(20, 10)
    start_heading = 0
    goal_heading = 0
    start_time = time.time()
    path_with_heading, g = vg_find_path(start_pos, goal_pos, start_heading, goal_heading, obstacles)
    print(time.time()-start_time)

    plot_vg_path(obstacles, path_with_heading, g)

    print("Path:")
    print(path_with_heading)

    rospy.init_node('robot_cmd_vel_publisher')
    # position_to_velocity(path_with_heading)
    
    