import rospy
import arcl_youbot_application.application_util as app_util
import arcl_youbot_planner.base_planner.base_util as base_util
from geometry_msgs.msg import Pose, Point, Quaternion
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_msgs.msg import UInt8
import os.path
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_about_axis, quaternion_from_matrix
import math
import copy
import pyrealsense2 as rs
import cv2   
import numpy as np
import functools
import time
from queue import PriorityQueue
from std_msgs.msg import Int8MultiArray

UNLOAD_HEIGHT = [0.17, 0.085, 0.01]
LOAD_HEIGHT = [0.2, 0.115, 0.03]
ZONE = [Pose(Point(0.315323382616, -1.01717340946, 0.103996619582), Quaternion(0.500282824039, -0.499866634607, 0.498632192612, -0.50121486187)),
        Pose(Point(-0.139539673924, -1.01989018917, 0.104218177497), Quaternion(0.498746186495, -0.503944218159, 0.49566963315, -0.5016015172)),
        Pose(Point(-0.593142211437, -1.02366733551, 0.106185980141), Quaternion(0.504684686661, -0.499624967575, 0.498205423355, -0.497453123331)),
        Pose(Point(-1.04596865177, -1.02525186539, 0.108994059265), Quaternion(0.504900157452, -0.500990092754, 0.496651381254, -0.497415482998))]
WIDTH = 640
HEIGHT = 480
NUM_ZONE = len(ZONE)
TOP = len(LOAD_HEIGHT)
EMPTY = 0
RED = 1
GREEN = 2
BLUE = 3
COLOR = [RED, GREEN, BLUE]      # len(COLOR) should be equal to NUM_ZONE
CV_MAX_DISTANCE = 2             # removing the background of objects more than
CV_MIN_AREA = 300               # valid object with area
LIFT_COST_SCALE = 1             # lift cost
MOVE_COST_SCALE = 1.5           # move cost scale
HEURISTIC_SCALE = 1             # heuristic scale, 5 for large space
offset = 0.34
offset_last_step = offset - 0.1


# =========================== Plan ===========================
class Node:
    """ Assume the environment has four places, we want to reallocate boxes so boxes with same color end up in the same place

    Attributes: 
        state: a numpy array PLACES*TOP of the color of a position. Start from left to right, top to bottom.
                x0   x3   x6   x9        
                x1   x4   x7   x10    ==>    x0 x1 x2 ... x11
                x2   x5   x8   x11
        base_position: position of robot, it should be in [0, PLACES), from left to right
        fork_position: position of fork, it should be in [0, TOP), from top to bottom
    """

    def __init__(self, state, base_position_prev=None, fork_position_prev=None, base_position_after=None, fork_position_after=None, cost=0, priority=0):
        self.state = state
        # position prev and after lifted down fork
        self.base_position_prev = base_position_prev
        self.fork_position_prev = fork_position_prev
        self.base_position_after = base_position_after
        self.fork_position_after =  fork_position_after
        self.cost = cost
        self.priority = priority
        
    def neighbors(self):
        neighbors = []
        
        for pu in range(NUM_ZONE):
            for tu in range(TOP):
                if self.state[tu + pu * TOP] != EMPTY:
                    num_lifted_boxes = np.sum(self.state[pu * TOP:tu + pu * TOP + 1] != EMPTY)

                    for pd in range(NUM_ZONE):
                        if pd != pu:
                            num_boxes = np.sum(self.state[pd * TOP:(pd + 1) * TOP] != EMPTY)
                            if num_boxes == 0:
                                neighbors.append(self.from_node_to_next(pu, tu, pd, TOP - 1, num_lifted_boxes))
                            elif num_lifted_boxes + num_boxes <= TOP:
                                for td in range(TOP):
                                    if self.state[td + pd * TOP] != EMPTY:
                                        neighbors.append(self.from_node_to_next(pu, tu, pd, td - 1, num_lifted_boxes))
                                        break

        return neighbors
                                
    def from_node_to_next(self, pu, tu, pd, td, num_lifted_boxes):
        state = np.copy(self.state)

        from_part_start = tu + pu * TOP + 1 - num_lifted_boxes
        from_part_end = tu + pu * TOP + 1
        to_part_start = td + pd * TOP + 1 - num_lifted_boxes
        to_part_end = td + pd * TOP + 1

        state[to_part_start:to_part_end] = state[from_part_start:from_part_end]
        state[from_part_start:from_part_end] = EMPTY

        lift_cost = abs(self.fork_position_after - from_part_end + 1) + abs(to_part_end - from_part_end)
        move_cost = abs(self.base_position_after - pu) + abs(pu - pd)
        # cost = lift_cost * LIFT_COST_SCALE + move_cost * MOVE_COST_SCALE
        cost = 1

        return Node(state, pu, tu, pd, td, cost)

    def __hash__(self):
        return hash((self.state.tobytes()))

    def __eq__(self, other):
        return (self.state == other.state).all()

    def __lt__(self, other):
        return self.priority < other.priority

    def __str__(self):
        return (str(self.state.reshape(TOP, NUM_ZONE, order='F')) + ', base position prev: ' + str(self.base_position_prev) + ', fork position prev: ' + str(self.fork_position_prev) + 
                                                                  ', base position after: ' + str(self.base_position_after) + ', fork position after: ' + str(self.fork_position_after))

def reconstruct_path(came_from, start, goal):
    current = goal
    path = []

    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start) # optional
    path.reverse() # optional

    return path

def heuristic(a, b):
    # it is not optimal
    return np.sum(a != b) * HEURISTIC_SCALE

def a_star_search(start, goal):
    frontier = PriorityQueue()
    frontier.put(start)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    
    i = 0
    while not frontier.empty():
        current = frontier.get()
        
        if current == goal:
            goal.base_position_prev = current.base_position_prev
            goal.fork_position_prev = current.fork_position_prev
            goal.base_position_after = current.base_position_after
            goal.fork_position_after = current.fork_position_after
            print("Found Goal! Cost is " + str(cost_so_far[current]))
            break
        
        for next_node in current.neighbors():
            new_cost = cost_so_far[current] + next_node.cost
            if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                cost_so_far[next_node] = new_cost
                # next_node.priority = new_cost + heuristic(goal.state, next_node.state)
                next_node.priority = new_cost
                frontier.put(next_node)
                came_from[next_node] = current
        
        if i % 5000 == 0:
            print(current)
            print(cost_so_far[current])
        i += 1
    
    return came_from, cost_so_far
# =========================== Plan ===========================


# =========================== Control ===========================
def step(base_load, fork_load, base_unload, fork_unload, prev_base_load):
    # load
    # object
    target_base_pose = copy.deepcopy(ZONE[base_load])
    # if base_load < prev_base_load:
    #     target_base_pose.position.x += 0.002
    # if base_load > prev_base_load:
    #     target_base_pose.position.x -= 0.002
    q = (target_base_pose.orientation.x, target_base_pose.orientation.y, target_base_pose.orientation.z, target_base_pose.orientation.w)
    obj_roll, obj_pitch, obj_yaw = euler_from_quaternion(q) 
    # base
    base_data = rospy.wait_for_message('/vrpn_client_node/' + "youbot_2" + '/pose', PoseStamped)
    current_base_pose = base_data.pose
    q = (base_data.pose.orientation.x, base_data.pose.orientation.y, base_data.pose.orientation.z, base_data.pose.orientation.w)
    base_roll, base_pitch, base_yaw = euler_from_quaternion(q) 
    # adjust pose
    diff_yaw = base_yaw - obj_yaw
    if abs(diff_yaw) > math.pi:
        if obj_yaw > 0:
            obj_yaw -= math.pi
        else:
            obj_yaw += math.pi
    target_base_pose.position.x -= offset * math.cos(obj_yaw)
    target_base_pose.position.y -= offset * math.sin(obj_yaw)
    q = quaternion_from_euler(obj_roll, obj_pitch, obj_yaw)
    target_base_pose.orientation.x = q[0]
    target_base_pose.orientation.y = q[1]
    target_base_pose.orientation.z = q[2]
    target_base_pose.orientation.w = q[3]
    # before move in
    last_step = copy.deepcopy(target_base_pose)
    last_step.position.x -= offset_last_step * math.cos(obj_yaw)
    last_step.position.y -= offset_last_step * math.sin(obj_yaw)
    # start loading
    env.move_to_target("youbot_2", last_step)
    env.set_forklift_position('youbot_2', UNLOAD_HEIGHT[fork_load])
    env.move_to_target("youbot_2", target_base_pose)
    env.set_forklift_position('youbot_2', LOAD_HEIGHT[fork_load])
    env.move_to_target("youbot_2", last_step)

    # unload
    # object
    target_base_pose = copy.deepcopy(ZONE[base_unload])
    # if base_unload < base_load:
    #     target_base_pose.position.x += 0.002
    # if base_unload > base_load:
    #     target_base_pose.position.x -= 0.002
    q = (target_base_pose.orientation.x, target_base_pose.orientation.y, target_base_pose.orientation.z, target_base_pose.orientation.w)
    obj_roll, obj_pitch, obj_yaw = euler_from_quaternion(q) 
    # base
    base_data = rospy.wait_for_message('/vrpn_client_node/' + "youbot_2" + '/pose', PoseStamped)
    current_base_pose = base_data.pose
    q = (base_data.pose.orientation.x, base_data.pose.orientation.y, base_data.pose.orientation.z, base_data.pose.orientation.w)
    base_roll, base_pitch, base_yaw = euler_from_quaternion(q) 
    # adjust pose
    diff_yaw = base_yaw - obj_yaw
    if abs(diff_yaw) > math.pi:
        if obj_yaw > 0:
            obj_yaw -= math.pi
        else:
            obj_yaw += math.pi
    target_base_pose.position.x -= offset * math.cos(obj_yaw)
    target_base_pose.position.y -= offset * math.sin(obj_yaw)
    q = quaternion_from_euler(obj_roll, obj_pitch, obj_yaw)
    target_base_pose.orientation.x = q[0]
    target_base_pose.orientation.y = q[1]
    target_base_pose.orientation.z = q[2]
    target_base_pose.orientation.w = q[3]
    # before move in
    last_step = copy.deepcopy(target_base_pose)
    last_step.position.x -= offset_last_step * math.cos(obj_yaw)
    last_step.position.y -= offset_last_step * math.sin(obj_yaw)
    # start unloading
    env.move_to_target("youbot_2", last_step)
    env.set_forklift_position('youbot_2', LOAD_HEIGHT[fork_unload])
    env.move_to_target("youbot_2", target_base_pose)
    env.set_forklift_position('youbot_2', UNLOAD_HEIGHT[fork_unload])
    env.move_to_target("youbot_2", last_step)
# =========================== Control ===========================


if __name__ == "__main__":
    rospy.init_node("forklift_demo")
    env = app_util.YoubotEnvironment(-1.5, 1.5, -3.0, 1.0, "youbot_2", 1)    

    rest_base_pose = Pose()
    rest_base_pose.position.x = 0
    rest_base_pose.position.y = 0
    rest_base_pose.position.z = 0.1
    rest_base_pose.orientation.x = 0
    rest_base_pose.orientation.y = 0
    rest_base_pose.orientation.z = -0.7071068
    rest_base_pose.orientation.w = 0.7071068

    target_base_pose = Pose()
    target_base_pose.position.x = 0
    target_base_pose.position.y = -0.5
    target_base_pose.position.z = 0.1
    target_base_pose.orientation.x = 0
    target_base_pose.orientation.y = 0
    target_base_pose.orientation.z = -0.7071068
    target_base_pose.orientation.w = 0.7071068

    # Get start infomation
    # image = rospy.wait_for_message('/image', Int8MultiArray)
    # start = list(image.data)
    start = [3, 2, 1, 3, 3, 2, 2, 1, 1]
    start.extend([EMPTY] * TOP)
    start = np.array(start)
    print('start', start)
    
    # Setup goal
    goal = []
    for color in COLOR:
        num_color = (start == color).sum()
        num_empty = TOP - num_color
        goal.extend([EMPTY] * num_empty)
        goal.extend([color] * num_color)
    goal.extend([EMPTY] * TOP)
    goal = np.array(goal)
    print('goal', goal)
    
    # get the execution path
    start_node = Node(start, 1, 2, 1, 2)
    goal_node = Node(goal)
    came_from, cost_so_far = a_star_search(start_node, goal_node)
    path = reconstruct_path(came_from, start_node, goal_node)
    control = []
    for p in path[1:]:
        control.append([p.base_position_prev, p.fork_position_prev, p.base_position_after, p.fork_position_after])
        print(str(p) + '\n')
    print('step', len(path))
    print('cost', cost_so_far[goal_node])

    # # execution
    prev_base_load = 1
    for count, command in enumerate(control):
        print('===== step =====', count + 1)
        step(command[0], command[1], command[2], command[3], prev_base_load)
        prev_base_load = command[2]
    print("Done!")

    # test
    # env.set_forklift_position('youbot_2', 0.03)
