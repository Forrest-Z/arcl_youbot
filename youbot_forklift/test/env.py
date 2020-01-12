import numpy as np
from queue import PriorityQueue

# number of places we can use
PLACES = 4
# max box in one place
TOP = 3
# possible color
EMPTY = 0
RED = 1
BLUE = 2
GREEN = 3
# lift cost
LIFT_COST_SCALE = 1
# move cost scale
MOVE_COST_SCALE = 1.5
# heuristic scale
HEURISTIC_SCALE = 5


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

    def __init__(self, state, base_position=None, fork_position=None, lift_cost=0, priority=0):
        self.state = state
        # position after lifted down fork
        self.base_position = base_position
        self.fork_position = fork_position
        self.lift_cost = lift_cost
        self.priority = priority
        
    def cost(self, from_node, to_node):
        return self.lift_cost * LIFT_COST_SCALE + abs(from_node.base_position - to_node.base_position) * MOVE_COST_SCALE

    def neighbors(self):
        neighbors = []
        
        for pu in range(PLACES):
            for tu in range(TOP):
                if self.state[tu + pu * TOP] != EMPTY:
                    num_lifted_boxes = np.sum(self.state[pu * TOP:tu + pu * TOP + 1] != EMPTY)

                    for pd in range(PLACES):
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

        lift_cost = abs(self.fork_position - from_part_end + 1) + abs(to_part_end - from_part_end)

        return Node(state, pd, td, lift_cost)

    def __hash__(self):
        return hash((self.state.tobytes()))

    def __eq__(self, other):
        return (self.state == other.state).all()

    def __lt__(self, other):
        return self.priority < other.priority

    def __str__(self):
        return str(self.state.reshape(TOP, PLACES, order='F')) + ', base position: ' + str(self.base_position) + ', fork position: ' + str(self.fork_position)

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
            goal.base_position = current.base_position
            goal.fork_position = current.fork_position
            print("Found Goal! Cost is " + str(cost_so_far[current]))
            break
        
        for next_node in current.neighbors():
            new_cost = cost_so_far[current] + current.cost(current, next_node)
            if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                cost_so_far[next_node] = new_cost
                next_node.priority = new_cost + heuristic(goal.state, next_node.state)
                # next_node.priority = new_cost
                frontier.put(next_node)
                came_from[next_node] = current
        
        if i % 1000 == 0:
            print(current)
            print(cost_so_far[current])
        i += 1
    
    return came_from, cost_so_far

if __name__ == "__main__":
    start = np.array([EMPTY, EMPTY, GREEN,
                      EMPTY, GREEN, RED,
                      EMPTY, EMPTY, RED,
                      EMPTY, EMPTY, EMPTY]) 

    goal = np.array([EMPTY, RED, RED,
                     EMPTY, EMPTY, EMPTY,
                     EMPTY, GREEN, GREEN,
                     EMPTY, EMPTY, EMPTY]) 

    start_node = Node(start, 0, 2)
    goal_node = Node(goal)

    came_from, cost_so_far = a_star_search(start_node, goal_node)
    path = reconstruct_path(came_from, start_node, goal_node)
    for p in path:
        print(str(p) + '\n')
    print(len(path))
    print(cost_so_far[goal_node])