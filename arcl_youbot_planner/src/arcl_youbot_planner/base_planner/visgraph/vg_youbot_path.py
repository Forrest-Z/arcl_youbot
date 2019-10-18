import arcl_youbot_planner.base_planner.visgraph as vg
from arcl_youbot_planner.base_planner.visgraph.visible_vertices import edge_distance
import commands
from shapely.geometry import Polygon
from shapely.ops import unary_union
import math

YOUBOT_RADIUS = 0.3  # in meters
TEST = True

def vg_find_path(start_pos, goal_pos, obstacles):
    """
    Assume obstacles are represent by a list of in-order points
    """

    # multi-threads
    cpu_cores = int(commands.getstatusoutput('cat /proc/cpuinfo | grep processor | wc -l')[1])

    # enlarged obstacles based on YOUBOT_RADIUS, join_style=2 flat, join_style=1 round
    dilated_obstacles = [Polygon(obs).buffer(YOUBOT_RADIUS, join_style=2) for obs in obstacles]

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

    print(obstacles)
    print(polygons)

    return path

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

if __name__ == "__main__":
    start_pos = (0, 0)
    goal_pos = (5, 5)
    obstacles = [[(1, 1), (2, 1.5), (1, 2)],
                 [(3, 2), (4, 2), (4, 4), (3, 4)],
                 [(4, 3), (4.5, 1), (5, 3)]]
    path = vg_find_path(start_pos, goal_pos, obstacles)

    start_heading = 0
    goal_heading = math.pi / 2
    path_with_heading = add_orientation(path, start_heading, goal_heading)

    print(path_with_heading)