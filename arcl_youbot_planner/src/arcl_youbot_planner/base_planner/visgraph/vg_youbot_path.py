# TODO remove this file

import arcl_youbot_planner.base_planner.visgraph as vg
from arcl_youbot_planner.base_planner.visgraph.visible_vertices import edge_distance
import commands
from shapely.geometry import Polygon, LinearRing
from shapely.ops import unary_union
import math

YOUBOT_RADIUS = 0.35  # in meters
TEST = False

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


if __name__ == "__main__":
    start_pos = (11, 11)
    goal_pos = (1.6, 2)
    obstacles = [[(1, 1), (2, 1), (2, 4), (1, 4)],
                 [(1.5, 5), (2.5, 5), (2.5, 6), (1.5, 6)],
                 [(3, 8), (5, 9), (4.5, 9.5), (2.8, 8.2)],
                 [(5, 3), (6, 3), (6, 4), (6, 5)],
                 [(7, 4), (8, 4), (8, 10), (7, 10)],
                 [(10, 5), (12, 5), (12, 7), (10, 7)]]
    path, g = vg_find_path(start_pos, goal_pos, obstacles)

    start_heading = 0
    goal_heading = math.pi / 2
    path_with_heading = add_orientation(path, start_heading, goal_heading)

    plot_vg_path(obstacles, path_with_heading, g)

    print("Path:")
    print(path_with_heading)