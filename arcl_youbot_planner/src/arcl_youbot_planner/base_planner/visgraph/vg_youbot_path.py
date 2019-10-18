import arcl_youbot_planner.base_planner.visgraph as vg
import commands
from shapely.geometry import Polygon
from shapely.ops import unary_union

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

    return path

if __name__ == "__main__":
    start_pos = (0, 0)
    goal_pos = (5, 5)
    obstacles = [[(1, 1), (2, 1.5), (1, 1.5)],
                 [(3, 2), (4, 2), (4, 4), (3, 4)],
                 [(4, 3), (4.5, 1), (5, 3)]]
    path = vg_find_path(start_pos, goal_pos, obstacles)
    print(path)