from matplotlib import pyplot
from shapely.geometry import Point, Polygon, LineString
from shapely.ops import unary_union
from descartes import PolygonPatch

from shapely.figures import SIZE, BLUE, GRAY, set_limits, plot_line, plot_coords

polygon1 = Polygon([(3, 2), (4, 2), (4, 4), (3, 4)]).buffer(0.3, join_style=2)
line1 = LineString([(3, 2), (4, 2), (4, 4), (3, 4), (3, 2)])

polygon2 = Polygon([(4, 3), (4.5, 1), (5, 3)]).buffer(0.3, join_style=2)
line2 = LineString([(4, 3), (4.5, 1), (5, 3), (4, 3)])

polygon3 = Polygon([(1, 1), (1.5, 1.5), (1, 1.5)]).buffer(0.3, join_style=2)
line3 = LineString([(1, 1), (1.5, 1.5), (1, 1.5), (1, 1)])

test_line = LineString([(6.8,1.8), (5,3)])

polygons = [polygon1, polygon2, polygon3]

fig = pyplot.figure(1, figsize=SIZE, dpi=180)

# 1
ax = fig.add_subplot(121)

plot_line(ax, line1)
plot_line(ax, line2)
plot_line(ax, line3)

for ob in polygons:
    p = PolygonPatch(ob, fc=GRAY, ec=GRAY, alpha=0.5, zorder=1)
    ax.add_patch(p)

ax.set_title('a) polygons')

set_limits(ax, -2, 8, -2, 8)

#2
ax = fig.add_subplot(122)

u = unary_union(polygons)

if not isinstance(u, Polygon):
    p = []
    for uu in u:
        plot_coords(ax, uu.exterior)
        p.append(uu.exterior.coords)
else:
    plot_coords(ax, u.exterior)

patch2b = PolygonPatch(u, fc=BLUE, ec=BLUE, alpha=0.5, zorder=1)
ax.add_patch(patch2b)
import math
ax.plot(1, 0, marker=(2, 1, 0-90), markersize=20, linestyle='None')
ax.plot(0, 0, marker=(2, 1, 45-90), markersize=20, linestyle='None')
ax.plot(3, 0, marker=(2, 1, 90-90), markersize=20, linestyle='None')
ax.plot(4, 0, marker=(2, 1, math.degrees(math.atan2(-2, 3))-90), markersize=20, linestyle='None')
ax.plot(10, 2, marker=(2, 1, math.degrees(math.atan2(3.8-2, 6.8-10))-90), markersize=20, linestyle='None')
print(math.atan2(3.8-2, 6.8-10))
ax.set_title('b) union')

intersection = test_line.intersection(u)
print('----')
print(test_line)
print(u)
print(intersection)
print('----')


plot_line(ax, test_line, linewidth=0.5)

set_limits(ax, -2, 11, -2, 8)

pyplot.show()
