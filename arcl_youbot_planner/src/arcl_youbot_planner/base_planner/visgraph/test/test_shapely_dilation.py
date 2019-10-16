from matplotlib import pyplot
from shapely.geometry import LineString, Polygon, LinearRing
from descartes import PolygonPatch

from shapely.figures import SIZE, BLUE, GRAY, set_limits, plot_line, plot_coords

# polygon = Polygon([(0, 0), (2, 0), (2, 4), (0, 4), (0, 3), (1, 3), (1, 1), (0, 1)])
# line = LineString([(0, 0), (2, 0), (2, 4), (0, 4), (0, 3), (1, 3), (1, 1), (0, 1), (0, 0)])

polygon = Polygon([(3, 2), (4, 2), (4, 4), (3, 4)])
line = LineString([(3, 2), (4, 2), (4, 4), (3, 4), (3, 2)])

fig = pyplot.figure(1, figsize=SIZE, dpi=90)

# 1
ax = fig.add_subplot(121)

plot_line(ax, line)

polygon = Polygon(polygon)

dilated = polygon.buffer(0.3, join_style=2)
plot_coords(ax, dilated.exterior)
print(type(dilated.exterior))
print(dilated.exterior)
print(type(dilated.exterior.coords))
print(dilated.exterior.coords)
print(type(list(dilated.exterior.coords)[0][0]))
patch1 = PolygonPatch(dilated, fc=BLUE, ec=BLUE, alpha=0.5, zorder=2)
ax.add_patch(patch1)

ax.set_title('a) dilation')

set_limits(ax, -1, 7, -1, 7)

# #2
# ax = fig.add_subplot(122)

# patch2a = PolygonPatch(dilated, fc=GRAY, ec=GRAY, alpha=0.5, zorder=1)
# ax.add_patch(patch2a)

# eroded = dilated.buffer(-0.3)

# # GeoJSON-like data works as well

# polygon = eroded.__geo_interface__
# # >>> geo['type']
# # 'Polygon'
# # >>> geo['coordinates'][0][:2]
# # ((0.50502525316941682, 0.78786796564403572), (0.5247963548222736, 0.8096820147509064))
# patch2b = PolygonPatch(polygon, fc=BLUE, ec=BLUE, alpha=0.5, zorder=2)
# ax.add_patch(patch2b)

# ax.set_title('b) erosion, join_style=1')

# set_limits(ax, -1, 5, -1, 5)

pyplot.show()

