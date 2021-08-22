from matplotlib import pyplot
from numpy import random
from open3d.cpu.pybind.visualization import draw_geometries
from matplotlib.patches import Rectangle

from scipy.spatial import ConvexHull, convex_hull_plot_2d
from utils import *


# scatter 3d points in a 2d image
def plot3D(x, y, z):
    fig = pyplot.figure()
    ax = fig.add_subplot(111, projection='3d')

    # change the viewing angle
    # change viewing angle
    # ax.view_init(20, 270)

    ax.scatter(x, y, z)
    pyplot.show()


# scatter 2d points
# send x, z coordinates to see room from ceiling POV
def plot2D(x, y):
    pyplot.scatter(x, y)
    pyplot.show()


# scatter 2d points with rectangle on the plot
def plot2DWithBox(x, y, box):
    bottomLeft = box[0]
    topRight = box[2]
    pyplot.scatter(x, y)
    rect = Rectangle((bottomLeft[0], bottomLeft[1]), abs(bottomLeft[0] - topRight[0]), abs(bottomLeft[1] - topRight[1]),
                     fill=False,
                     color="purple",
                     linewidth=2)
    pyplot.gca().add_patch(rect)
    pyplot.show()


# scatter all points in 2d and color the clusters centers in different color,
# also colors the furthest center in a third color
def plot2DWithClustersCenters(x, y, centers):
    pyplot.scatter(x, y)

    centersX = []
    centersY = []
    avgX = 0
    avgY = 0
    for coordinates in centers:
        avgX += coordinates[0]
        avgY += coordinates[1]
        centersX.append(coordinates[0])
        centersY.append(coordinates[1])
    pyplot.scatter(centersX, centersY)

    avgX = float(avgX / len(centers))
    avgY = float(avgY / len(centers))

    pyplot.scatter(avgX, avgY)
    print('Center of clusters centers: ', [avgX, avgY])

    center = avgX, avgY
    maxDistance = float('-inf')
    furthestCenter = None
    for point in centers:
        distance = distanceBetween2Points(center, point)
        if distance > maxDistance:
            maxDistance = distance
            furthestCenter = point

    print('furthestPoint: ', furthestCenter)
    pyplot.scatter(furthestCenter[0], furthestCenter[1])

    pyplot.show()


# gets x, y, z coordinates and shows an interactive points cloud
def showCloud(x, y, z):
    cloud = makeCloud(x, y, z)
    draw_geometries([cloud])  # Visualize the point cloud


# todo: remove, not used
def plotConvexHull(x, y):
    points = []
    for i in range(len(x)):
        points.append([x[i], y[i]])
    rng = random.default_rng()
    points = rng.random((30, 2))  # 30 random points in 2-D
    print(points)
    hull = ConvexHull(points)
    pyplot.plot(points[:, 0], points[:, 1], 'o')
    for simplex in hull.simplices:
        pyplot.plot(points[simplex, 0], points[simplex, 1], 'k-')
    pyplot.plot(points[hull.vertices, 0], points[hull.vertices, 1], 'r--', lw=2)
    pyplot.plot(points[hull.vertices[0], 0], points[hull.vertices[0], 1], 'ro')
    pyplot.show()
