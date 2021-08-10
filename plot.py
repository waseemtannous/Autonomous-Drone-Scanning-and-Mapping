from matplotlib import pyplot
from numpy import vstack, random
from open3d.cpu.pybind.io import read_point_cloud
from open3d.cpu.pybind.visualization import draw_geometries

from pandas import DataFrame
from pyntcloud import PyntCloud
from scipy.spatial import ConvexHull, convex_hull_plot_2d


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


# gets x, y, z coordinates and shows an interactive points cloud
def showCloud(x, y, z):
    points = vstack((x, y, z)).transpose()
    cloud = PyntCloud(DataFrame(data=points, columns=["x", "y", "z"]))

    cloud.to_file("PointData/output.ply")

    cloud = read_point_cloud("PointData/output.ply")  # Read the point cloud
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
