from matplotlib import pyplot
from numpy import vstack
from open3d.cpu.pybind.io import read_point_cloud
from open3d.cpu.pybind.visualization import draw_geometries

from pandas import DataFrame
from pyntcloud import PyntCloud


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
