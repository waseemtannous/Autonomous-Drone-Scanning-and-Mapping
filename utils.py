from os import chdir, system
from sklearn.cluster import KMeans
from numpy import array, asarray
from math import sqrt
from numpy import vstack
from open3d.cpu.pybind.io import read_point_cloud
from pandas import DataFrame
from pyntcloud import PyntCloud


# reads csv file and returns x, y, z arrays
def readCSV(fileName: str):
    file = open(fileName, 'r')
    x, y, z = [], [], []
    for line in file:
        line = line.strip('\n')
        arr = line.split(',')
        x.append(float(arr[0]))
        y.append(float(arr[1]))
        z.append(float(arr[2]))
    file.close()
    return x, y, z


# run ORB_SLAM2
def runOrbSlam2():
    chdir('/home/waseem/ORB_SLAM2')
    system('./Examples/Monocular/mono_tum 2 Vocabulary/ORBvoc.txt Examples/Monocular/DRONE_PARAMS.yaml')


# K means algorithm to find clusters centers
def KMeansAlgo(x, y, numberOfClusters):
    points = []
    for i in range(len(x)):
        points.append([x[i], y[i]])
    X = array(points)
    kmeans = KMeans(n_clusters=numberOfClusters, random_state=0).fit(X)

    # cast to array
    points = list(kmeans.cluster_centers_)
    centers = []
    for point in points:
        centers.append(list(point))

    return centers


# returns distance between 2 2D-points
def distanceBetween2Points(point1, point2):
    deltaX = point1[0] - point2[0]
    deltaY = point1[1] - point2[1]
    return sqrt((deltaX * deltaX) + (deltaY * deltaY))


# returns the x, y, z of all points the the point cloud
def pcdToArrays(pcd):
    pointsArray = list(asarray(pcd.points))
    x, y, z = [], [], []
    for point in pointsArray:
        x.append(point[0])
        y.append(point[1])
        z.append(point[2])
    return x, y, z


# make point cloud from xyz coordinates
def makeCloud(x, y, z):
    points = vstack((x, y, z)).transpose()
    cloud = PyntCloud(DataFrame(data=points, columns=["x", "y", "z"]))

    cloud.to_file("PointData/output.ply")

    cloud = read_point_cloud("PointData/output.ply")  # Read the point cloud
    return cloud
