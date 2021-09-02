from os import chdir, system
from sklearn.cluster import KMeans
from numpy import array, asarray, max, vstack
from math import sqrt, tan, degrees
from open3d.cpu.pybind.io import read_point_cloud
from pandas import DataFrame
from pyntcloud import PyntCloud
import scipy.cluster.hierarchy as hcluster


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
# algorithm is not used
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


# returns coordinates of all the points outside the box
def pointsOutOfBox(x, y, box):
    bottomLeft = box[0]
    topRight = box[2]

    pointsX = []
    pointsY = []

    for i in range(len(x)):
        if bottomLeft[0] <= x[i] <= topRight[0] and bottomLeft[1] <= y[i] <= topRight[1]:
            continue
        pointsX.append(x[i])
        pointsY.append(y[i])

    return pointsX, pointsY


# https://stackoverflow.com/questions/10136470/unsupervised-clustering-with-unknown-number-of-clusters
# https://docs.scipy.org/doc/scipy/reference/generated/scipy.cluster.hierarchy.fclusterdata.html#scipy.cluster.hierarchy.fclusterdata
"""
https://en.wikipedia.org/wiki/Hierarchical_clustering:
hierarchical clustering (also called hierarchical cluster analysis or HCA)
is a method of cluster analysis which seeks to build a hierarchy of clusters.
"""
def hierarchicalClustering(x, y, thresh=1.5):
    points = []
    for i in range(len(x)):
        points.append([x[i], y[i]])

    data = array(points)

    # clustering
    clustersIndex = hcluster.fclusterdata(data, thresh, criterion="distance")

    clustersIndex = list(clustersIndex)

    numOfClusters = max(clustersIndex)

    clusters = [[] for _ in range(numOfClusters)]

    for i in range(len(points)):
        index = clustersIndex[i] - 1
        clusters[index].append(points[i])

    return clusters


# get the center of the clusters
def getClustersCenters(clusters):
    centerPoints = []
    for cluster in clusters:
        sumX, sumY = 0, 0
        for point in cluster:
            sumX += point[0]
            sumY += point[1]
        centerPoint = (float(sumX / len(cluster)), float(sumY / len(cluster)))
        centerPoints.append(centerPoint)
    return centerPoints


# move drone to exit
def moveToExit(drone, exits):
    dronePosition = (0, 0)

    # choose the furthest exit
    maxDistance = float('-inf')
    furthestPoint = None
    for exitPoint in exits:
        distance = distanceBetween2Points(dronePosition, exitPoint)
        if distance > maxDistance:
            maxDistance = distance
            furthestPoint = exitPoint

    # calculate angle
    x, y = furthestPoint
    print(furthestPoint)
    angle = 90 - int(degrees(tan(float(abs(y) / abs(x)))))
    if x > 0 > y:
        angle += 90
    elif x < 0 and y < 0:
        angle += 180
    elif x < 0 < y:
        angle += 270

    drone.rotate_clockwise(angle)

    # 1 unit in ORB_SLAM2 is about 160cm in real life
    distance = int(maxDistance * 160)
    print("distance ", distance)
    while distance > 500:
        drone.move_forward(500)
        distance -= 500
    drone.move_forward(distance)
