import json

from djitellopy import Tello
from time import sleep
from utils import *
from plot import *
from ExitFinding import *
from PointCloudCleaning import *
from testDensity import *
import numpy

import threading

ROTATE_ANGLE = 30
MAX_ANGLE = 360
val = True


def move(drone):
    drone.move_up(20)
    drone.move_down(20)


def droneWait():
    while val:
        drone.move_up(20)
        drone.move_down(20)


def drone360():
    with open('config.json') as f:
        data = json.load(f)
    drone = Tello()
    drone.connect()
    drone.speed = data["speed"]

    print(drone.get_battery())

    drone.streamoff()
    drone.streamon()

    x = threading.Thread(target=runOrbSlam2)
    x.start()

    drone.takeoff()

    drone.move_up(80)

    angle = 0

    sleep(4)

    print('Starting ...')

    while angle <= (MAX_ANGLE + 15):
        drone.rotate_clockwise(15)
        move(drone)
        angle += 15
        sleep(3)

    drone.streamoff()

    t = threading.Thread(target=droneWait)
    t.start()

    x.join()
    return drone


if __name__ == '__main__':
    # plot2D(x, z)
    # # plotConvexHull(x, y)
    # KMeansAlgo(x, z)
    # x1 = [2.87091167, 0.39248181, 1.67489614, 15.01441968]
    # y1 = [1.00472885, 1.02433301, -0.04094384, 0.6501901]
    # plot2D(x1, y1)
    drone = drone360()
    # x, y, z = readCSV('PointData/pointDataJackobs.csv')
    x, y, z = readCSV('/tmp/pointData.csv')
    pcd = makeCloud(x, y, z)
    inlierPCD, outlierPCD = removeStatisticalOutlier(pcd, voxel_size=0.01, nb_neighbors=30, std_ratio=5.0)
    outX, outY, outZ = pcdToArrays(outlierPCD)
    inX, inY, inZ = pcdToArrays(inlierPCD)
    # plot2D(inX, inZ)
    # showCloud(outX, outY, outZ)
    # showCloud(inX, inY, inZ)
    # plot2D(inX, inZ)
    # box = findBestBoundingBox(x, y, z)

    box = getAverageRectangle(inX, inZ)
    plot2DWithBox(inX, inZ, box)
    x, y = pointsOutOfBox(inX, inZ, box)
    # x = x[:] + x[:] + [(-1 * point) for point in x]
    # y = y[:] + [(-1 * point) for point in y] + y[:]
    clusters = hierarchicalClustering(x, y)
    clustersCenters = getClustersCenters(clusters)
    val = False

    print(clustersCenters)

    moveToExit(drone, clustersCenters)
    drone.end()

    # print(x)
    # print(y)
    # box = getAverageRectangle(x, z)

    # points = []
    # for i in range(len(inX)):
    #     points.append((inX[i], inZ[i]))
    # print(getBoxFitness(box, points))

    # plot2DWithBox(inX, inZ, box)
    #
    # centers = KMeansAlgo(x, z, numberOfClusters=5)
    # print('Center of clusters: ', centers)
    # plot2DWithClustersCenters(x, z, centers)

    # for i in range(len(x)):
    #     x[i] = x[i] * 100
    #
    # for i in range(len(y)):
    #     y[i] = y[i] * 100
    #
    # for i in range(len(z)):
    #     z[i] = z[i] * 100
    #
    # plot2D(x, z)
