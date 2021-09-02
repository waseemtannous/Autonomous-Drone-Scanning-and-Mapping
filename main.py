from json import load
from djitellopy import Tello
from time import sleep
from plot import *
from ExitFinding import *
from PointCloudCleaning import *
from threading import Thread

MAX_ANGLE = 360


# move drone up and down for focus
def move(drone):
    drone.move_up(20)
    drone.move_down(20)


# move the drone 360 degrees and run ORBSLAM2 in a separate thread
# returns drone object
def drone360(data):
    drone = Tello()
    drone.connect()
    drone.speed = int(data["speed"])

    rotationAngle = int(data["rotationAngle"])
    height = int(data["height"])
    sleepTime = int(data["sleep"])

    drone.streamoff()
    drone.streamon()

    ORBSLAM2_THREAD = Thread(target=runOrbSlam2)
    ORBSLAM2_THREAD.start()

    drone.takeoff()

    drone.move_up(int(height - drone.get_height()))

    angle = 0

    sleep(sleepTime)

    print('Starting ORB_SLAM2 ...')

    while angle <= (MAX_ANGLE + rotationAngle):
        drone.rotate_clockwise(rotationAngle)
        move(drone)
        angle += rotationAngle
        sleep(sleepTime)

    drone.streamoff()

    drone.end()

    ORBSLAM2_THREAD.join()
    return drone


# load the config.json file
def loadConfig():
    with open('config.json') as f:
        return load(f)


if __name__ == '__main__':
    data = loadConfig()
    while True:
        drone = drone360(data)
        x, y, z = readCSV('/tmp/pointData.csv')
        pcd = makeCloud(x, y, z)
        inlierPCD, outlierPCD = removeStatisticalOutlier(pcd, voxel_size=float(data["voxel_size"]),
                                                         nb_neighbors=int(data["nb_neighbors"]),
                                                         std_ratio=float(data["std_ratio"]))
        inX, inY, inZ = pcdToArrays(inlierPCD)
        box = getAverageRectangle(inX, inZ)

        plot2DWithBox(inX, inZ, box)
        xOut, yOut = pointsOutOfBox(inX, inZ, box)
        clusters = hierarchicalClustering(xOut, yOut, float(data["thresh"]))
        clustersCenters = getClustersCenters(clusters)

        # break if there are no exits in the room
        if len(clustersCenters) == 0:
            break
        moveToExit(drone, clustersCenters)
        drone.end()
