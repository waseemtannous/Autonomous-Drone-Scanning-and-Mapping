from os import chdir, system
from sklearn.cluster import KMeans
from numpy import array


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
def KMeansAlgo(x, y):
    points = []
    for i in range(len(x)):
        points.append([x[i], y[i]])
    X = array(points)
    # X = np.array([[1, 2], [1, 4], [1, 0], [10, 2], [10, 4], [10, 0]])
    kmeans = KMeans(n_clusters=4, random_state=0).fit(X)
    # print(kmeans.labels_)
    # kmeans.predict([[0, 0], [12, 3]])
    var = kmeans.cluster_centers_
    print(kmeans.cluster_centers_)
