from os import chdir, system


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
