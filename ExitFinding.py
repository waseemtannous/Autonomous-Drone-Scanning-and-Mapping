from math import log2, floor
from utils import *


# return a rectangle that bounds the room and corresponds to the walls
def getAverageRectangle(x, y):
    # get AVG "center" point
    centerX = float(sum(x) / len(x))
    centerY = float(sum(y) / len(y))
    centerPoint = (centerX, centerY)

    # get all points left of center, right, up and down
    leftX = [(x[i], y[i]) for i in range(len(x)) if x[i] < centerX]
    rightX = [(x[i], y[i]) for i in range(len(x)) if x[i] > centerX]
    upY = [(x[i], y[i]) for i in range(len(y)) if y[i] > centerY]
    downY = [(x[i], y[i]) for i in range(len(y)) if y[i] < centerY]

    # calculate the distance between each point and the center
    leftDistances = [distanceBetween2Points(point, centerPoint) for point in leftX]
    rightDistances = [distanceBetween2Points(point, centerPoint) for point in rightX]
    upDistances = [distanceBetween2Points(point, centerPoint) for point in upY]
    downDistances = [distanceBetween2Points(point, centerPoint) for point in downY]

    # get rectangle coordinates
    xLeft = centerX - (2 * float(sum(leftDistances) / len(leftDistances)))
    xRight = centerX + (2 * float(sum(rightDistances) / len(rightDistances)))
    yUp = centerY + (2 * float(sum(upDistances) / len(upDistances)))
    yDown = centerY - (2 * float(sum(downDistances) / len(downDistances)))

    return (xLeft, yDown), (xRight, yDown), (xRight, yUp), (xLeft, yUp)


##########################################################################################
# AI algorithm for finding a rectangle with # of outside points == # of inside points
# not used in the final algorithm
def findBestBoundingBox(x, y, z):
    numberOfPoints = len(x)
    points = []
    for i in range(numberOfPoints):
        points.append((x[i], y[i], z[i]))

    # sort points by 'y' value
    points.sort(key=lambda tup: tup[1])

    # divide points into log(n) groups
    subarrays = splitIntoSubarrays(points, floor(1 * log2(numberOfPoints)))

    boundingBoxes = []

    # for each group, find bounding box
    for array in subarrays:
        boundingBoxes.append(boundingBox(array))

    bestFitness = getBoxFitness(boundingBoxes[0], points)
    bestBox = boundingBoxes[0]
    for box in boundingBoxes:
        fitness = getBoxFitness(box, points)
        if fitness < bestFitness:
            bestFitness = fitness
            bestBox = box

    print("bestFitness: ", bestFitness)
    return bestBox


# split array into subarrays, each one of size k.
def splitIntoSubarrays(array, k):
    start = 0
    subarrays = []
    while start <= len(array):
        end = start + k
        end = min(end, len(array))
        if start == end:
            break
        subarrays.append(array[start: end])
        start = start + k
    return subarrays


# took from https://stackoverflow.com/questions/46335488/how-to-efficiently-find-the-bounding-box-of-a-collection-of
# -points
def boundingBox(points):
    """returns a list containing the al points of the bounding box,
        starting from bottom left and going anti-clockwise
        """
    min_x, min_y = float('inf'), float('inf')
    max_x, max_y = float('-inf'), float('-inf')
    for x, _, y in points:
        min_x = min(min_x, x)
        min_y = min(min_y, y)
        max_x = max(max_x, x)
        max_y = max(max_y, y)

    return (min_x, min_y), (max_x, min_y), (max_x, max_y), (min_x, max_y)


# gets a box and all points
# returns abs(points outside - points inside)
def getBoxFitness(box, points):
    bottomLeft = box[0]
    topRight = box[2]
    fitness = 0

    for point in points:

        # check if on the top side or bottom
        if bottomLeft[0] <= point[0] <= topRight[0] and ((point[1] == bottomLeft[1]) or (point[1] == topRight[1])):
            continue
        # check if on the left or right
        if bottomLeft[1] <= point[1] <= topRight[1] and ((point[0] == topRight[0]) or (point[0] == bottomLeft[0])):
            continue

        # check if out or inside the box
        if bottomLeft[0] < point[0] < topRight[0] and bottomLeft[1] < point[1] < topRight[1]:
            fitness = fitness + 1
        else:
            fitness = fitness - 1

    return abs(fitness)
