import cv2
from numpy import zeros, mgrid, float32
from glob import glob
from djitellopy import Tello
from time import sleep


def calibrate():
    # Defining the dimensions of checkerboard
    CHECKERBOARD = (9, 6)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # Creating vector to store vectors of 3D points for each checkerboard image
    objpoints = []
    # Creating vector to store vectors of 2D points for each checkerboard image
    imgpoints = []

    # Defining the world coordinates for 3D points
    objp = zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), float32)
    objp[0, :, :2] = mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    prev_img_shape = None

    # Extracting path of individual image stored in a given directory
    images = glob('images/*.jpg')
    gray = None
    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # Find the chess board corners
        # If desired number of corners are found in the image then ret = true
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD,
                                                 cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)

        """
        If desired number of corner are detected,
        we refine the pixel coordinates and display 
        them on the images of checker board
        """
        if ret == True:
            objpoints.append(objp)
            # refining pixel coordinates for given 2d points.
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

            imgpoints.append(corners2)

            # Draw and display the corners
            img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)

        # cv2.imshow('img', img)
        # cv2.waitKey(1)

        h, w = img.shape[:2]
    cv2.destroyAllWindows()

    """
    Performing camera calibration by 
    passing the value of known 3D points (objpoints)
    and corresponding pixel coordinates of the 
    detected corners (imgpoints)
    """
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    print('Camera.fx: ', mtx[0][0])
    print('Camera.fy: ', mtx[1][1])
    print('Camera.cx: ', mtx[0][2])
    print('Camera.cy: ', mtx[1][2])
    print()
    print('Camera.k1: ', dist[0][0])
    print('Camera.k2: ', dist[0][1])
    print('Camera.p1: ', dist[0][2])
    print('Camera.p2: ', dist[0][3])
    print('Camera.k3: ', dist[0][4])


def getImagesFromDrone():
    # connect to drone
    drone = Tello()
    drone.connect()
    print('Battery = ', drone.get_battery(), '%')

    # start video stream
    drone.streamoff()
    drone.streamon()

    for i in range(15):
        sleep(1.5)
        img = drone.get_frame_read().frame
        cv2.imshow('img', img)
        cv2.waitKey(0)
        cv2.imwrite('images/image' + str(i) + '.jpg', img)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    getImagesFromDrone()
    print('calibrating ...')
    calibrate()
