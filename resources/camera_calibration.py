import numpy as np
import cv2 as cv
import os


def camera_calibration(chessboard, camera, path, show_patterns=False):
    # Termination criteria
    if show_patterns:
        criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # Prepare object points, like (0,0,0), (1,0,0), (2,0,0), ..., (6,5,0)
    objp = np.zeros((chessboard[0] * chessboard[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:chessboard[0], 0:chessboard[1]].T.reshape(-1, 2)

    # Arrays to store object points and image points from all the images.
    objpoints = []  # 3d points in real world space
    imgpoints = []  # 2d points in image plane

    images = os.listdir(path)

    for img_path in images:
        img = cv.imread(f'{path}/{img_path}')
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv.findChessboardCorners(gray, chessboard, None)

        # If found, add object points, image points (after refining them)
        if ret:
            objpoints.append(objp)
            imgpoints.append(corners)

            if show_patterns:
                corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

                # Draw and display the corners
                cv.drawChessboardCorners(img, chessboard, corners2, ret)
                cv.imshow('Image', img)
                cv.waitKey(500)

    if show_patterns:
        cv.destroyAllWindows()

    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, camera, None, None)
    return mtx
