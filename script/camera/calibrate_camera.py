# adapted from https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
import numpy as np
import cv2 as cv
import glob

w = 8
h = 5
square_size = 0.03  # 3cm

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(h,5,0)
objp = np.zeros((h * w, 3), np.float32)
objp[:, :2] = np.mgrid[0:w, 0:h].T.reshape(-1, 2) * square_size
print(objp)
# Arrays to store object points and image points from all the images.
objpoints = []  # 3d point in real world space
imgpoints = []  # 2d points in image plane.
images = glob.glob('./script/camera/data/*.png')
for fname in images:
  img = cv.imread(fname)
  gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
  # Find the chess board corners
  ret, corners = cv.findChessboardCorners(gray, (w, h), None)
  # If found, add object points, image points (after refining them)
  if ret == True:
    objpoints.append(objp)
    corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    imgpoints.append(corners2)
    # Draw and display the corners
    cv.drawChessboardCorners(img, (w, h), corners2, ret)
    cv.imshow('img', img)
    cv.waitKey(500)
cv.destroyAllWindows()

ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

print(ret, mtx, dist, rvecs, tvecs)

img = cv.imread(images[0])
h, w = img.shape[:2]
newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))

# undistort
dst = cv.undistort(img, mtx, dist, None, newcameramtx)
# crop the image
x, y, w, h = roi
dst = dst[y:y + h, x:x + w]
cv.imwrite('calibresult.png', dst)