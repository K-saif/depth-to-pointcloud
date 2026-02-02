import cv2
import numpy as np

def compute_disparity(left_img, right_img):
    gray_l = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)
    gray_r = cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY)

    stereo = cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=128,
        blockSize=5
    )

    disparity = stereo.compute(gray_l, gray_r).astype(np.float32) / 16.0
    disparity[disparity <= 0] = 0.1

    return disparity
