import os
import time
import numpy as np
import cv2

disCoeffs = np.zeros([4, 1]) * 1.0
print(time.strftime('%H%M%S'))

points1 = np.float32([[56, 65], [368, 52], [28, 387], [389, 390]])
points2 = np.float32([[0, 0], [300, 0], [0, 300], [300, 300]])

matrix = cv2.getPerspectiveTransform(points1, points2)

test = [1,2,3]