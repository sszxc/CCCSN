# -*- coding:utf-8 -*-
import cv2
import numpy as np

img = cv2.imread("C:\\Users\\73677\\Documents\\CCCSN\\Code\\1.jpg")

rows, cols, _ = img.shape
points1 = np.float32([[56, 65], [368, 52], [28, 387], [389, 390]])
points2 = np.float32([[0, 0], [300, 0], [0, 300], [300, 300]])

matrix = cv2.getPerspectiveTransform(points1, points2)
# 将四个点组成的平面转换成另四个点组成的一个平面

# 注意数据类型float
new_matrix = np.array([[1, 0, 0],
                       [0, 1, 0],
                       [0.001, 0.0001, 1]],
                      dtype=np.float)

new_matrix2 = np.array([[0],
                       [0],
                       [1]],
                      dtype=np.float)

intrinsicMatrix = np.array([[827.678512401081, 0, 649.519595992254],
                            [0, 827.856142111345, 479.829876653072],
                            [0, 0, 1]],
                           dtype=np.float)

extrinsicMatrix = np.array([[1, 0, 0, 0],
                            [0, 0.98, -0.17, 0],
                            [0, 0.17, 0.98, 0],
                            [0, 0, 0, 1]],
                           dtype=np.float)

invIM = np.linalg.inv(intrinsicMatrix)
# output = cv2.warpPerspective(img, invMat, (cols, rows))

dot = np.dot(invIM, new_matrix2)

# result=np.ndarray() # 需要先给定大小

# for i in range(img.shape[0]):
#     for j in range(img.shape[1]):
#         target = np.dot(invIM, np.array([i, j, 1]))
#         result[target] = img[i, j,:]              

output = cv2.warpPerspective(img, new_matrix, (cols, rows))
# 通过warpPerspective函数来进行变换

cv2.imshow('img', img)
cv2.imshow('output', output)
cv2.waitKey()
cv2.destroyAllWindows()
