# Author: Xuechao Zhang
# Date: Sept 8th, 2020
# Description: 欧拉角到旋转矩阵的转换
#       EulerAngle → RotationMatrix
#       https://blog.csdn.net/ouyangandy/article/details/105965898

import numpy as np
import math

# Calculates Rotation Matrix given euler angles.

def EulerToRM(theta):

    theta = [x / 180.0 * 3.14159265 for x in theta]  # 角度转弧度

    R_x = np.array([[1,                  0,                   0],
                    [0, math.cos(theta[0]), -math.sin(theta[0])],
                    [0,  math.sin(theta[0]), math.cos(theta[0])]])

    R_y = np.array([[math.cos(theta[1]), 0, math.sin(theta[1])],
                    [0,                  1,                  0],
                    [-math.sin(theta[1]), 0, math.cos(theta[1])]])

    R_z = np.array([[math.cos(theta[2]), -math.sin(theta[2]), 0],
                    [math.sin(theta[2]), math.cos(theta[2]),  0],
                    [0,                  0,                   1]])

    R = np.dot(R_z, np.dot(R_y, R_x))

    return R

if __name__ == "__main__":
    # eulerAngles = [13.8972, -175.3867, 29.0579]
    eulerAngles = [10, 0, 0]
    print(EulerToRM(eulerAngles))
    print("h")
