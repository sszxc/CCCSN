# Author: Xuechao Zhang
# Date: March 17th, 2021
# Description: 相机坐标与世界坐标的转换

import numpy as np
import cv2
import random
import math


class Cam:
    def __init__(self):
        print("camera set!")

    def init_IM(self, fx, fy, cx, cy):
        self.IM = np.array([
            [fx, 0, cx],
            [0, fy, cy],
            [0, 0, 1]],
            dtype=float
        )

    def EulerToRM(self, theta):

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

    def init_T(self, euler, dx, dy, dz):
        '''
        相机与世界之间的转换矩阵
        '''
        RM = self.EulerToRM(euler)
        T_43 = np.r_[RM, [[0, 0, 0]]]
        # self.T_cam2world = np.c_[T_43, [dx, dy, dz, 1]]
        T_rotate = np.c_[T_43, [0, 0, 0, 1]]
        T_trans = np.array([
            [1, 0, 0, dx],
            [0, 1, 0, dy],
            [0, 0, 1, dz],
            [0, 0, 0, 1]],
            dtype=float
        )
        self.T_cam2world = np.dot(T_rotate, T_trans)  # 相乘顺序很关键
        self.T_world2cam = np.linalg.inv(self.T_cam2world)

    def init_view(self, height=600, width=1000, color=(255, 255, 255)):
        '''
        世界坐标系的大小 3000mm×5000m×2500m，西北角为原点
        俯视图相机分辨率定义为 1000*600
        普通节点相机分辨率默认为 1280*800
        '''
        graph = np.zeros((height, width, 3), np.uint8)
        graph[:, :, 0] += color[0]
        graph[:, :, 1] += color[1]
        graph[:, :, 2] += color[2]
        self.img = graph

    def world2cam(self, point):
        '''
        从世界坐标到相机坐标
        补上第四维的齐次 1；矩阵乘法；去掉齐次 1
        '''
        point_in_cam = np.dot(self.T_cam2world, np.r_[point, [[1]]])
        return np.delete(point_in_cam, 3, 0)

    def capture(self, point):
        '''
        从世界坐标到像素坐标 → “拍照”
        '''
        point_in_cam = self.world2cam(point)
        dot = np.dot(self.IM, point_in_cam)
        dot /= dot[2]  # 归一化
        pixel = tuple(dot.astype(np.int).T.tolist()[0][0:2])  # 去掉第三维的1 转tuple
        return pixel


def randomColor():
    return (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))


if __name__ == "__main__":
    # 创建理想俯视图相机
    # 位于场地中间 高2.5m处
    # 内参用于匹配图片和真实世界
    topview = Cam()
    topview.init_IM(500, 500, -500, -300)
    topview.init_T([0, 0, 0], 2500, 1500, 0)

    # 创建一个普通节点相机
    # 内参来自 1280*800 112122-112340
    cam_1 = Cam()
    cam_1.init_IM(827.678512401081, 827.856142111345,
                  649.519595992254, 479.829876653072)
    cam_1_T_para = [[0, 0, 0], 0, 0, 2500]
    cam_1.init_T(*cam_1_T_para)

    # 随机空间点
    points = [0] * 8
    pixels = [0] * 8
    points[0] = np.array([[0], [0], [2500]], dtype=float)
    points[1] = np.array([[0], [3000], [2500]], dtype=float)
    points[2] = np.array([[5000], [3000], [2500]], dtype=float)
    points[3] = np.array([[5000], [0], [2500]], dtype=float)
    points[4] = np.array([[2500], [0], [2500]], dtype=float)
    points[5] = np.array([[0], [1500], [2500]], dtype=float)
    points[6] = np.array([[2500], [3000], [2500]], dtype=float)
    points[7] = np.array([[5000], [1500], [2500]], dtype=float)
    # points[6] = np.array([[1980], [900], [2500]], dtype=float)
    # points[7] = np.array([[2400], [1800], [2500]], dtype=float)

    while 1:
        print(cam_1_T_para)
        cam_1.init_T(*cam_1_T_para)

        # 分别用两个相机拍照
        topview.init_view()
        cam_1.init_view(800, 1280)

        pixels = [topview.capture(point) for point in points]

        for point in points:
            color = randomColor()
            pixel_0 = topview.capture(point)
            cv2.circle(topview.img, pixel_0, 20, color, -1)
            pixel_1 = cam_1.capture(point)
            cv2.circle(cam_1.img, pixel_1, 20, color, -1)

        cv2.imshow('topview', topview.img)
        cv2.imshow('camview_1', cam_1.img)

        # WASD平移 ZX高度 UJIKOL旋转 0复位
        k = cv2.waitKey(0) & 0xFF
        if k == ord('A'):
            cam_1_T_para[1] += 1000
        elif k == ord('D'):
            cam_1_T_para[1] -= 1000
        elif k == ord('W'):
            cam_1_T_para[2] += 1000
        elif k == ord('S'):
            cam_1_T_para[2] -= 1000
        elif k == ord('Z'):
            cam_1_T_para[3] += 1000
        elif k == ord('X'):
            cam_1_T_para[3] -= 1000
        elif k == ord('U'):
            cam_1_T_para[0][0] += 10
        elif k == ord('J'):
            cam_1_T_para[0][0] -= 10
        elif k == ord('I'):
            cam_1_T_para[0][1] += 10
        elif k == ord('K'):
            cam_1_T_para[0][1] -= 10
        elif k == ord('O'):
            cam_1_T_para[0][2] += 10
        elif k == ord('L'):
            cam_1_T_para[0][2] -= 10
        elif k == ord('0'):
            cam_1_T_para = [[0, 0, 0], 0, 0, 10000]
        elif k == ord('q') or k == ord('Q'):  # 按下q键，程序退出
            break

    cv2.destroyAllWindows()  # 释放并销毁窗口
