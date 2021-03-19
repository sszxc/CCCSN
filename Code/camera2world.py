# Author: Xuechao Zhang
# Date: March 17th, 2021
# Description: 相机坐标与世界坐标的转换

import numpy as np
import cv2

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
    
    def init_T(self, euler, dx, dy, dz):
        self.T_cam2world = np.array([
            [1, 0, 0, dx],
            [0, 1, 0, dy],
            [0, 0, 1, dz],
            [0, 0, 0, 1]],
            dtype=float
        )
        self.T_world2cam = np.linalg.inv(self.T_cam2world)

    def create_camview(self, height=600, width=1000, color=(255, 255, 255)):
        '''
        世界坐标系的大小 3000mm×5000m×2500m，西北角为原点
        俯视图相机分辨率定义为 1000*600
        普通节点相机分辨率默认为 1280*800
        '''
        graph = np.zeros((height, width, 3), np.uint8)
        graph[:, :, 0] += color[0]
        graph[:, :, 1] += color[1]
        graph[:,:, 2] += color[2]
        self.img = graph

    def capture(self, point):
        '''
        从相机坐标到像素坐标 “拍照”
        '''
        dot = np.dot(self.IM, point)
        dot /= dot[2]  # 归一化
        pixel = tuple(dot.astype(np.int).T.tolist()[0][0:2])  # 去掉第三维的1 转tuple
        return pixel

# # 1280*800 112122-112340
# fx = 827.678512401081
# fy = 827.856142111345
# cx = 649.519595992254
# cy = 479.829876653072

# # 相机坐标系到像素坐标系的转换矩阵
# IM_cam = np.array([
#     [fx, 0, cx],
#     [0, fy, cy],
#     [0, 0, 1]],
#     dtype=float
# )

# # 虚拟相机 焦距用于匹配分辨率
# IM_worldcam = np.array([
#     [500, 0, 0],
#     [0, 500, 0],
#     [0, 0, 1]],
#     dtype=float
# )

# T_cam2world = np.array([
#     [1, 0, 0, 100],
#     [0, 1, 0, 0],
#     [0, 0, 1, 0],
#     [0, 0, 0, 1]],
#     dtype=float
# )

# T_world2cam = np.linalg.inv(T_cam2world)


# def create_camview(height=600, width=1000, color=(255, 255, 255)):
#     '''
#     世界坐标系的大小 3000mm×5000m×2500m，西北角为原点
#     俯视图相机分辨率定义为 1000*600
#     普通节点相机分辨率默认为 1280*800
#     '''
#     img = np.zeros((height, width, 3), np.uint8)
#     img[:, :, 0] += color[0]
#     img[:, :, 1] += color[1]
#     img[:, :, 2] += color[2]
#     return img

# def world2cam(T, point):
#     '''
#     从世界坐标到相机坐标
#     '''
#     return 0

# def cam2pixel(IM, point):
#     '''
#     从相机坐标到像素坐标 “拍照”
#     IM: 内参矩阵 point: 齐次坐标
#     '''
#     dot = np.dot(IM, point)
#     dot /= dot[2]  # 归一化
#     pixel = tuple(dot.astype(np.int).T.tolist()[0][0:2])  # 去掉第三维的1 转tuple
#     return pixel

if __name__ == "__main__":
    # 创建俯视图相机和普通节点相机
    topview = Cam()
    topview.init_IM(500, 500, 0, 0)
    topview.init_T([0, 0, 0], 0, 0, 0)
    topview.create_camview()

    cam_1 = Cam()
    cam_1.init_IM(827.678512401081, 827.856142111345, 649.519595992254, 479.829876653072)
    cam_1.init_T([0, 0, 0], 0, 0, 0)
    cam_1.create_camview(800, 1280)
    
    # 随机三个空间点
    point_1 = np.array([[0], [0], [2500]], dtype=float)
    point_2 = np.array([[220], [1080], [2500]], dtype=float)
    point_3 = np.array([[5000], [3000], [2500]], dtype=float)

    # 转换到俯视图相机坐标
    pixel_1 = topview.capture(point_1)
    pixel_2 = topview.capture(point_2)
    pixel_3 = topview.capture(point_3)

    cv2.circle(topview.img, pixel_1, 20, (0, 150, 255), -1)
    cv2.circle(topview.img, pixel_2, 20, (150, 255, 0), -1)
    cv2.circle(topview.img, pixel_3, 20, (255, 0, 150), -1)

    # cv2.line(topview, (0, 0), (511, 511), (255, 0, 0), 5)

    cv2.imshow('topview', topview.img)
    # cv2.imshow('camview_1', camview_1)
    cv2.waitKey()