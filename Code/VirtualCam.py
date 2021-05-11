# Author: Xuechao Zhang
# Date: March 17th, 2021
# Description: 实现虚拟相机模型
#               测试不同姿态下像素坐标与世界坐标的转换

import numpy as np
import cv2
import random
import math
from Xbox import *
import copy

# CONTROLLER_TYPE = 0 # XPS15
CONTROLLER_TYPE = 1  # DELL7070


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

        # R = np.dot(R_z, np.dot(R_y, R_x)) # 之前一直是这个 加上旋转之后发现乘的顺序不对 可能和轴组顺序有关
        R = np.dot(R_y, np.dot(R_x, R_z))

        return R

    def init_T(self, euler, dx, dy, dz):
        '''
        相机与世界之间的转换矩阵
        '''
        self.dz = dz  # 姑且用来衡量距离对分辨率的影响
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
        # 计算相机聚焦中心 竖直向下的向量经过RM矩阵旋转得到法向
        # 正负号暂时没明白
        self.direction = np.dot(
            np.array([0, 0, 1], dtype=np.float), RM)  # 方向向量
        focus_x = dx - dz/self.direction[2]*self.direction[0]
        focus_y = dy - dz/self.direction[2]*self.direction[1]
        # print(-focus_x,-focus_y)
        self.Focus = np.array([[-focus_x], [-focus_y], [0]], dtype=float)

    def init_M(self, pixel_before, pixel_after):
        '''
        点对计算相机到全局的图片变换矩阵
        '''
        self.M = cv2.getPerspectiveTransform(pixel_before, pixel_after)
        self.M_inv = np.linalg.inv(self.M)

    def init_FocusCenter(self, center):
        '''
        用俯视图拍出来的中心点
        '''
        self.FocusCenter = center

    def init_view(self, height=600, width=1000, color=(255, 255, 255)):
        '''
        新建图像
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

    def ValidView(self, distort=False, update=True):
        '''
        计算节点相机实际覆盖范围
        distort考虑畸变效果 效率较低 功能还存在问题
        update在重复调用的时候可以关掉
        '''
        if update:
            if distort:
                # 1280*800 112122-112340
                fx = 827.678512401081
                cx = 649.519595992254
                fy = 827.856142111345
                cy = 479.829876653072
                k1, k2, p1, p2, k3 = 0.335814019871572, -0.101431758719313, 0.0, 0.0, 0.0

                # 相机坐标系到像素坐标系的转换矩阵
                k = np.array([
                    [fx, 0, cx],
                    [0, fy, cy],
                    [0, 0, 1]
                ])
                # 畸变系数
                d = np.array([
                    k1, k2, p1, p2, k3
                ])
                height, width = self.img.shape[:2]
                mapx, mapy = cv2.initUndistortRectifyMap(
                    k, d, None, k, (width, height), 5)

                self.imgview = np.zeros((600, 1000, 3), np.uint8)
                # 遍历相机图片 畸变映射之后投到俯视图
                # 只要遍历边框就行? 但这样还要填充
                for h in range(0, height, 5):
                    for w in range(0, width, 5):
                        h_distort, w_distort = (mapx[h, w], mapy[h, w])
                        pixel_in_topview = np.dot(
                            self.M, np.array([w_distort, h_distort, 1]).T)
                        pixel_in_topview /= pixel_in_topview[2]  # 归一化
                        pixel = pixel_in_topview.astype(np.int).T.tolist()[0:2]
                        if 0 <= pixel[0] and pixel[0] < 600 and 0 <= pixel[1] and pixel[1] < 1000:
                            self.imgview[tuple(pixel)] = [255, 255, 255]
                        # print(h,w)
            else:
                imgframe_cam = np.zeros(self.img.shape[:2]+(3,), np.uint8)
                imgframe_cam.fill(255)
                self.imgview = cv2.warpPerspective(
                    imgframe_cam, self.M_inv, (1000, 600), borderValue=(0, 0, 0))
        return self.imgview


def randomColor():
    return (0, 0, 0)
    # return (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))


def SensingQuality(heatmap, cameras):
    '''
    heatmap: 用灰度表示事件重要性 255不重要→0重要
    cameras: 用于监控的相机 读取外参、畸变矩阵
    imgframe: 有效像素 和heatmap一样大
    '''
    score = 0
    score_count = 0
    height = heatmap.shape[0]
    weight = heatmap.shape[1]
    img = np.zeros((height, weight, 3), np.uint8)

    # 叠加一下有效范围
    imgframe = np.zeros((height, weight, 3), np.uint8)
    for camera in cameras:
        imgframe = cv2.add(imgframe, camera.ValidView())

    # range加步长 提高运算速度
    gap_step = 10
    for h in range(0, height, gap_step):
        for w in range(0, weight, gap_step):
            heat = 255 - heatmap[h, w, 0]  # 灰度
            Q_perspective = 0  # 视角质量=到中心的距离(之后可以用remap衡量)
            Q_resolution = 0  # 分辨率质量=每个像素法向距离 高斯函数？
            Q_pr = 0
            for camera in cameras:
                # 全 255 属于有效视野
                if camera.ValidView(update=False)[h, w].sum() == 765:
                    dh = h-camera.FocusCenter[1]
                    dw = w-camera.FocusCenter[0]

                    distance = (dh)**2 + (dw)**2
                    Q_perspective = max((150000-distance)/150000, 0)  # 0-1 规格化

                    altitude = abs(
                        np.dot(np.array([dw, dh, 0], dtype=np.float), camera.direction))
                    Q_resolution = max((5000 - altitude)/5000, 0)  # 0-1 规格化
                    Q_resolution *= max((8000-camera.dz)/8000, 0)  # 高度对分辨率的影响

                    Q_pr = max(Q_perspective*Q_resolution,
                               Q_pr)  # 多个相机之间的最高覆盖质量

            if Q_pr != 0:
                # 红色高质量 蓝色低质量
                # img[h, w, 0] = 255-Q_pr*255
                # img[h, w, 2] = Q_pr*255
                cv2.circle(img, (w, h), 3, (255-Q_pr*255, 120, Q_pr*255), -1)

            score += heat * Q_pr
            score_count += 1

        # print (h)
    # 要不要除 还要想一下
    # if score:
    #     score /= score_count
    print("score:", score)
    return score, img


if __name__ == "__main__":
    # 创建理想俯视图相机
    # 位于场地中间 高2.5m处
    # 内参用于匹配图片和真实世界
    topview = Cam()
    topview.init_IM(500, 500, -500, -300)
    topview.init_T([0, 0, 0], 2500, 1500, 2500)

    # 创建一个普通节点相机
    # 内参来自 1280*800 112122-112340
    cam_1 = Cam()
    cam_1.init_IM(827.678512401081, 827.856142111345,
                  649.519595992254, 479.829876653072)
    cam_1_T_para_defult = [[0, 0, 0], -3000, -1500, 2500]
    cam_1_T_para = copy.deepcopy(cam_1_T_para_defult)
    cam_1.init_T(*cam_1_T_para)

    # 创建第二个相机
    # 内参来自 1280*800 112122-112340
    cam_2 = Cam()
    cam_2.init_IM(827.678512401081, 827.856142111345,
                  649.519595992254, 479.829876653072)
    cam_2_T_para_defult = [[0, 0, 0], -2000, -1000, 1500]
    cam_2_T_para = copy.deepcopy(cam_2_T_para_defult)
    cam_2.init_T(*cam_2_T_para)

    # 在地面上随意设置一些空间点
    points = [0] * 8
    points[0] = np.array([[0], [0], [0]], dtype=float)
    points[1] = np.array([[0], [3000], [0]], dtype=float)
    points[2] = np.array([[5000], [3000], [0]], dtype=float)
    points[3] = np.array([[5000], [0], [0]], dtype=float)
    points[4] = np.array([[2500], [0], [0]], dtype=float)
    points[5] = np.array([[0], [1500], [0]], dtype=float)
    points[6] = np.array([[2500], [3000], [0]], dtype=float)
    points[7] = np.array([[5000], [1500], [0]], dtype=float)
    # points[6] = np.array([[1980], [900], [0]], dtype=float)
    # points[7] = np.array([[2400], [1800], [0]], dtype=float)

    # 节点相机的角点
    corners = []
    corners.append(np.array([0, 0, 1], dtype=np.float32))
    corners.append(np.array([1280, 0, 1], dtype=np.float32))
    corners.append(np.array([0, 800, 1], dtype=np.float32))
    corners.append(np.array([1280, 800, 1], dtype=np.float32))

    # 初始化手柄控制
    joystick = joyinit()

    # 下次写成 for cam in cameras:
    while 1:
        print("cam1T:", cam_1_T_para)
        cam_1.init_T(*cam_1_T_para)
        print("cam2T:", cam_2_T_para)
        cam_2.init_T(*cam_2_T_para)

        # 相机初始化
        topview.init_view()
        cam_1.init_view(800, 1280)
        cam_2.init_view(800, 1280)

        # 标记一下相机视角中心
        cam_1.init_FocusCenter(topview.capture(cam_1.Focus))
        cam_2.init_FocusCenter(topview.capture(cam_2.Focus))

        pixel_0 = []
        pixel_1 = []
        pixel_2 = []

        # 地面点转换到相机坐标
        for point in points:
            color = randomColor()
            pixel_0.append(topview.capture(point))
            # cv2.circle(topview.img, pixel_0[-1], 20, color, -1)
            pixel_1.append(cam_1.capture(point))
            # cv2.circle(cam_1.img, pixel_1[-1], 20, color, -1)
            pixel_2.append(cam_2.capture(point))

        # 利用前四个点生成变换矩阵
        pixel_topview = np.float32(pixel_0[0:4])
        pixel_cam1 = np.float32(pixel_1[0:4])
        pixel_cam2 = np.float32(pixel_2[0:4])
        cam_1.init_M(pixel_topview, pixel_cam1)
        cam_2.init_M(pixel_topview, pixel_cam2)

        # 导入底图 即heatmap
        img_fullview = cv2.imread('./img/heat2.jpg')
        # 更新两个相机
        topview.img = img_fullview
        img_nodecam = cv2.warpPerspective(
            img_fullview, cam_1.M, (1280, 800), borderValue=(255, 255, 255))

        # 从相机角点到投影图 画两个相机
        corners_projected = []
        for corner in corners:
            corner_projected = np.dot(cam_1.M_inv, corner.T)
            corner_projected /= corner_projected[2]  # 归一化
            corners_projected.append(tuple(corner_projected.astype(
                np.int).T.tolist()[0:2]))
            cv2.circle(topview.img, tuple(corner_projected.astype(
                np.int).T.tolist()[0:2]), 15, (0, 255, 0), -1)
        cv2.line(
            topview.img, corners_projected[0], corners_projected[1], (0, 255, 0))
        cv2.line(
            topview.img, corners_projected[1], corners_projected[3], (0, 255, 0))
        cv2.line(
            topview.img, corners_projected[2], corners_projected[3], (0, 255, 0))
        cv2.line(
            topview.img, corners_projected[2], corners_projected[0], (0, 255, 0))

        corners_projected = []
        for corner in corners:
            corner_projected = np.dot(cam_2.M_inv, corner.T)
            corner_projected /= corner_projected[2]  # 归一化
            corners_projected.append(tuple(corner_projected.astype(
                np.int).T.tolist()[0:2]))
            cv2.circle(topview.img, tuple(corner_projected.astype(
                np.int).T.tolist()[0:2]), 15, (0, 0, 255), -1)
        cv2.line(
            topview.img, corners_projected[0], corners_projected[1], (0, 0, 255))
        cv2.line(
            topview.img, corners_projected[1], corners_projected[3], (0, 0, 255))
        cv2.line(
            topview.img, corners_projected[2], corners_projected[3], (0, 0, 255))
        cv2.line(
            topview.img, corners_projected[2], corners_projected[0], (0, 0, 255))

        # 标记一下相机视角中心
        cv2.circle(topview.img, cam_1.FocusCenter,
                   10, (0, 255, 255), -1)
        cv2.circle(topview.img, cam_2.FocusCenter,
                   10, (0, 255, 255), -1)

        # 计算覆盖质量
        score, score_img = SensingQuality(
            img_fullview, [cam_1, cam_2])

        # 节点相机的画面 缩放一下 看起来方便点
        cam_1.img = cv2.resize(
            img_nodecam, (int(0.5*img_nodecam.shape[1]), int(0.5*img_nodecam.shape[0])))

        cv2.imshow('topview', topview.img)
        cv2.imshow('camview_1', cam_1.img)
        cv2.imshow('score', score_img)

        if joystick:
            # 左摇杆水平位移 右摇杆角度 LT&RT高度
            # A 退出 B 复位
            if CONTROLLER_TYPE == 0:
                axis, button, hat = joystick_input(joystick)
                cam_1_T_para[1] += axis[0] * -30
                cam_1_T_para[2] += axis[1] * -30
                cam_1_T_para[3] += (axis[4] - axis[5]) * 30
                cam_1_T_para[0][1] = axis[2] * -50
                cam_1_T_para[0][0] = axis[3] * 50
                if button[0] == 1:
                    break
                elif button[1] == 1:
                    cam_1_T_para = copy.deepcopy(cam_1_T_para_defult)
            elif CONTROLLER_TYPE == 1:
                axis, button, hat = joystick_input(joystick)
                cam_1_T_para[1] += axis[0] * -30
                cam_1_T_para[2] += axis[1] * -30
                cam_1_T_para[3] += axis[2] * 50
                cam_1_T_para[0][1] = axis[3] * -50
                cam_1_T_para[0][0] = axis[4] * 50
                if button[0] == 1:
                    break
                elif button[1] == 1:
                    cam_1_T_para = copy.deepcopy(cam_1_T_para_defult)

        else:
            # WASD平移 ZX高度 UJIKOL旋转 0复位
            k = cv2.waitKey(0) & 0xFF
            if k == ord('q') or k == ord('Q'):  # 按下q键，程序退出
                break
            elif k == ord('A'):
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
                cam_1_T_para = cam_1_T_para_defult

    cv2.destroyAllWindows()  # 释放并销毁窗口
