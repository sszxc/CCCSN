# Author: Xuechao Zhang
# Date: March 17th, 2021
# Description: 实现虚拟相机模型
#               测试不同姿态下像素坐标与世界坐标的转换

import numpy as np
import cv2
import random
import math
from XboxController.Xbox import *
import copy
import time
from PSO import *
from Camera import save_image

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

    def init_M2topview(self, points_3D, pixel_topview):
        '''
        点对计算相机到全局的图片变换矩阵
        '''
        pixel = [self.capture(point) for point in points_3D]
        self.M = cv2.getPerspectiveTransform(np.float32(pixel_topview), np.float32(pixel))
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

    def cam_frame_project(self, topviewimg, color):
        '''
        相机轮廓投影到俯视图
        '''
        # 节点相机的角点
        height, width = self.img.shape[:2]
        corners = []
        corners.append(np.array([0, 0, 1], dtype=np.float32))
        corners.append(np.array([width, 0, 1], dtype=np.float32))
        corners.append(np.array([0, height, 1], dtype=np.float32))
        corners.append(np.array([width, height, 1], dtype=np.float32))
        # 从相机角点到投影图 画两个相机
        corners_projected = []
        for corner in corners:
            corner_projected = np.dot(self.M_inv, corner.T)
            corner_projected /= corner_projected[2]  # 归一化
            corners_projected.append(tuple(corner_projected.astype(
                np.int).T.tolist()[0:2]))
            cv2.circle(topviewimg, tuple(corner_projected.astype(
                np.int).T.tolist()[0:2]), 15, color, -1)
        cv2.line(
            topviewimg, corners_projected[0], corners_projected[1], color,2)
        cv2.line(
            topviewimg, corners_projected[1], corners_projected[3], color,2)
        cv2.line(
            topviewimg, corners_projected[2], corners_projected[3], color,2)
        cv2.line(
            topviewimg, corners_projected[2], corners_projected[0], color,2)
        # 标记一下相机视角中心
        cv2.circle(topviewimg, self.FocusCenter,
                    10, (0, 255, 255), -1)
        return 0

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
    start_time = time.time()
    score = 0
    score_count = 0
    height = heatmap.shape[0]
    weight = heatmap.shape[1]
    img = np.zeros((height, weight, 3), np.uint8)
    img.fill(255)

    # 叠加一下有效范围
    imgframe = np.zeros((height, weight, 3), np.uint8)
    for camera in cameras:
        imgframe = cv2.add(imgframe, camera.ValidView())

    # range加步长 提高运算速度
    gap_step = 25 #10
    for h in range(0, height, gap_step):
        for w in range(0, weight, gap_step):
            heat = 255 - heatmap[h, w, 0]  # 只看B通道 当作灰度处理
            Q_perspective = 0  # 视角质量=到中心的距离(之后可以用remap衡量)
            Q_resolution = 0  # 分辨率质量=每个像素法向距离 高斯函数？
            Q_pr = 0
            for camera in cameras:
                if camera.ValidView(update=False)[h, w].sum() == 765:# 全 255 属于有效视野
                    dh = h-camera.FocusCenter[1]
                    dw = w-camera.FocusCenter[0]

                    distance = (dh)**2 + (dw)**2
                    Q_perspective = max((150000-distance)/150000, 0)  # 0-1 规格化

                    altitude = abs(
                        np.dot(np.array([dw, dh, 0], dtype=np.float), camera.direction))
                    Q_resolution = max((5000 - altitude)/5000, 0)  # 0-1 规格化
                    Q_resolution *= max((8000-camera.dz)/8000, 0)  # 高度对分辨率的影响

                    Q_pr = max(Q_perspective*Q_resolution, Q_pr)  # 多个相机之间的最高覆盖质量

            if Q_pr != 0:
                # 红色高质量 蓝色低质量
                # img[h, w, 0] = 255-Q_pr*255
                # img[h, w, 2] = Q_pr*255
                cv2.rectangle(img, (w,h),(w+gap_step,h+gap_step),(255-Q_pr*255, 120, Q_pr*255),-1)
                # cv2.circle(img, (w, h), 3, (255-Q_pr*255, 120, Q_pr*255), -1)
                
            score += heat * Q_pr
            score_count += 1

        # print (h)
    end_time = time.time()
    # print("score:", score)
    # print("time cost:", end_time-start_time)
    return score, img

def refresh_cam(heatmapfile,cam_1_T_para,cam_2_T_para, autosolve = False):
    '''
    为了优化求解 简单打包一下 没法外部调用
    '''
    cam_1.init_T(*cam_1_T_para)
    cam_2.init_T(*cam_2_T_para)
    if not autosolve:
        print("cam1T:", cam_1_T_para)
        print("cam2T:", cam_2_T_para)

    # 相机初始化
    topview.init_view()
    cam_1.init_view(800, 1280)
    cam_2.init_view(800, 1280)

    # 标记一下相机视角中心
    cam_1.init_FocusCenter(topview.capture(cam_1.Focus))
    cam_2.init_FocusCenter(topview.capture(cam_2.Focus))

    # 利用前四个点生成变换矩阵
    pixel_topview = [topview.capture(point) for point in points[0:4]]
    cam_1.init_M2topview(points[0:4], pixel_topview)
    cam_2.init_M2topview(points[0:4], pixel_topview)

    # 导入底图 即 heatmap
    img_fullview = cv2.imread(heatmapfile)
    if not autosolve:
        # 更新两个相机
        topview.img = img_fullview
        img_nodecam1 = cv2.warpPerspective(
            img_fullview, cam_1.M, (1280, 800), borderValue=(255, 255, 255))

    if not autosolve:
        # 相机投影到俯视图
        cam_1.cam_frame_project(topview.img,(0,255,0))
        cam_2.cam_frame_project(topview.img,(0,0,255))

    # 计算覆盖质量
    score, score_img = SensingQuality(img_fullview, [cam_1, cam_2])

    if not autosolve:
        # 节点相机的画面 缩放一下 看起来方便点
        cam_1.img = cv2.resize(
            img_nodecam1, (int(0.5*img_nodecam1.shape[1]), int(0.5*img_nodecam1.shape[0])))
    
    return score, score_img

def update_by_controller(T_para, T_para_defult, CONTROLLER_TYPE=1):
    '''
    CONTROLLER_TYPE：不同电脑对手柄的配置不同 0:XPS15 1:DELL7070
    Quit_command：触发退出命令
    手柄：左摇杆水平位移 右摇杆角度 LT&RT高度 A退出 B复位
    键盘：WASD平移 ZX高度 UJIKOL旋转 0复位 Q退出
    '''
    Quit_command = 0
    if 'joystick' not in globals():
        # 初始化手柄控制
        global joystick
        joystick = joyinit()
    if joystick:
        if CONTROLLER_TYPE == 0:
            axis, button, hat = joystick_input(joystick)
            T_para[1] += axis[0] * -30
            T_para[2] += axis[1] * -30
            T_para[3] += (axis[4] - axis[5]) * 30
            T_para[0][1] = axis[2] * -50
            T_para[0][0] = axis[3] * 50
            if button[0] == 1:
                Quit_command = 1
            elif button[1] == 1:
                T_para = copy.deepcopy(T_para_defult)
        elif CONTROLLER_TYPE == 1:
            axis, button, hat = joystick_input(joystick)
            T_para[1] += axis[0] * -30
            T_para[2] += axis[1] * -30
            T_para[3] += axis[2] * 50
            T_para[0][1] = axis[3] * -50
            T_para[0][0] = axis[4] * 50
            if button[0] == 1:
                Quit_command = 1
            elif button[1] == 1:
                T_para = copy.deepcopy(T_para_defult)

    else:
        k = cv2.waitKey(0) & 0xFF
        if k == ord('q') or k == ord('Q'):
            Quit_command = 1
        elif k == ord('A'):
            T_para[1] += 250
        elif k == ord('D'):
            T_para[1] -= 250
        elif k == ord('W'):
            T_para[2] += 250
        elif k == ord('S'):
            T_para[2] -= 250
        elif k == ord('Z'):
            T_para[3] += 250
        elif k == ord('X'):
            T_para[3] -= 250
        elif k == ord('U'):
            T_para[0][0] += 10
        elif k == ord('J'):
            T_para[0][0] -= 10
        elif k == ord('I'):
            T_para[0][1] += 10
        elif k == ord('K'):
            T_para[0][1] -= 10
        elif k == ord('O'):
            T_para[0][2] += 10
        elif k == ord('L'):
            T_para[0][2] -= 10
        elif k == ord('0'):
            T_para = T_para_defult

    return T_para, Quit_command


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

    while 0:
        score, score_img = refresh_cam('./img/heat0.jpg', cam_1_T_para, cam_2_T_para, autosolve=False)

        cv2.imshow('topview', topview.img)
        cv2.imshow('camview_1', cam_1.img)
        cv2.imshow('score', score_img)

        # 用键盘/手柄更新云台参数
        cam_1_T_para , Quit_command = update_by_controller(
            cam_1_T_para, cam_1_T_para_defult, CONTROLLER_TYPE=1)
        if Quit_command:
            break

    def score_function(cam_12_T_para):
        # 把不需要优化的补上
        score, _ = refresh_cam(
            './img/heat0.jpg', [cam_12_T_para.tolist()[0:3], *cam_1_T_para_defult[1:4]], [cam_12_T_para.tolist()[3:6], *cam_2_T_para_defult[1:4]], autosolve=True)
        return score

    # PSO 自动求解
    bundary_lower = [-60, -60, -90, -60, -60, -90]
    bundary_upper = [60, 60, 90, 60, 60, 90]
    pso = PSO(cam_1_T_para[0]+cam_2_T_para[0], bundary_upper,
                bundary_lower, score_function)
    # pso.evolve()
    # 为了中间画图 在这边拆开了
    history = []
    plt.figure().canvas.set_window_title('Evolve')  # 窗口名
    plt.ion()

    for step in range(200):
        pso.update()
        print(str(step)+' best score:'+str(pso.g_bestf))
        history.append(pso.g_bestf)
        plt.clf()
        plt.plot(range(step+1), history)
        plt.rcParams['font.sans-serif'] = 'SimHei'  # 显示中文不乱码
        plt.xlabel(u"迭代次数")  # X轴标签
        plt.ylabel(u"适应度")  # Y轴标签
        plt.title(u"迭代过程")  # 标题
        _, score_img = refresh_cam(
            './img/heat0.jpg', [pso.g_bestx.tolist()[0:3], *cam_1_T_para_defult[1:4]], [pso.g_bestx.tolist()[3:6], *cam_2_T_para_defult[1:4]], autosolve=False)
        cv2.imshow('topview', topview.img)
        cv2.imshow('camview_1', cam_1.img)
        cv2.imshow('score', score_img)
        save_image(topview.img, rename_by_time=False,
                   path="/img/2cam_heat0/")
        plt.pause(0.0001)
        plt.ioff()

    plt.show()

    cv2.destroyAllWindows()  # 释放并销毁窗口
