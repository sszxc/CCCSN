# Author: Xuechao Zhang
# Date: March 2nd, 2021
# Description: USB 免驱摄像头图像采集
#               MATLAB + OpenCV 畸变矫正
#               pupil_apriltags 实现 AprilTag 识别

import sys
import os
import cv2
import time
import numpy as np
from pupil_apriltags import Detector

to_detect_tag = False


def init_camera(ID):
    cap = cv2.VideoCapture(ID)
    flag = cap.isOpened()

    if flag:
        print("init...")
        # print(cap.set(6, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G')))
        cap.set(3, 1280)
        cap.set(4, 800)
        cap.set(5, 120.0)
        # print(cap.set(cv2.CAP_PROP_FPS, 240.0))
        print("width: "+str(cap.get(3)))
        print("height: " + str(cap.get(4)))
        print("FPS: " + str(cap.get(5)))
        print("Start!")
    else:
        print("Camera not found!")

    return flag, cap

def undistort(frame):
    # 1280*800 112122-112340
    fx = 827.678512401081
    cx = 649.519595992254
    fy = 827.856142111345
    cy = 479.829876653072
    k1, k2, p1, p2, k3 = -0.335814019871572, 0.101431758719313, 0.0, 0.0, 0.0

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
    h, w = frame.shape[:2]
    mapx, mapy = cv2.initUndistortRectifyMap(k, d, None, k, (w, h), 5)
    return cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)

def undistort_fisheye(frame):
    if "map1" not in globals():
        # 1280*800 112122-112340
        DIM = (1280, 800)
        scale = 0.6 # change fov
        K = np.array([[812.3122536784955, 0.0, 646.5710723320936], [
            0.0, 811.5918123346368, 478.3398614399181], [0.0, 0.0, 1.0]])
        D = np.array([[-0.0371896696328494], [-0.03502691542221994],
                    [0.05785462115863493], [-0.036406347514904874]])

        # dim1 is the dimension of input image to un-distort
        dim1 = frame.shape[:2][::-1]
        assert dim1[0]/dim1[1] == DIM[0] / \
            DIM[1], "Image to undistort needs to have same aspect ratio as the ones used in calibration"
        if dim1[0] != DIM[0]:
            frame = cv2.resize(frame, DIM, interpolation=cv2.INTER_AREA)
        Knew = K.copy()
        Knew[(0, 1), (0, 1)] = scale * Knew[(0, 1), (0, 1)]
        global map1, map2
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(
            K, D, np.eye(3), Knew, DIM, cv2.CV_16SC2)

    undistorted_img = cv2.remap(
        frame, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

    return undistorted_img

def at_detect(frame):
    at_detector = Detector(families='tag36h11',
                           nthreads=1,
                           quad_decimate=1.0,
                           quad_sigma=0.0,
                           refine_edges=1,
                           decode_sharpening=0.25,
                           debug=0)

    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags = at_detector.detect(
        frame_gray, estimate_tag_pose=False, camera_params=None, tag_size=None)

    for tag in tags:
        cv2.circle(frame, tuple(tag.corners[0].astype(
            int)), 4, (255, 0, 0), 2)  # left-top
        cv2.circle(frame, tuple(tag.corners[1].astype(
            int)), 4, (255, 0, 0), 2)  # right-top
        cv2.circle(frame, tuple(tag.corners[2].astype(
            int)), 4, (255, 0, 0), 2)  # right-bottom
        cv2.circle(frame, tuple(tag.corners[3].astype(
            int)), 4, (255, 0, 0), 2)  # left-bottom
        cv2.putText(frame, str(tag.tag_id), tuple(tag.center.astype(
            int)), cv2.FONT_HERSHEY_PLAIN, 2.0, (0, 0, 255), 4)

    return frame

def save_image(img, rename_by_time):
    '''
    True: 按照时间命名 False: 按照序号命名
    '''
    filename = sys.path[0] + "/img/"
    if rename_by_time:
        filename += time.strftime('%H%M%S')
    else:
        if 'index' not in globals():
            global index
            index = 1  # 保存图片起始索引
        else:
            index += 1
        filename += str(index)
    filename += ".jpg"
    # cv2.imwrite("./img/" + filename, frame_undistort) # 非中文路径保存图片
    cv2.imencode('.jpg', frame_undistort)[1].tofile(filename)  # 中文路径保存图片
    print("save img successfuly!")
    print(filename)

def cal_fps(filter_length):
    '''
    filter_length 帧率计数平滑
    '''
    if 'timestamp' not in globals():
        global timestamp
        timestamp = [time.time()] * filter_length  # 平滑初始化
        fps = 1
    else:
        timestamp = timestamp[1:filter_length]+[time.time()]
        # if int(time.time()*100) % 3 ==0: # 减慢刷新速度
        fps = round((filter_length-1) /
                    (timestamp[filter_length-1] - timestamp[0]), 2)  # 保留两位
    return fps

if __name__ == "__main__":

    flag, cap = init_camera(0)

    while (flag):
        ret, frame = cap.read()
        # cv2.imshow("Capture_raw", frame)

        # frame_undistort = frame
        frame_undistort = undistort_fisheye(frame)
        # frame_undistort = undistort(frame)

        # frame_undistort = at_detect(frame_undistort)

        cv2.putText(frame_undistort, "FPS:" + str(cal_fps(10)),
                    (0, 25), cv2.FONT_HERSHEY_PLAIN, 2.0, (0, 255, 0), 2)
        cv2.imshow("Capture", frame_undistort)

        k = cv2.waitKey(1) & 0xFF
        if k == ord('s') or k == ord('S'):  # 按下s键，进入下面的保存图片操作
            save_image(frame_undistort, False)
        elif k == ord('q') or k == ord('Q'):  # 按下q键，程序退出
            break

    cap.release()  # 释放摄像头
    cv2.destroyAllWindows()  # 释放并销毁窗口
