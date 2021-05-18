# Author: Xuechao Zhang
# Date: March 2nd, 2021
# Description: USB 免驱摄像头图像采集
#               MATLAB + OpenCV 畸变矫正
#               pupil_apriltags 实现 AprilTag 识别定位

import sys
import os
import cv2
import time
import numpy as np
from pupil_apriltags import Detector
from Send2DB import *


def init_camera(ID):
    '''
    相机初始化
    '''
    cap = cv2.VideoCapture(ID)
    flag = cap.isOpened()

    if flag:
        print("init...")
        # print(cap.set(6, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G')))
        # cap.set(3, 640)
        # cap.set(4, 400)
        # cap.set(5, 240.0)
        cap.set(3, 1920)
        cap.set(4, 1080)
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
    '''
    畸变矫正
    '''
    if 'mapx' not in globals():
        global mapx, mapy
        # 1280*800 112122-112340
        fx = 827.678512401081
        cx = 649.519595992254
        fy = 827.856142111345
        cy = 479.829876653072
        k1, k2, p1, p2, k3 = -0.335814019871572, 0.101431758719313, 0.0, 0.0, 0.0

        # 170度 1280*800
        # fx = 640.314848396396
        # cx = 587.652366196605
        # fy = 642.765274234341
        # cy = 323.040226037541
        # k1, k2, p1, p2, k3 = -0.316424682686498, 0.0684998923953000, 0.0, 0.0, 0.0

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
    '''
    鱼眼模型矫正 不合适
    '''
    if "map1" not in globals():
        # 1280*800 112122-112340
        DIM = (1280, 800)
        scale = 0.6  # change fov
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
    '''
    AprilTag 检测器
    '''
    if 'at_detector' not in globals():
        global at_detector
        at_detector = Detector(families='tag16h5',
                               nthreads=8,
                               quad_decimate=2.0,
                               quad_sigma=0.0,
                               refine_edges=1,
                               decode_sharpening=0.25,
                               debug=0)
    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags = at_detector.detect(
        frame_gray, estimate_tag_pose=False, camera_params=[827.678512401081, 827.856142111345, 649.519595992254, 479.829876653072], tag_size=0.1)

    tags_fliter = [tag for tag in tags if tag.hamming == 0]  # 用小码的时候增大一点检测阈值
    return tags_fliter


def at_print(frame, tags):
    '''
    四角定位码透视变换 计算01标准化坐标
    '''

    tag_realpos = np.zeros([15, 3])  # tag 真实坐标

    if tags:
        # tags.sort(key=lambda element: element.tag_id, reverse=False) # 检测器自带了排序
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
            # cv2.putText(frame, str(tag.tag_id)+str(tag.pose_t.round(1)), tuple(tag.center.astype(
            #     int)), cv2.FONT_HERSHEY_PLAIN, 2.0, (0, 0, 255), 4)

        # 定位码齐全 画框
        if len(tags) > 3:
            IDs = [tag.tag_id for tag in tags]
            if 0 in IDs and 1 in IDs and 4 in IDs and 5 in IDs:
                srcPoints = [tags[IDs.index(i)].center.astype(int)
                             for i in [0, 1, 4, 5]]
                cv2.line(frame, tuple(srcPoints[0]),
                         tuple(srcPoints[1]), (0, 255, 0), thickness=2)
                cv2.line(frame, tuple(srcPoints[1]),
                         tuple(srcPoints[3]), (0, 255, 0), thickness=2)
                cv2.line(frame, tuple(srcPoints[2]),
                         tuple(srcPoints[0]), (0, 255, 0), thickness=2)
                cv2.line(frame, tuple(srcPoints[3]),
                         tuple(srcPoints[2]), (0, 255, 0), thickness=2)

                # 实际地图3m*2m
                # (150~1050)*(100~700)
                destPoints = np.float32([[150, 700],
                                         [150, 100],
                                         [1050, 700],
                                         [1050, 100]])
                if 'project_H' not in globals():
                    global project_H
                project_H = cv2.getPerspectiveTransform(
                    np.array(srcPoints).astype(np.float32), destPoints)

        # 做透视变换
        if 'project_H' in globals():
            frame = cv2.warpPerspective(
                frame, project_H, (frame.shape[1], frame.shape[0]))

        if len(tags) > 4:
            # 如果有多的 tag 计算相对位置
            for tag in tags[4:]:  # 鉴于前几个都是定位码 直接看后面的也行
                coord_homo = np.array(
                    [tag.center[0], tag.center[1], 1], dtype=np.float32)  # 齐次坐标
                coord_rel = np.dot(project_H, coord_homo)  # 透视变换
                coord_rel /= coord_rel[2]  # 透视变换归一化
                rel_x = round((coord_rel[0] - 150)/900.0*3000)  # 归一化
                rel_y = round((coord_rel[1] - 100)/600.0*2000)
                cv2.putText(frame, str(rel_x)+","+str(rel_y),
                            tuple(coord_rel.astype(int)[0:2]), cv2.FONT_HERSHEY_PLAIN, 1.5, (255, 0, 255), 4)
                if tag.tag_id < 15 and rel_x > 0 and rel_y > 0:  # 数据库临时限制
                    tag_realpos[tag.tag_id, 0] = rel_x
                    tag_realpos[tag.tag_id, 1] = rel_y

    str_tag_realpos = ''  # 整理发给服务器的字符串
    for i in range(tag_realpos.shape[0]):
        str_tag_realpos += ', '
        str_tag_realpos += str(round(tag_realpos[i][0]))
        str_tag_realpos += ', '
        str_tag_realpos += str(round(tag_realpos[i][1]))
        str_tag_realpos += ', '
        str_tag_realpos += "0.0"  # 角度暂时没算

    return frame, str_tag_realpos


def save_image(img, rename_by_time, path="/img/"):
    '''
    True: 按照时间命名 False: 按照序号命名
    '''
    filename = sys.path[0] + path
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
    # cv2.imwrite("./img/" + filename, img) # 非中文路径保存图片
    cv2.imencode('.jpg', img)[1].tofile(filename)  # 中文路径保存图片
    print("save img successfuly!")
    print(filename)


def cal_fps():
    '''
    平滑计算帧率
    '''
    filter_length = 10
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

    # 初始化相机和数据库
    flag, cap = init_camera(1)
    robot_positions_DB = MySQL()
    robot_positions_DB.connect()

    while (flag):
        time_start = time.time()
        ret, frame = cap.read()
        # cv2.imshow("Capture_raw", frame)
        time_getpic = time.time() - time_start

        frame_undistort = undistort(frame)
        # frame_undistort = undistort_fisheye(frame)
        time_undistort = time.time() - time_start

        tags = at_detect(frame_undistort)
        time_detect = time.time() - time_start

        frame_out, tag_pos = at_print(frame_undistort, tags)
        time_project = time.time() - time_start

        # robot_positions_DB.insert(tag_pos)
        time_send2db = time.time() - time_start

        img_or_console = 1  # 显示图像or输出到命令行
        if img_or_console:
            cv2.putText(frame_out, "FPS:" + str(cal_fps()),
                        (0, 30), cv2.FONT_HERSHEY_PLAIN, 2.8, (0, 255, 0), 2)
            cv2.putText(frame_out, "getpic:" + str(round(time_getpic*1000, 4))+"ms",
                        (0, 55), cv2.FONT_HERSHEY_PLAIN, 1.8, (0, 255, 0), 2)
            cv2.putText(frame_out, "undistort:" + str(round(time_undistort*1000, 4))+"ms",
                        (0, 80), cv2.FONT_HERSHEY_PLAIN, 1.8, (0, 255, 0), 2)
            cv2.putText(frame_out, "apriltag:" + str(round(time_detect*1000, 4))+"ms",
                        (0, 105), cv2.FONT_HERSHEY_PLAIN, 1.8, (0, 255, 0), 2)
            cv2.putText(frame_out, "project:" + str(round(time_project*1000, 4))+"ms",
                        (0, 130), cv2.FONT_HERSHEY_PLAIN, 1.8, (0, 255, 0), 2)
            cv2.putText(frame_out, "send2db:" + str(round(time_send2db*1000, 4))+"ms",
                        (0, 155), cv2.FONT_HERSHEY_PLAIN, 1.8, (0, 255, 0), 2)
            cv2.imshow("Capture", frame_out)

            k = cv2.waitKey(1) & 0xFF
            if k == ord('s') or k == ord('S'):  # 按下s键，保存图片
                save_image(frame_undistort, True)
            elif k == ord('q') or k == ord('Q'):  # 按下q键，程序退出
                break
        else:
            print("FPS:" + str(cal_fps()))
            print("getpic:" + str(round(time_getpic*1000, 4))+"ms")
            print("undistort:" + str(round(time_undistort*1000, 4))+"ms")
            print("apriltag:" + str(round(time_detect*1000, 4))+"ms")
            print("project:" + str(round(time_project*1000, 4))+"ms")

    robot_positions_DB.close()
    cap.release()  # 释放摄像头
    cv2.destroyAllWindows()  # 释放并销毁窗口
