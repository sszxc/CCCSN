# Author: Xuechao Zhang
# Date: March 2nd, 2021
# Description: USB 免驱摄像头图像采集
#               AprilTag 使用个人开发者复现版本

import sys
import os
import cv2
import time
import numpy as np
from AprilTag import Apriltag

def undistort(frame):
    # 1280*800 112122-112340
    fx = 827.678512401081
    cx = 649.519595992254
    fy = 827.856142111345
    cy = 479.829876653072
    k1, k2, p1, p2, k3 = -0.335814019871572, 0.101431758719313, 0.0, 0.0, 0.0

    # 640*400 202050-202136
    # fx = 411.869480403473
    # cx = 316.221792967186
    # fy = 411.811639873307
    # cy = 239.229856557444
    # k1, k2, p1, p2, k3 = -0.354841715091639, 0.131675385816656, 8.48988830394474e-05, 0.000396990585927779, 0.0
    
    # 默认分辨率 194058-194442
    # fx = 410.891485351593
    # cx = 317.759476269588
    # fy = 410.779363278594
    # cy = 278.997726501866
    # k1, k2, p1, p2, k3 = -0.367535262603673, 0.151868281437559, 0.0, 0.0, 0.0

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

cap = cv2.VideoCapture(0)
flag = cap.isOpened()

ap = Apriltag()
ap.create_detector(debug=False)

index = 1  # 保存图片起始索引

if flag:
    timestamp = [time.time()] * 10

    # 只有这个分辨率支持 240 fps
    print("init...")
    # print(cap.set(6, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G')))
    # print(cap.set(3, 640))
    # print(cap.set(4, 400))

    frames_width = cap.get(3)
    print("width: "+str(frames_width))
    frames_height = cap.get(4)
    print("height: " + str(frames_height))
    fps = cap.get(5)
    print("FPS: " + str(fps))

while (flag):
    ret, frame = cap.read()
    frame_undistort = undistort(frame)
    cv2.imshow("Capture_raw", frame)
    cv2.imshow("Capture", frame_undistort)

    quads, detections = ap.detect(frame_undistort)  # 检测过程
    cv2.drawContours(frame_undistort, quads, -1, (0, 255, 0), 2)
    cv2.putText(frame_undistort, "FPS:" + str(fps), (0, 25), cv2.FONT_HERSHEY_PLAIN, 2.0, (0, 255, 0), 2)

    cv2.imshow("AprilTag", frame_undistort)

    k = cv2.waitKey(1) & 0xFF
    if k == ord('s') or k == ord('S'):  # 按下s键，进入下面的保存图片操作
        filename = sys.path[0] + "/img/"
        # filename += str(index)
        filename += time.strftime('%H%M%S')
        filename += ".jpg"
        # cv2.imwrite("./img/" + str(index) + ".jpg", frame)
        cv2.imencode('.jpg', frame)[1].tofile(filename)  # 中文路径保存图片
        print("save img successfuly!")
        print(filename)
        print("-------------------------")
        index += 1
    elif k == ord('q')or k == ord('Q'):  # 按下q键，程序退出
        break


    timestamp = timestamp[1:10]+[time.time()]
    fps = round(9 / (timestamp[9] - timestamp[0]), 2)  # 保留两位

cap.release() # 释放摄像头
cv2.destroyAllWindows() # 释放并销毁窗口



