# Author: Xuechao Zhang
# Date: May 5th, 2021
# Description: 打开两个相机 并排显示
#               软件上支持同时开启 之前的问题出在硬件上

import numpy as np
import cv2
import time

def cal_fps():
    '''
    平滑计算帧率
    '''
    filter_length = 20
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


if __name__ == '__main__':
    cap0 = cv2.VideoCapture(0)
    cap1 = cv2.VideoCapture(1)

    cap0.set(3, 640)
    cap0.set(4, 480)
    cap0.set(5, 120.0)
    cap1.set(3, 640)
    cap1.set(4, 480)
    cap1.set(5, 120.0)

    while True:
        ret, frame0 = cap0.read()
        ret, frame1 = cap1.read()

        panel = np.hstack((frame0, frame1))

        cv2.putText(panel, "FPS:" + str(cal_fps()),
                    (0, 25), cv2.FONT_HERSHEY_PLAIN, 2.0, (0, 255, 0), 2)

        cv2.imshow("panel", panel)
        keyPress = cv2.waitKey(1)
        if keyPress == ord('Q'):
            break

    cap0.release()
    cap1.release()
    cv2.destroyAllWindows()
