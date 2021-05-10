# Author: Xuechao Zhang
# Date: April 14th, 2021
# Description: 多线程开相机 没写明白 烂尾了

from multiprocessing import Process, Queue
import numpy as np
import cv2
import time



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

def camera_read(threadName, q, ID):
    """
    读相机
    """
    cap = cv2.VideoCapture(ID, cv2.CAP_DSHOW)
    # 加了这个DSHOW延时显著降低 但是帧率大概在10
    # https://www.javaer101.com/en/article/37683838.html

    cap.set(3, 1280)
    cap.set(4, 800)
    cap.set(5, 120.0)

    while True:
        ret, frame = cap.read()

        q.put(frame)
        break
        # cv2.imshow(str(ID), frame)
        # keyPress = cv2.waitKey(1)
        # if keyPress == ord('Q'):
        #     break

    cap.release()
    cv2.destroyAllWindows()

def img_show(threadName, q0, q1):
    """
    合并显示
    """
    
    img1 = None
    while True:
        # img0 = q0.get()
        if not q1.empty():
            # global img1
            img1 = q1.get()
        # try:
        panel = np.hstack((img1, img1))

        cv2.putText(panel, "FPS:" + str(cal_fps()),
            (0, 25), cv2.FONT_HERSHEY_PLAIN, 2.0, (0, 255, 0), 2)
        cv2.imshow("panel", panel)
        keyPress = cv2.waitKey(1)
        if keyPress == ord('Q'):
            break
        # except:
        #     pass


# Multi-process
# record = []
# lock = multiprocessing.Lock()

if __name__ == '__main__':
    q0 = Queue()
    q1 = Queue()

    process_0 = Process(target=camera_read, args=('C0', q0, 1))
    process_0.start()
    # record.append(process_0)
    process_1 = Process(target=camera_read, args=('C1', q1, 0))
    process_1.start()
    # record.append(process_1)
    process_show = Process(target=img_show, args=('show', q0, q1))
    process_show.start()
    # record.append(process_show)

    process_0.join()
    process_1.join()
    process_show.join()
