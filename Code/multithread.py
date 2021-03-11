# Author: Xuechao Zhang
# Date: March 10th, 2021
# Description: 双线程测试 舵机+图像处理

import multiprocessing
import time
from Servo import *
from Camera import *

def print_time(threadName, delay):
    print(threadName +" start!")
    count = 0
    while count < 5:
        time.sleep(delay)
        count += 1
        print(threadName +" "+ time.ctime(time.time()))
    print(threadName +" shut!")

def Camera_process(threadName):
    cap = cv2.VideoCapture(0)
    flag = cap.isOpened()

    at_detector = Detector(families='tag36h11',
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)

    if flag:
        print("init...")
        # print(cap.set(6, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G')))
        print(cap.set(3, 1280))
        print(cap.set(4, 800))
        print(cap.set(5, 120.0))
        # print(cap.set(cv2.CAP_PROP_FPS, 240.0))
        frames_width = cap.get(3)
        print("width: "+str(frames_width))
        frames_height = cap.get(4)
        print("height: " + str(frames_height))
        fps = cap.get(5)
        print("FPS: " + str(fps))
        print("Start!")
    else:
        print("Camera not found!")

    while (flag):
        ret, frame = cap.read()
        # cv2.imshow("Capture_raw", frame)
        frame_undistort = undistort(frame)
        if to_detect_tag:
            frame_undistort_gray = cv2.cvtColor(frame_undistort, cv2.COLOR_BGR2GRAY)
            tags = at_detector.detect(frame_undistort_gray, estimate_tag_pose=False, camera_params=None, tag_size=None)

            for tag in tags:
                cv2.circle(frame_undistort, tuple(tag.corners[0].astype(int)), 4,(255,0,0), 2) # left-top
                cv2.circle(frame_undistort, tuple(tag.corners[1].astype(int)), 4,(255,0,0), 2) # right-top
                cv2.circle(frame_undistort, tuple(tag.corners[2].astype(int)), 4,(255,0,0), 2) # right-bottom
                cv2.circle(frame_undistort, tuple(tag.corners[3].astype(int)), 4,(255,0,0), 2) # left-bottom
                cv2.putText(frame_undistort, str(tag.tag_id), tuple(tag.center.astype(int)), cv2.FONT_HERSHEY_PLAIN, 2.0, (0, 0, 255), 4)

        cv2.putText(frame_undistort, "FPS:" + str(cal_fps(10)), (0, 25), cv2.FONT_HERSHEY_PLAIN, 2.0, (0, 255, 0), 2)
        cv2.imshow("Capture", frame_undistort)

        k = cv2.waitKey(1) & 0xFF
        if k == ord('s') or k == ord('S'):  # 按下s键，进入下面的保存图片操作
            save_image(frame_undistort, False)
        elif k == ord('q')or k == ord('Q'):  # 按下q键，程序退出
            break

    cap.release() # 释放摄像头
    cv2.destroyAllWindows() # 释放并销毁窗口

def Servo_process(threadName):
    init_port()

    init_servo(1)
    init_servo(2)

    target_angle = [0]*2 # 12个舵机位置
    target_angle = [512, 700]

    # 开始测试
    while 1:
        # print("Press any key to continue! (or press ESC to quit!)")
        # if getch() == chr(0x1b):
        #     break
        
        # target_angle[0]=scs_goal_position[index]
        # target_angle[1]=scs_goal_position[index]

        a,b = (input("输入舵机目标角度(0~1023)：").split())
        # a = (input("输入舵机目标角度："))
        target_angle[0]= int(a)
        target_angle[1]= int(b)
        set_angle(target_angle)

        # # Change goal position
        # if index == 0:
        #     index = 1
        # else:
        #     index = 0
    
    remove_servo(1)
    remove_servo(2)
    close_port()

# Multi-process
record = []
lock = multiprocessing.Lock()

if __name__ == '__main__':

    process_s = multiprocessing.Process(target=Servo_process, args=('Servo', ))
    process_c = multiprocessing.Process(target=Camera_process, args=('Camera', ))
    process_c.start()
    record.append(process_c)
    process_s.start()
    record.append(process_s)

    process_c.join()
    process_s.join()