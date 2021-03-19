# Author: Xuechao Zhang
# Date: March 10th, 2021
# Description: 相机 + 云台
#               根据舵机角度对相机图像透视变换

from Servo import *
from Camera import *
import math


def rad(x):
    return x * np.pi / 180


def Perspective_Transformation(img, anglex, angley, anglez):
    w, h = img.shape[0:2]

    # fov = 42
    fov = 119

    # 镜头与图像间的距离，21为半可视角，算z的距离是为了保证在此可视角度下恰好显示整幅图像
    z = np.sqrt(w ** 2 + h ** 2) / 2 / np.tan(rad(fov / 2))
    # 齐次变换矩阵
    rx = np.array([[1, 0, 0, 0],
                   [0, np.cos(rad(anglex)), -np.sin(rad(anglex)), 0],
                   [0, -np.sin(rad(anglex)), np.cos(rad(anglex)), 0, ],
                   [0, 0, 0, 1]], np.float32)

    ry = np.array([[np.cos(rad(angley)), 0, np.sin(rad(angley)), 0],
                   [0, 1, 0, 0],
                   [-np.sin(rad(angley)), 0, np.cos(rad(angley)), 0, ],
                   [0, 0, 0, 1]], np.float32)

    rz = np.array([[np.cos(rad(anglez)), np.sin(rad(anglez)), 0, 0],
                   [-np.sin(rad(anglez)), np.cos(rad(anglez)), 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]], np.float32)

    r = rx.dot(ry).dot(rz)

    # 四对点的生成
    pcenter = np.array([h / 2, w / 2, 0, 0], np.float32)

    p1 = np.array([0, 0, 0, 0], np.float32) - pcenter
    p2 = np.array([w, 0, 0, 0], np.float32) - pcenter
    p3 = np.array([0, h, 0, 0], np.float32) - pcenter
    p4 = np.array([w, h, 0, 0], np.float32) - pcenter

    dst1 = r.dot(p1)
    dst2 = r.dot(p2)
    dst3 = r.dot(p3)
    dst4 = r.dot(p4)

    list_dst = [dst1, dst2, dst3, dst4]

    org = np.array([[0, 0],
                    [w, 0],
                    [0, h],
                    [w, h]], np.float32)

    dst = np.zeros((4, 2), np.float32)

    # 投影至成像平面
    for i in range(4):
        dst[i, 0] = list_dst[i][0] * z / (z - list_dst[i][2]) + pcenter[0]
        dst[i, 1] = list_dst[i][1] * z / (z - list_dst[i][2]) + pcenter[1]

    warpR = cv2.getPerspectiveTransform(org, dst)

    result = cv2.warpPerspective(img, warpR, (h, w))

    return result


if __name__ == "__main__":
    init_port()
    init_servo(1)
    init_servo(2)
    # target_angle = [0]*12 # 12个舵机位置
    target_angle = [512, 510]
    set_angle(target_angle)

    flag, cap = init_camera(0)

    while (flag):
        ret, frame = cap.read()
        frame_undistort = undistort_fisheye(frame)

        frame_undistort = cv2.resize(
            frame_undistort, (0, 0), fx=0.25, fy=0.25, interpolation=cv2.INTER_NEAREST)

        # 黑边
        frame_undistort = cv2.copyMakeBorder(
            frame_undistort, 200, 200, 200, 200, cv2.BORDER_CONSTANT, 0)

        # 平移
        offset = -2460 * \
            math.sin(math.radians(
                510-target_angle[1]*300/1024))/0.001208*0.0001
        mat_translation = np.float32([[1, 0, 0], [0, 1, offset]])
        frame_undistort = cv2.warpAffine(
            frame_undistort, mat_translation, frame_undistort.shape[0:2])

        # 透视
        frame_undistort = Perspective_Transformation(
            frame_undistort, (510-target_angle[1])*300/1024*2.0, 0, 0)
        # 检测
        # frame_undistort = at_detect(frame_undistort)

        cv2.putText(frame_undistort, "FPS:" + str(cal_fps(10)),
                    (0, 25), cv2.FONT_HERSHEY_PLAIN, 2.0, (0, 255, 0), 2)
        cv2.imshow("Capture", frame_undistort)

        k = cv2.waitKey(1) & 0xFF
        if k == ord('A'):
            target_angle[0] -= 10
            set_angle(target_angle)
            print([target_angle])
        elif k == ord('D'):
            target_angle[0] += 10
            set_angle(target_angle)
            print([target_angle])
        elif k == ord('W'):
            target_angle[1] -= 10
            set_angle(target_angle)
            print([target_angle])
        elif k == ord('S'):
            target_angle[1] += 10
            set_angle(target_angle)
            print([target_angle])
        elif k == ord('q') or k == ord('Q'):  # 按下q键，程序退出
            break

    remove_servo(1)
    remove_servo(2)
    close_port()
    cap.release()  # 释放摄像头
    cv2.destroyAllWindows()  # 释放并销毁窗口
