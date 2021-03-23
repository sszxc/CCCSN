# Author: Xuechao Zhang
# Date: March 10th, 2021
# Description: 相机 + 云台
#               键盘/手柄控制云台舵机
#               相机图像自适应透视变换

from Servo import *
from Camera import *
import math
from Xbox import *
from VirtualCam import Cam
import copy

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

    # 模仿虚拟相机 先建立实例
    # 创建理想俯视图相机
    topview = Cam()
    topview.init_IM(500, 500, -500, -300)
    topview.init_T([0, 0, 0], 2500, 1500, 0)
    # 创建普通节点相机
    cam_1 = Cam()
    cam_1.init_IM(827.678512401081, 827.856142111345,
                  649.519595992254, 479.829876653072)
    cam_1_T_para_defult = [[0, 0, 0], -3000, -1500, -500]
    cam_1_T_para = copy.deepcopy(cam_1_T_para_defult)
    cam_1.init_T(*cam_1_T_para)

    # 四个空间点
    points = [0] * 4
    points[0] = np.array([[0], [0], [2500]], dtype=float)
    points[1] = np.array([[0], [3000], [2500]], dtype=float)
    points[2] = np.array([[5000], [3000], [2500]], dtype=float)
    points[3] = np.array([[5000], [0], [2500]], dtype=float)

    # 节点相机的角点
    corners = []
    corners.append(np.array([0, 0, 1], dtype=np.float32))
    corners.append(np.array([1280, 0, 1], dtype=np.float32))
    corners.append(np.array([0, 800, 1], dtype=np.float32))
    corners.append(np.array([1280, 800, 1], dtype=np.float32))

    # 初始化手柄控制
    joystick = joyinit()

    while (flag):
        real_angle = set_angle(target_angle)

        topview.init_view()

        # 从舵机角度计算cam_1_T_para
        cam_1_T_para[0][0] = (real_angle[1] - 510) * 0.3
        cam_1_T_para[0][2] = (real_angle[0] - 512) * 0.3
        cam_1.init_T(*cam_1_T_para)

        pixel_0 = []
        pixel_1 = []
        for point in points:
            pixel_0.append(topview.capture(point))
            pixel_1.append(cam_1.capture(point))

        # 四个点生成变换矩阵
        pixel_before = np.float32(pixel_0[0:4])
        pixel_after = np.float32(pixel_1[0:4])
        M = cv2.getPerspectiveTransform(pixel_before, pixel_after)

        M_inv = np.linalg.inv(M)
        for corner in corners:
            corner_projected = np.dot(M_inv, corner.T)
            corner_projected /= corner_projected[2]  # 归一化
            cv2.circle(topview.img, tuple(corner_projected.astype(
                np.int).T.tolist()[0:2]), 20, (0, 255, 0), -1)
        
        ret, frame = cap.read()
        frame_undistort = undistort_fisheye(frame)

        frame_projected = cv2.warpPerspective(
            frame_undistort, M_inv, (1000, 600), borderValue=(255, 255, 255))

        # # 黑边
        # frame_undistort = cv2.copyMakeBorder(
        #     frame_undistort, 200, 200, 200, 200, cv2.BORDER_CONSTANT, 0)

        # # 平移
        # offset = -2460 * \
        #     math.sin(math.radians(
        #         510-target_angle[1]*300/1024))/0.001208*0.0001
        # mat_translation = np.float32([[1, 0, 0], [0, 1, offset]])
        # frame_undistort = cv2.warpAffine(
        #     frame_undistort, mat_translation, frame_undistort.shape[0:2])

        # # 透视
        # frame_undistort = Perspective_Transformation(
        #     frame_undistort, (510-target_angle[1])*300/1024*2.0, 0, 0)
        # 检测
        # frame_undistort = at_detect(frame_undistort)

        # frame_undistort = cv2.resize(
        #     frame_undistort, (0, 0), fx=0.25, fy=0.25, interpolation=cv2.INTER_NEAREST)

        cv2.putText(frame_projected, "FPS:" + str(cal_fps(10)),
                    (0, 25), cv2.FONT_HERSHEY_PLAIN, 2.0, (0, 255, 0), 2)
        cv2.imshow("Capture", frame_projected)

        if joystick:
            # A 退出 B 复位
            axis, button, hat = joystick_input(joystick)
            target_angle[0] = 512 + int(axis[3] * 200)
            target_angle[1] = 510 + int(axis[1] * 100)
            if button[0] == 1:
                break
            elif button[1] == 1:
                target_angle = [512, 510]
            
        else:
            k = cv2.waitKey(1) & 0xFF
            if k == ord('A'):
                target_angle[0] -= 10
                print([target_angle])
            elif k == ord('D'):
                target_angle[0] += 10
                print([target_angle])
            elif k == ord('W'):
                target_angle[1] -= 10
                print([target_angle])
            elif k == ord('S'):
                target_angle[1] += 10
                print([target_angle])
            elif k == ord('q') or k == ord('Q'):  # 按下q键，程序退出
                break

    remove_servo(1)
    remove_servo(2)
    close_port()
    cap.release()  # 释放摄像头
    cv2.destroyAllWindows()  # 释放并销毁窗口
