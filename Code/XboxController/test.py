# 适用于 Xbox 手柄
# 2021.03.22 后续修改参考 VirtualCam.py 项目

import pygame

pygame.init()
pygame.joystick.init()
if pygame.joystick.get_count():
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    # axes = joystick.get_numaxes() # 6个
    # buttons = joystick.get_numbuttons()  # 16个 只知道十个有用
    # hats = joystick.get_numhats()  # 1个

    while (1):
        for event in pygame.event.get():  # User did something
            if event.type == pygame.QUIT:  # If user clicked close
                done = True  # Flag that we are done so we exit this loop

        # 左右摇杆
        for i in range(4):
            axis = joystick.get_axis(i)
            if axis > 0.6 or axis < -0.6:
                print("axis"+str(i) + ":" + str(axis))

        # LT&RT
        for i in range(4, 6):
            axis = joystick.get_axis(i)
            if axis > 0.0:
                print("axis"+str(i) + ":" + str(axis))

        # 按键
        for i in range(10):
            button = joystick.get_button(i)
            if button != 0:
                print("buttons"+str(i)+":"+str(button))

        # 十字键
        for i in range(1):
            hat = joystick.get_hat(i)
            if hat != (0, 0):
                print("hat"+str(i)+":"+str(hat))
