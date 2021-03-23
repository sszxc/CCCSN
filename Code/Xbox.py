# Author: Xuechao Zhang
# Date: March 22th, 2021
# Description: 获取Xbox手柄的输入作为调试

# 适用于 Xbox One 手柄 XPS15 电脑
# USB over Network 到 DELL7070 电脑上无法输出

import pygame
import time

def joyinit():
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count():
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        # axes = joystick.get_numaxes() # 6个
        # buttons = joystick.get_numbuttons()  # 16个 只知道十个有用
        # hats = joystick.get_numhats()  # 1个
    return joystick

def joystick_input(joystick):
    # for event in pygame.event.get():  # User did something
    #     if event.type == pygame.QUIT:  # If user clicked close
    #         done = True  # Flag that we are done so we exit this loop

    pygame.event.get()

    axis = [joystick.get_axis(i) for i in range(joystick.get_numaxes())] # 摇杆
    button = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]  # 按键
    hat = [joystick.get_hat(i) for i in range(joystick.get_numhats())]  # 十字键

    return axis, button, hat

if __name__ == "__main__":
    joystick = joyinit()
    if joystick:
        while (1):
            axis, button, hat = joystick_input(joystick)
            print(axis)
            print(button)
            print(hat)
            time.sleep(1)
