# Author: Xuechao Zhang
# Date: March 8th, 2021
# Description: 飞特 SCS009 舵机测试
#               参考自 sync_write.py
#               注意在 VSCode 中使用 input 要外部终端

import os

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from scservo_sdk import *                    # Uses SCServo SDK library

# Control table address
ADDR_SCS_TORQUE_ENABLE     = 40
ADDR_STS_GOAL_ACC          = 41
ADDR_STS_GOAL_POSITION     = 42
ADDR_STS_GOAL_SPEED        = 46
ADDR_STS_PRESENT_POSITION  = 56

# 连接设置
SCS1_ID                     = 1                 # SCServo#1 舵机ID : 1
SCS2_ID                     = 2                 # SCServo#1 舵机ID : 2
BAUDRATE                    = 1000000           # SCServo default baudrate : 1000000
DEVICENAME                  = 'COM4'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

# 简单往返测试参数
SCS_MOVING_STATUS_THRESHOLD = 20                # SCServo moving status threshold
SCS_MOVING_SPEED            = 0                 # SCServo moving speed
SCS_MOVING_ACC              = 0                 # SCServo moving acc
protocol_end                = 1                 # SCServo bit end(STS/SMS=0, SCS=1)


SCS_MINIMUM_POSITION_VALUE  = 100               # SCServo will rotate between this value
SCS_MAXIMUM_POSITION_VALUE  = 1000              # and this value (note that the SCServo would not move when the position value is out of movable range. Check e-manual about the range of the SCServo you use.)

index = 0
scs_goal_position = [SCS_MINIMUM_POSITION_VALUE, SCS_MAXIMUM_POSITION_VALUE]         # Goal position

def init_port():
    '''
    打开端口 初始化波特率
    '''
    # Initialize PortHandler instance
    # Set the port path
    # Get methods and members of PortHandlerLinux or PortHandlerWindows
    global portHandler
    portHandler = PortHandler(DEVICENAME)

    # Initialize PacketHandler instance
    # Get methods and members of Protocol
    global packetHandler
    packetHandler = PacketHandler(protocol_end)

    # Initialize GroupSyncWrite instance
    global groupSyncWrite
    groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_STS_GOAL_POSITION, 2)

    # Open port
    if portHandler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch()
        quit()

    # Set port baudrate
    if portHandler.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()
    return 0

def init_servo(ID):
    '''
    写入速度加速度
    '''
    # SCServo acc
    scs_comm_result, scs_error = packetHandler.write1ByteTxRx(portHandler, ID, ADDR_STS_GOAL_ACC, SCS_MOVING_ACC)
    if scs_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(scs_comm_result))
    elif scs_error != 0:
        print("%s" % packetHandler.getRxPacketError(scs_error))

    # SCServo speed
    scs_comm_result, scs_error = packetHandler.write2ByteTxRx(portHandler, ID, ADDR_STS_GOAL_SPEED, SCS_MOVING_SPEED)
    if scs_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(scs_comm_result))
    elif scs_error != 0:
        print("%s" % packetHandler.getRxPacketError(scs_error))
    return 0

def set_angle(angle_list):
    '''
    TODO 舵机角度反馈
    '''
    # 写入目标角度
    for ID in range(1,len(angle_list)+1):
        param_goal_position = [SCS_LOBYTE(angle_list[ID-1]), SCS_HIBYTE(angle_list[ID-1])]
        # Add SCServo goal position value to the Syncwrite parameter storage
        scs_addparam_result = groupSyncWrite.addParam(ID, param_goal_position)
        if scs_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % ID)
            quit()    

    # 开始写入
    # Syncwrite goal position
    scs_comm_result = groupSyncWrite.txPacket()
    if scs_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(scs_comm_result))

    # Clear syncwrite parameter storage
    groupSyncWrite.clearParam()

    # TODO 监控舵机角度的话要用下面这段代码 多线程的时候可以试试
    # 闭环
    while 1:
        # Read SCServo#1 present position
        scs1_present_position_speed, scs_comm_result, scs_error = packetHandler.read4ByteTxRx(portHandler, SCS1_ID, ADDR_STS_PRESENT_POSITION)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            print("%s" % packetHandler.getRxPacketError(scs_error))
        scs1_present_position = SCS_LOWORD(scs1_present_position_speed)
        scs1_present_speed = SCS_HIWORD(scs1_present_position_speed)

        # Read SCServo#2 present position
        scs2_present_position_speed, scs_comm_result, scs_error = packetHandler.read4ByteTxRx(portHandler, SCS2_ID, ADDR_STS_PRESENT_POSITION)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            print("%s" % packetHandler.getRxPacketError(scs_error))
        scs2_present_position = SCS_LOWORD(scs2_present_position_speed)
        scs2_present_speed = SCS_HIWORD(scs2_present_position_speed)

        # print("[ID:%03d] GoalPos:%03d PresPos:%03d PresSpd:%03d\t[ID:%03d] GoalPos:%03d PresPos:%03d PresSpd:%03d" 
        #     % (SCS1_ID, angle, scs1_present_position, SCS_TOHOST(scs1_present_speed, 15), 
        #         SCS2_ID, angle, scs2_present_position, SCS_TOHOST(scs2_present_speed, 15)))
        # if not ((abs(angle - scs1_present_position) > SCS_MOVING_STATUS_THRESHOLD) and (abs(angle - scs2_present_position) > SCS_MOVING_STATUS_THRESHOLD)):
        #     break
        break
    return 0

def remove_servo(ID):
    '''
    扭矩限制在0
    '''
    # SCServo torque
    scs_comm_result, scs_error = packetHandler.write1ByteTxRx(portHandler, ID, ADDR_SCS_TORQUE_ENABLE, 0)
    if scs_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(scs_comm_result))
    elif scs_error != 0:
        print("%s" % packetHandler.getRxPacketError(scs_error))
    return 0

def close_port():
    '''
    关闭端口
    '''
    # Close port
    portHandler.closePort()
    return 0

if __name__ == "__main__":
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