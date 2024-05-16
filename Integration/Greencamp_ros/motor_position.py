# motor position

import os # 경로 뽑는 모듈  # 이것은 운영체제(OS : Operating System)를 제어

if os.name == 'nt': # 'nt'를 리턴할 때는 윈도우 posix를 리턴 할 때는 리눅스
    import msvcrt


    def getch():
        return msvcrt.getch().decode() # 키 누르기를 읽고 결과 문자를 바이트열로 반환합니다
else:
    import sys, tty, termios # sys모듈을 이용하면 현재 프롬프트를 바꿀 수 있다.
                             

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)


    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import *  # Uses Dynamixel SDK library

DISCRETE_TO_DEGREE = 360 / 4095
DEGREE_TO_DISCRETE = 4095 / 360
TOLERANCE_DEGREE = 1
CRITERIA = TOLERANCE_DEGREE * DEGREE_TO_DISCRETE 



class Motor:
    # Control table address
    ADDR_TORQUE_ENABLE = 64 # ADDR 주소
    ADDR_GOAL_POSITION = 116
    ADDR_PRESENT_POSITION = 132
    ADDR_GOAL_VELOCITY = 104
    ADDR_PRESENT_VELOCITY = 128

    # Protocol version
    PROTOCOL_VERSION = 2.0

    # Default setting
    DXL_ID1 = 1 
    DXL_ID2 = 2
    DXL_ID3 = 3
    DXL_ID4 = 4
    DXL_ID5 = 5
    DXL_ID6 = 6

    BAUDRATE = 57600
    # DEVICENAME = '/dev/tty.usbserial-FT66WBIV'
    DEVICENAME = '/dev/ttyUSB0'
    
    TORQUE_ENABLE = 1
    TORQUE_DISABLE = 0
    
    #하드웨어 기준이야 1이면 있는 곳
    home = 1
    target = 0

    # home = 1 and tracking_done = 1
    # home = 0 and tracking_done = 1 이게 start tracking
    # home = 0 and tracking_done = 0 이건 tacking
    # home = 0 and tracking_done = 1 이게 tracking done

    # 포트와 패킷 핸들러 초기화
    def __init__(self) -> None: # 처음 클래스가 생성될 때 이 코드가 돌아감
        self.portHandler = PortHandler(self.DEVICENAME) # port 레지스터로 들어가는 친구
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION) # 통신 packet 레지스터로 들어가는 친구
        self.error1 = 0
        self.error2 = 0
        self.error3 = 0
        self.error4 = 0
        self.error5 = 0 
        self.error6 = 0
        self.error7 = 0
        

        # 포트 열기
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()

        # 포트를 보드와 맞는 전송 속도로 맞춤
        if self.portHandler.setBaudRate(self.BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()

        # 토크 인가
        self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID1, self.ADDR_TORQUE_ENABLE,
                                          self.TORQUE_ENABLE)
        self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID2, self.ADDR_TORQUE_ENABLE,
                                          self.TORQUE_ENABLE)
        self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID3, self.ADDR_TORQUE_ENABLE,
                                          self.TORQUE_ENABLE)
        self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID4, self.ADDR_TORQUE_ENABLE,
                                          self.TORQUE_ENABLE)
        self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID5, self.ADDR_TORQUE_ENABLE,
                                          self.TORQUE_ENABLE)
        self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID6, self.ADDR_TORQUE_ENABLE,
                                          self.TORQUE_ENABLE)
        # self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID7, self.ADDR_TORQUE_ENABLE,
        #                                   self.TORQUE_ENABLE)
                                          

    def __del__(self) -> None:
        # 토크 인가 전류 해제
        self.packetHandler.write1ByteTxRx(self.portHandler, Motor.DXL_ID1, Motor.ADDR_TORQUE_ENABLE, Motor.TORQUE_DISABLE)
        self.packetHandler.write1ByteTxRx(self.portHandler, Motor.DXL_ID2, Motor.ADDR_TORQUE_ENABLE, Motor.TORQUE_DISABLE)
        self.packetHandler.write1ByteTxRx(self.portHandler, Motor.DXL_ID3, Motor.ADDR_TORQUE_ENABLE, Motor.TORQUE_DISABLE)
        self.packetHandler.write1ByteTxRx(self.portHandler, Motor.DXL_ID4, Motor.ADDR_TORQUE_ENABLE, Motor.TORQUE_DISABLE)
        self.packetHandler.write1ByteTxRx(self.portHandler, Motor.DXL_ID5, Motor.ADDR_TORQUE_ENABLE, Motor.TORQUE_DISABLE)
        self.packetHandler.write1ByteTxRx(self.portHandler, Motor.DXL_ID6, Motor.ADDR_TORQUE_ENABLE, Motor.TORQUE_DISABLE)
        # self.packetHandler.write1ByteTxRx(self.portHandler, Motor.DXL_ID7, Motor.ADDR_TORQUE_ENABLE, Motor.TORQUE_DISABLE)
        

        # 포트 해제
        self.portHandler.closePort()

    # 모터의 각도를 조절 하는 메서드 2147483647 4294967296
    # def degree(self, degree1, degree2, degree3, degree4, degree5, degree6): # 이 메서드에 들어오는 각도대로 따라가게 하는 메서드
    def degree(self, degree1, degree2, degree3, degree4, degree5, degree6): # 이 메서드에 들어오는 각도대로 따라가게 하는 메서드
        print('가동중...')
        
        
        # initial present position
        initial_present_position_ID1, _, _ = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_ID1, self.ADDR_PRESENT_POSITION)
        if initial_present_position_ID1 > 100000000:
            initial_present_position_ID1 = initial_present_position_ID1 - 4294967296
        initial_present_position_ID2, _, _ = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_ID2, self.ADDR_PRESENT_POSITION)
        if initial_present_position_ID2 > 100000000:
            initial_present_position_ID2 = initial_present_position_ID2 - 4294967296
        initial_present_position_ID3, _, _ = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_ID3, self.ADDR_PRESENT_POSITION)
        if initial_present_position_ID3 > 100000000:
            initial_present_position_ID3 = initial_present_position_ID3 - 4294967296
        initial_present_position_ID4, _, _ = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_ID4, self.ADDR_PRESENT_POSITION)
        if initial_present_position_ID4 > 100000000:
            initial_present_position_ID4 = initial_present_position_ID4 - 4294967296
        initial_present_position_ID5, _, _ = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_ID5, self.ADDR_PRESENT_POSITION)
        if initial_present_position_ID5 > 100000000:
            initial_present_position_ID5 = initial_present_position_ID5 - 4294967296
        initial_present_position_ID6, _, _ = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_ID6, self.ADDR_PRESENT_POSITION)
        if initial_present_position_ID6 > 100000000:
            initial_present_position_ID6 = initial_present_position_ID6 - 4294967296

        DESIRED_VELOCITY = 15
        MAX_VELOCITY_RESOLUTION = 4095
        
        VELOCITY_PER_RESOLUTION = DESIRED_VELOCITY / MAX_VELOCITY_RESOLUTION
        P_GAIN = 1 
        # goal position about input degree
        goal_position_ID1 = int((degree1 / 360.0) * 4095)
        goal_position_ID2 = int((degree2 / 360.0) * 4095)
        goal_position_ID3 = int((degree3 / 360.0) * 4095)
        goal_position_ID4 = int((degree4 / 360.0) * 4095)
        goal_position_ID5 = int((degree5 / 360.0) * 4095)
        goal_position_ID6 = int((degree6 / 360.0) * 4095)
        


        # error calculation
        self.error1 = (goal_position_ID1 - initial_present_position_ID1) # 최대 4095야
        self.error2 = (goal_position_ID2 - initial_present_position_ID2)
        self.error3 = (goal_position_ID3 - initial_present_position_ID3)
        self.error4 = (goal_position_ID4 - initial_present_position_ID4)
        self.error5 = (goal_position_ID5 - initial_present_position_ID5)
        self.error6 = (goal_position_ID6 - initial_present_position_ID6)
        # self.error7 = (goal_position_ID7 - initial_present_position_ID7)

        dxl_comm_result1, dxl_error1 = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID1, self.ADDR_GOAL_POSITION, goal_position_ID1)
        dxl_comm_result2, dxl_error2 = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID2, self.ADDR_GOAL_POSITION, goal_position_ID2)
        dxl_comm_result3, dxl_error3 = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID3, self.ADDR_GOAL_POSITION, goal_position_ID3)
        dxl_comm_result4, dxl_error4 = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID4, self.ADDR_GOAL_POSITION, goal_position_ID4)
        dxl_comm_result5, dxl_error5 = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID5, self.ADDR_GOAL_POSITION, goal_position_ID5)
        dxl_comm_result6, dxl_error6 = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID6, self.ADDR_GOAL_POSITION, goal_position_ID6)
        # dxl_comm_result7, dxl_error7 = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID7, self.ADDR_GOAL_POSITION, goal_position_ID7)

        flag1 = False
        flag2 = False
        flag3 = False
        flag4 = False
        flag5 = False
        flag6 = False
        # flag6 = False

        # print(f'ID1 -> goal angle : {(goal_position_ID1 / 4095) * 360}, present angle : {(present_position_ID1 / 4095) * 360}')
        # print(f'ID2 -> goal angle : {(goal_position_ID2 / 4095) * 360}, present angle : {(present_position_ID2 / 4095) * 360}')
        # print(f'ID3 -> goal angle : {(goal_position_ID3 / 4095) * 360}, present angle : {(present_position_ID3 / 4095) * 360}')
        # print(f'ID4 -> goal angle : {(goal_position_ID4 / 4095) * 360}, present angle : {(present_position_ID4 / 4095) * 360}')
        # print(f'ID5 -> goal angle : {(goal_position_ID5 / 4095) * 360}, present angle : {(present_position_ID5 / 4095) * 360}')

        print( "criteria : %d, error1 : %d, error2 : %d, error3 : %d, error4 : %d, error5 : %d" % (CRITERIA, abs(self.error1),abs(self.error2), abs(self.error3), abs(self.error4), abs(self.error5)))

    def Is_it_tracking(self):
        # if (abs(self.error1) < CRITERIA) and (abs(self.error2) < CRITERIA) and (abs(self.error3) < CRITERIA) and (abs(self.error4) < CRITERIA) and (abs(self.error5) < CRITERIA) and (abs(self.error6) < CRITERIA): @@add@@
        if (abs(self.error1) < CRITERIA) and (abs(self.error2) < CRITERIA) and (abs(self.error3) < CRITERIA) and (abs(self.error4) < CRITERIA) and (abs(self.error5) < CRITERIA):
            print("No Tracking IS DONE!!!!")
            return False
        else:
            print("I'M TRACKING THE PATH!!!!")
            return True
    
    def Is_it_gripped(self):
        if abs(self.error7 < 1*DEGREE_TO_DISCRETE):
            return True
        else:
            return False