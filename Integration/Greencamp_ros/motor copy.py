# motor veloity P control
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

    # 포트와 패킷 핸들러 초기화
    def __init__(self) -> None: # 처음 클래스가 생성될 때 이 코드가 돌아감
        self.portHandler = PortHandler(self.DEVICENAME) # port 레지스터로 들어가는 친구
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION) # 통신 packet 레지스터로 들어가는 친구
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

    def __del__(self) -> None:
        # 토크 인가 전류 해제
        self.packetHandler.write1ByteTxRx(self.portHandler, Motor.DXL_ID1, Motor.ADDR_TORQUE_ENABLE, Motor.TORQUE_DISABLE)

        # 포트 해제
        self.portHandler.closePort()

    # 모터의 각도를 조절 하는 메서드 2147483647 4294967296
    def degree(self, degree1, degree2, degree3, degree4, degree5): # 이 메서드에 들어오는 각도대로 따라가게 하는 메서드
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

        DESIRED_VELOCITY = 15

        # goal position about input degree
        goal_position_ID1 = int((degree1 / 360.0) * 4095)


        # error calculation
        error1 = goal_position_ID1 - initial_present_position_ID1 # 최대
        error2 = goal_position_ID2 - initial_present_position_ID2
        error3 = goal_position_ID3 - initial_present_position_ID3
        error4 = goal_position_ID4 - initial_present_position_ID4
        error5 = goal_position_ID5 - initial_present_position_ID5
        error6 = goal_position_ID6 - initial_present_position_ID6








        if error1 > 0:
            VELOCITY_ID1 = DESIRED_VELOCITY
        else:
            VELOCITY_ID1 = -DESIRED_VELOCITY
        
        goal_position_ID2 = int((degree2 / 360.0) * 4095)
        if error2 > 0:
            VELOCITY_ID2 = DESIRED_VELOCITY
        else:
            VELOCITY_ID2 = -DESIRED_VELOCITY
        
        goal_position_ID3 = int((degree3 / 360.0) * 4095)
        if error3 > 0:
            VELOCITY_ID3 = DESIRED_VELOCITY
        else:
            VELOCITY_ID3 = -DESIRED_VELOCITY
        
        goal_position_ID4 = int((degree4 / 360.0) * 4095)
        if error4 > 0:
            VELOCITY_ID4 = DESIRED_VELOCITY
        else:
            VELOCITY_ID4 = -DESIRED_VELOCITY
        goal_position_ID5 = int((degree5 / 360.0) * 4095)

        if error5 > 0:
            VELOCITY_ID5 = DESIRED_VELOCITY
        else:
            VELOCITY_ID5 = -DESIRED_VELOCITY

        # print(goal_position_ID3)
        # print(initial_present_position_ID3)

        # Write goal position
        self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID1, self.ADDR_GOAL_VELOCITY, VELOCITY_ID1)
        self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID2, self.ADDR_GOAL_VELOCITY, VELOCITY_ID2)
        self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID3, self.ADDR_GOAL_VELOCITY, VELOCITY_ID3)
        self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID4, self.ADDR_GOAL_VELOCITY, VELOCITY_ID4)
        self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID5, self.ADDR_GOAL_VELOCITY, VELOCITY_ID5)

        flag1 = False
        flag2 = False
        flag3 = False
        flag4 = False
        flag5 = False

        # while 1:
            # Read present position
        present_position_ID1, _, _ = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_ID1, self.ADDR_PRESENT_POSITION)
        if present_position_ID1 > 100000000:
            present_position_ID1 = present_position_ID1 - 4294967296
        present_position_ID2, _, _ = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_ID2, self.ADDR_PRESENT_POSITION)
        if present_position_ID2 > 100000000:
            present_position_ID2 = present_position_ID2 - 4294967296
        present_position_ID3, _, _ = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_ID3, self.ADDR_PRESENT_POSITION)
        if present_position_ID3 > 100000000:
            present_position_ID3 = present_position_ID3 - 4294967296
        present_position_ID4, _, _ = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_ID4, self.ADDR_PRESENT_POSITION)
        if present_position_ID4 > 100000000:
            present_position_ID4 = present_position_ID4 - 4294967296
        present_position_ID5, _, _ = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_ID5, self.ADDR_PRESENT_POSITION)
        if present_position_ID5 > 100000000:
            present_position_ID5 = present_position_ID5 - 4294967296

        print(f'ID1 -> goal angle : {(goal_position_ID1 / 4095) * 360}, present angle : {(present_position_ID1 / 4095) * 360}')
        print(f'ID2 -> goal angle : {(goal_position_ID2 / 4095) * 360}, present angle : {(present_position_ID2 / 4095) * 360}')
        print(f'ID3 -> goal angle : {(goal_position_ID3 / 4095) * 360}, present angle : {(present_position_ID3 / 4095) * 360}')
        print(f'ID4 -> goal angle : {(goal_position_ID4 / 4095) * 360}, present angle : {(present_position_ID4 / 4095) * 360}')
        print(f'ID5 -> goal angle : {(goal_position_ID5 / 4095) * 360}, present angle : {(present_position_ID5 / 4095) * 360}')

        if abs(goal_position_ID1 - present_position_ID1) < 100:
            self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID1, self.ADDR_GOAL_VELOCITY*k*(abs(goal_position_ID1 - present_position_ID1)), 0)
            
        if abs(goal_position_ID2 - present_position_ID2) < 100:
            self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID2, self.ADDR_GOAL_VELOCITY, 0)
            
        if abs(goal_position_ID3 - present_position_ID3) < 200:
            self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID3, self.ADDR_GOAL_VELOCITY, 0)
            
        if abs(goal_position_ID4 - present_position_ID4) < 100:
            self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID4, self.ADDR_GOAL_VELOCITY, 0)
            
        if abs(goal_position_ID5 - present_position_ID5) < 100:
            self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID5, self.ADDR_GOAL_VELOCITY, 0)
            
        # if flag1 and flag2 and flag3 and flag4 and flag5:
            # break

        print(
            f'flag1 : {flag1}, flag2 : {flag2}, flag3 : {flag3}, flag4 : {flag4}, flag5 : {flag5}')
