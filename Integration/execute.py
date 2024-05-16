import time
import motor
import kinematics

# .py = module
# .py가 포함된 폴더 = package
# from abc import xyz : abc 라는 package(폴더) 에서 xyz 라는 module (파일)을 불러옴

# set home
motor = motor.Motor()
# motor.degree(0, 0, 0, 0, 0)

for i in range(1, 2):
    # 1 1 0 -> 1 1 1
    # Px = 1
    # Py = 1
    # Pz = i * 0.1

    # Py = -0.1
    # Px = 0.1
    Py = 0
    Px = 0
    Pz = 0

    result = kinematics.kinematics(Px, Py, Pz) # 여기서는 위치에 대한 모터 각도를 역기구학을 통해서 뽑아냄
    # print(result)

    # motor.degree(result[1][0], result[1][1], result[1][2])
    motor.degree(result[1][0], result[1][1], result[1][2], result[1][3], result[1][4])
# #
# #     # range is -150 < degree < +150
# #     if (-150 < result[0][3] < 150) and (-150 < result[0][4] < 150) and (-150 < result[0][5] < 150):
# #         motor.degree(result[0][3], result[0][4], result[0][5])
# #     elif (-150 < result[1][3] < 150) and (-150 < result[1][4] < 150) and (-150 < result[1][5] < 150):
# #         motor.degree(result[1][3], result[1][4], result[1][5])
# #     elif (-150 < result[2][3] < 150) and (-150 < result[2][4] < 150) and (-150 < result[2][5] < 150):
# #         motor.degree(result[2][3], result[2][4], result[2][5])
# #     elif (-150 < result[3][3] < 150) and (-150 < result[3][4] < 150) and (-150 < result[3][5] < 150):
# #         motor.degree(result[3][3], result[3][4], result[3][5])
# #     else:
# #         break
