from math import atan2, cos, pi, sin
import sympy as sp


def kinematics(Px, Py, Pz) -> tuple:
    # 고정 링크 parameter
    a2 = 0.3  # L1
    a3 = 0
    d3 = 0
    d4 = 0.3  # L2

    # 방위 파라 미터
    R11 = 1
    R12 = 0
    R13 = 0
    R21 = 0
    R22 = -1
    R23 = 0
    R31 = 0
    R32 = 0
    R33 = -1

    # 위치 parameter
    # Px = 1
    # Py = 1
    # Pz = -1

    # 최종 매트 릭스
    T06 = sp.Matrix(([R11, R12, R13, Px],
                     [R21, R22, R23, Py],
                     [R31, R32, R33, Pz],
                     [0, 0, 0, 1],))

    th1 = sp.Matrix(([atan2(Py, Px) - atan2(d3, (Px ** 2 + Py ** 2 - d3 ** 2) ** 0.5)],
                     [atan2(Py, Px) - atan2(d3, (Px ** 2 + Py ** 2 - d3 ** 2) ** 0.5)],
                     [atan2(Py, Px) - atan2(d3, -(Px ** 2 + Py ** 2 - d3 ** 2) ** 0.5)],
                     [atan2(Py, Px) - atan2(d3, -(Px ** 2 + Py ** 2 - d3 ** 2) ** 0.5)],))

    K = (Px ** 2 + Py ** 2 + Pz ** 2 - a2 ** 2 - a3 ** 2 - d3 ** 2 - d4 ** 2) / 2 * a2

    th3 = sp.Matrix(([atan2(a3, d4) - atan2(K, (a3 ** 2 + d4 ** 2 - K ** 2) ** 0.5)],
                     [atan2(a3, d4) - atan2(K, -(a3 ** 2 + d4 ** 2 - K ** 2) ** 0.5)],
                     [atan2(a3, d4) - atan2(K, (a3 ** 2 + d4 ** 2 - K ** 2) ** 0.5)],
                     [atan2(a3, d4) - atan2(K, -(a3 ** 2 + d4 ** 2 - K ** 2) ** 0.5)],))

    th23 = sp.Matrix(
        ([atan2((-a3 - a2 * cos(th3[0])) * Pz - (cos(th1[0]) * Px + sin(th1[0]) * Py) * (d4 - a2 * sin(th3[0])),
                (a2 * sin(th3[0]) - d4) * Pz + (a3 + a2 * cos(th3[0])) * (cos(th1[0]) * Px + sin(th1[0]) * Py))],
         [atan2((-a3 - a2 * cos(th3[1])) * Pz - (cos(th1[1]) * Px + sin(th1[1]) * Py) * (d4 - a2 * sin(th3[1])),
                (a2 * sin(th3[1]) - d4) * Pz + (a3 + a2 * cos(th3[1])) * (cos(th1[1]) * Px + sin(th1[1]) * Py))],
         [atan2((-a3 - a2 * cos(th3[2])) * Pz - (cos(th1[2]) * Px + sin(th1[2]) * Py) * (d4 - a2 * sin(th3[2])),
                (a2 * sin(th3[2]) - d4) * Pz + (a3 + a2 * cos(th3[2])) * (cos(th1[2]) * Px + sin(th1[2]) * Py))],
         [atan2((-a3 - a2 * cos(th3[3])) * Pz - (cos(th1[3]) * Px + sin(th1[3]) * Py) * (d4 - a2 * sin(th3[3])),
                (a2 * sin(th3[3]) - d4) * Pz + (a3 + a2 * cos(th3[3])) * (cos(th1[3]) * Px + sin(th1[3]) * Py))],))

    th2 = th23 - th3

    th4 = sp.Matrix(([atan2(-R13 * sin(th1[0]) + R23 * cos(th1[0]),
                            -R13 * cos(th1[0]) * cos(th2[0] + th3[0]) - R23 * sin(th1[0]) * cos(
                                th2[0] + th3[0]) + R33 * sin(th2[0] + th3[0]))],
                     [atan2(-R13 * sin(th1[1]) + R23 * cos(th1[1]),
                            -R13 * cos(th1[1]) * cos(th2[1] + th3[1]) - R23 * sin(th1[1]) * cos(
                                th2[1] + th3[1]) + R33 * sin(th2[1] + th3[1]))],
                     [atan2(-R13 * sin(th1[2]) + R23 * cos(th1[2]),
                            -R13 * cos(th1[2]) * cos(th2[2] + th3[2]) - R23 * sin(th1[2]) * cos(
                                th2[2] + th3[2]) + R33 * sin(th2[2] + th3[2]))],
                     [atan2(-R13 * sin(th1[3]) + R23 * cos(th1[3]),
                            -R13 * cos(th1[3]) * cos(th2[3] + th3[3]) - R23 * sin(th1[3]) * cos(
                                th2[3] + th3[3]) + R33 * sin(th2[3] + th3[3]))],))

    th5 = sp.Matrix(([atan2(
        -R13 * (cos(th1[0]) * cos(th2[0] + th3[0]) * cos(th4[0]) + sin(th1[0]) * sin(th4[0])) - R23 * (
                sin(th1[0]) * cos(th2[0] + th3[0]) * cos(th4[0]) - cos(th1[0]) * sin(th4[0])) + R33 * sin(
            th2[0] + th3[0]) * cos(th4[0]),
        -R13 * cos(th1[0]) * sin(th2[0] + th3[0]) - R23 * sin(th1[0]) * sin(th2[0] + th3[0]) - R33 * cos(
            th2[0] + th3[0]))],
                     [atan2(
                         -R13 * (cos(th1[1]) * cos(th2[1] + th3[1]) * cos(th4[1]) + sin(th1[1]) * sin(th4[1])) - R23 * (
                                 sin(th1[1]) * cos(th2[1] + th3[1]) * cos(th4[1]) - cos(th1[1]) * sin(
                             th4[1])) + R33 * sin(th2[1] + th3[1]) * cos(th4[1]),
                         -R13 * cos(th1[1]) * sin(th2[1] + th3[1]) - R23 * sin(th1[1]) * sin(
                             th2[1] + th3[1]) - R33 * cos(th2[1] + th3[1]))],
                     [atan2(
                         -R13 * (cos(th1[2]) * cos(th2[2] + th3[2]) * cos(th4[2]) + sin(th1[2]) * sin(th4[2])) - R23 * (
                                 sin(th1[2]) * cos(th2[2] + th3[2]) * cos(th4[2]) - cos(th1[2]) * sin(
                             th4[2])) + R33 * sin(th2[2] + th3[2]) * cos(th4[2]),
                         -R13 * cos(th1[2]) * sin(th2[2] + th3[2]) - R23 * sin(th1[2]) * sin(
                             th2[2] + th3[2]) - R33 * cos(th2[2] + th3[2]))],
                     [atan2(
                         -R13 * (cos(th1[3]) * cos(th2[3] + th3[3]) * cos(th4[3]) + sin(th1[3]) * sin(th4[3])) - R23 * (
                                 sin(th1[3]) * cos(th2[3] + th3[3]) * cos(th4[3]) - cos(th1[3]) * sin(
                             th4[3])) + R33 * sin(th2[3] + th3[3]) * cos(th4[3]),
                         -R13 * cos(th1[3]) * sin(th2[3] + th3[3]) - R23 * sin(th1[3]) * sin(
                             th2[3] + th3[3]) - R33 * cos(th2[3] + th3[3]))],))

    th6 = sp.Matrix(([atan2(
        -R11 * (cos(th1[0]) * cos(th2[0] + th3[0]) * sin(th4[0]) - sin(th1[0]) * cos(th4[0])) - R21 * (
                sin(th1[0]) * cos(th2[0] + th3[0]) * sin(th4[0]) + cos(th1[0]) * cos(th4[0])) + R31 * (
                sin(th2[0] + th3[0]) * sin(th4[0])), R11 * (
                (cos(th1[0]) * cos(th2[0] + th3[0]) * cos(th4[0]) + sin(th1[0]) * sin(th4[0])) * cos(th5[0]) - cos(
            th1[0]) * sin(th2[0] + th3[0]) * sin(th5[0])) + R21 * (
                (sin(th1[0]) * cos(th2[0] + th3[0]) * cos(th4[0]) - cos(th1[0]) * sin(th4[0])) * cos(th5[0]) - sin(
            th1[0]) * sin(th2[0] + th3[0]) * sin(th5[0])) - R31 * (
                sin(th2[0] + th3[0]) * cos(th4[0]) * cos(th5[0]) + cos(th2[0] + th3[0]) * sin(th5[0])))],
                     [atan2(
                         -R11 * (cos(th1[1]) * cos(th2[1] + th3[1]) * sin(th4[1]) - sin(th1[1]) * cos(th4[1])) - R21 * (
                                 sin(th1[1]) * cos(th2[1] + th3[1]) * sin(th4[1]) + cos(th1[1]) * cos(
                             th4[1])) + R31 * (sin(th2[1] + th3[1]) * sin(th4[1])), R11 * ((cos(th1[1]) * cos(
                             th2[1] + th3[1]) * cos(th4[1]) + sin(th1[1]) * sin(th4[1])) * cos(th5[1]) - cos(
                             th1[1]) * sin(th2[1] + th3[1]) * sin(th5[1])) + R21 * ((sin(th1[1]) * cos(
                             th2[1] + th3[1]) * cos(th4[1]) - cos(th1[1]) * sin(th4[1])) * cos(th5[1]) - sin(
                             th1[1]) * sin(th2[1] + th3[1]) * sin(th5[1])) - R31 * (
                                 sin(th2[1] + th3[1]) * cos(th4[1]) * cos(th5[1]) + cos(th2[1] + th3[1]) * sin(
                             th5[1])))],
                     [atan2(
                         -R11 * (cos(th1[2]) * cos(th2[2] + th3[2]) * sin(th4[2]) - sin(th1[2]) * cos(th4[2])) - R21 * (
                                 sin(th1[2]) * cos(th2[2] + th3[2]) * sin(th4[2]) + cos(th1[2]) * cos(
                             th4[2])) + R31 * (sin(th2[2] + th3[2]) * sin(th4[2])), R11 * ((cos(th1[2]) * cos(
                             th2[2] + th3[2]) * cos(th4[2]) + sin(th1[2]) * sin(th4[2])) * cos(th5[2]) - cos(
                             th1[2]) * sin(th2[2] + th3[2]) * sin(th5[2])) + R21 * ((sin(th1[2]) * cos(
                             th2[2] + th3[2]) * cos(th4[2]) - cos(th1[2]) * sin(th4[2])) * cos(th5[2]) - sin(
                             th1[2]) * sin(th2[2] + th3[2]) * sin(th5[2])) - R31 * (
                                 sin(th2[2] + th3[2]) * cos(th4[2]) * cos(th5[2]) + cos(th2[2] + th3[2]) * sin(
                             th5[2])))],
                     [atan2(
                         -R11 * (cos(th1[3]) * cos(th2[3] + th3[3]) * sin(th4[3]) - sin(th1[3]) * cos(th4[3])) - R21 * (
                                 sin(th1[3]) * cos(th2[3] + th3[3]) * sin(th4[3]) + cos(th1[3]) * cos(
                             th4[3])) + R31 * (sin(th2[3] + th3[3]) * sin(th4[3])), R11 * ((cos(th1[3]) * cos(
                             th2[3] + th3[3]) * cos(th4[3]) + sin(th1[3]) * sin(th4[3])) * cos(th5[3]) - cos(
                             th1[3]) * sin(th2[3] + th3[3]) * sin(th5[3])) + R21 * ((sin(th1[3]) * cos(
                             th2[3] + th3[3]) * cos(th4[3]) - cos(th1[3]) * sin(th4[3])) * cos(th5[3]) - sin(
                             th1[3]) * sin(th2[3] + th3[3]) * sin(th5[3])) - R31 * (
                                 sin(th2[3] + th3[3]) * cos(th4[3]) * cos(th5[3]) + cos(th2[3] + th3[3]) * sin(
                             th5[3])))],))

    solution1 = [th1[0], -th2[0], th3[0], th4[0], th5[0], th6[0]]
    solution2 = [th1[1], -th2[1], th3[1], th4[1], th5[1], th6[1]]
    solution3 = [th1[2], -th2[2], th3[2], th4[2], th5[2], th6[2]]
    solution4 = [th1[3], -th2[3], th3[3], th4[3], th5[3], th6[3]]

    sol1 = [round(x * 180 / pi) for x in solution1]
    sol2 = [round(x * 180 / pi) for x in solution2]
    sol3 = [round(x * 180 / pi) for x in solution3]
    sol4 = [round(x * 180 / pi) for x in solution4]

    sols = (sol1, sol2, sol3, sol4)
    # print(sols[0])
    # print(sols[1])
    # print(sols[2])
    # print(sols[3])

    # kinematics()
    return sols


def take():
    print('ppp')
