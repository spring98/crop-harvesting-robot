#!/usr/bin/env python2

R2D = 180/3.141592
import sensor_msgs.msg
from sensor_msgs.msg import JointState

import rospy
from std_msgs.msg import Time

import time
import motor
import kinematics

motor = motor.Motor()

class Moveit():
    def __init__(self):
        rospy.init_node('listener', anonymous=True)

        rospy.Subscriber("/joint_states",  JointState, self.callback)
        self.number = 0
    
    def callback(self, data):
        if self.number > 5:
            self.data = data.position
            
            motor.degree(self.data[0]*R2D, self.data[1]*R2D, self.data[2]*R2D, self.data[3]*R2D, self.data[4]*R2D)
            self.number = 0
            
        self.number += 1    
        
        # 만약 esc 들어오면 motor.input() - velocity 0으로 가게끔 만드는 것.
        # motor.degree(0, 0, 0, 0, self.data[4]*R2D)\



        
if __name__ == '__main__':
    mv = Moveit()
    rospy.spin()





# for i in range(1, 2):
#     # 1 1 0 -> 1 1 1
#     # Px = 1
#     # Py = 1
#     # Pz = i * 0.1

#     # Py = -0.1
#     # Px = 0.1
#     Py = 0
#     Px = 0
#     Pz = 0

#     result = kinematics.kinematics(Px, Py, Pz)
#     # print(result)

#     # motor.degree(result[1][0], result[1][1], result[1][2])
#     motor.degree(result[1][0], result[1][1], result[1][2], result[1][3], result[1][4])