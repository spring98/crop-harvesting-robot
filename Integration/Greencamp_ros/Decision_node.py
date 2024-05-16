#!/usr/bin/env python2

import rospy
import math
from std_msgs.msg import String
from std_msgs.msg import Time
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from yolo_msg_pkg.msg import detection


# from custom_pkg.msg import D2C

class Decision():
    def __init__(self):
        rospy.init_node('Decision', anonymous=True)
        rospy.Subscriber('/timer', Time, self.Time_call)
        rospy.Subscriber('/is_it_tracking', Bool, self.Is_it_tracking)
        rospy.Subscriber('/xyz_sensor', detection, self.Odometry_sensing)
        rospy.Subscriber('/xyz_hardware', Odometry, self.Odometry_control)
        
        self.pub_mode = rospy.Publisher('/Decision_mode', String, queue_size=1)
        self.pub_xyz_target = rospy.Publisher('/xyz_target', Odometry, queue_size=1)
        self.x_loc = 0
        self.y_loc = 0
        self.z_loc = 0

        self.point_list = []
        self.xyz_global = Odometry()
        self.min_length = 10000000
        
        self.distance_max = 0.6
        self.distance_min = 0.5
        self.target_flag = False

        # 0.25, -0.11, 0.369
        self.x_global = -0.0764 #home 일 때 x_global 넣으면 될 듯?
        self.y_global = 0.02326 #home 일 때 y_global 넣으면 될 듯?
        self.z_glabal = 0.57 #home 일 때 z_global 넣으면 될 듯?
        self.mode = "Go_home"
        self.is_it_tracking = True # Tracking 중이야?

    def Time_call(self, time): # mode를 각각의 노드에 보내줘야함
        
        if self.mode == "Go_home" and self.is_it_tracking == True:
            self.mode = "Go_home"

        elif self.mode == "Go_home" and self.is_it_tracking == False:
            self.mode = "Go_home2_start"

        elif self.mode == "Go_home2_start" and self.is_it_tracking == True:
            self.mode = "Go_home2"


        elif self.mode == "Go_home2" and self.is_it_tracking == False:
            self.mode = "Start_tracking"
        
        elif self.mode == "Start_tracking" and self.is_it_tracking == True:
            self.mode = "Tracking"

        elif self.mode == "Tracking" and self.is_it_tracking == False:
            self.mode = "Grip_start"

        elif self.mode == "Grip_start" and self.is_it_tracking == True:
            self.mode = "Grip"
            
        elif self.mode == "Grip" and self.is_it_tracking == False:
            self.mode = "Go_home_grip_start"

        elif self.mode == "Go_home_grip_start" and self.is_it_tracking == True:
            self.mode = "Go_home_grip"

        elif self.mode == "Go_home_grip" and self.is_it_tracking == False:
            self.mode = "Let_it_go_start"

        elif self.mode == "Let_it_go_start" and self.is_it_tracking == True:
            self.mode = "Let_it_go"

        elif self.mode == "Let_it_go" and self.is_it_tracking == False:
            self.mode = "First_senario_end"

        self.pub_mode.publish(self.mode)
        # print('mode : ' + self.mode)



    def Odometry_sensing(self, data): # sensing 이랑 연동해서 사용할 부분 많은 데이터 받도록 수정
        #sensing팀이 고치기. 리스트로 받아오기
        #self.point_list = []
        print("number of targets : ",data.len)
        if(data.len > 0):
            self.point_list = list(zip(data.X,data.Y,data.Z))
            self.point_list = self.point_list[:data.len]
        
        
        print(self.point_list)
    
    def Odometry_control(self, data):
        self.x_current = data.pose.pose.position.x
        self.y_current = data.pose.pose.position.y
        self.z_current = data.pose.pose.position.z

    def Select_target(self): ########################################################
        # you should deep copy self.point_list
        # now_target_list = self.point_list[:] # (x,y,distance) 
        # # now_target_list = self.Calculation_XYZ(now_target_list) # (x, y, z)
        
        # # target_list = []
        # # l = len(now_target_list)
        # # for i in len(now_target_list): # list delete
        # #     target_list = self.Check_distance(now_target_list(1).x, now_target_list(2).y, now_target_list(3).z)
        

        # # for (x,y,z) in len(now_target_list): # closest target choice
        # #     self.Choice_target_point(x,y,z)
            
        
        # # if
        # # return True
        # if self.target_flag == True:
        #     self.pub_xyz_target.publish(self.xyz_global)
        #     return True
        # else:
            
        # if ok: # workspace 안에 있는 target을 인식할 때
        #     return True 
        # else: # workspace 밖에 있는 target을 인식할 때
        #     return False
        return False

    # def Check_distance(self, x,y,z):
    #     distance = math.sqrt(x^2 + y^2 + (z-0.3)^2)
    #     if(distance < self.distance_max) and (distance > distance_min):
    #         # ok
    #         self.target_flag = True
    #     else:
    #         # delete
    #     return target_list        


    def Choice_target_point(self,x=0,y=0,z=0): # choice target in possible points
        length = math.sqrt(x^2+y^2+z^2)
        if (length < self.min_length):
            self.xyz_global.pose.pose.position.x = x
            self.xyz_global.pose.pose.position.y = y
            self.xyz_global.pose.pose.position.z = z
            self.min_length = length
            

    # def Calculation_XYZ(self): # pixel to XYZ transform
        #x_current, y_current, z_current / x_loc, y_loc, z_loc / 필요하면 degree받아다가 계산해야함

        
        
        # Odometry msg 형태로 담아서 publish 
        # 도착지점 : Trajectory_planning_node.py/XYZ_target_call()
    
    def Is_it_tracking(self, data):
        self.is_it_tracking = data.data
        print("data : ", data.data)


Dec = Decision()
rospy.spin()