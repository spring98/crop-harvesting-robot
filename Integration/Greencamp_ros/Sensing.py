#!/usr/bin/env python2

import rospy
# ros 통신을 위해서는 통신하는 토픽 메세지의 메세지 타입을 import 해야함 / 이미 만들어진 무수히 많은 ros msg가 있는데 필요하면 구글링해서 필요한거 쓰면 됌
from std_msgs.msg import String
from std_msgs.msg import Time
from nav_msgs.msg import Odometry

class Sensing():
    def __init__(self):
        rospy.init_node('Sensing', anonymous=True) # 노드 이름 설정 + 노드 시작
        rospy.Subscriber('/timer', Time, self.Time_call) # /timer라는 이름을 가진 토픽 메세지를 subscriber함. rospy.Subscriber('메세지이름', 메세지타입, 콜백함수) + 토픽 메세지를 받을때마다 콜백함수가 한바퀴 돌게됌 loop개념
        rospy.Subscriber('/D2S', String, self.Decision_call)

        self.pub = rospy.Publisher('/S2D', Odometry, queue_size=1) # /S2D라는 이름을 가진 토픽 메세지를 Publish함. rospy.Publisher('메세지이름', 메세지타입, 큐사이즈)
                                                                   # Queue Size는 발행되는 메세지를 얼마나 가지고 있을지에 관련된 변수이며 신규 데이터가 들어오는 경우 오래된 데이터부터 삭제하게 된다.
        self.msg = Odometry()  # msg 타입 설정

    def Time_call(self, data): # 

        # msg에 카메라 상대좌표 위치 계산한거 넣으면 publish 해줌 // msg형식에 맞게 pose.pose~~ 쓴거임
        self.msg.pose.pose.position.x = 1
        self.msg.pose.pose.position.y = 2
        self.msg.pose.pose.position.z = 3

        self.pub.publish(self.msg)  # 메세지 안에 필요한 값을 넣었다면 이렇게 msg를 넣어서 publish를 하면 됌

        print(self.msg.pose.pose.position.x)
        print('\n')
    
    def Decision_call(self, data): # /D2S 토픽 메세지를 받을 때마다 도는 함수
        # if data == "??":
        # 여기에 Decision으로 부터 받은 상태 정보 받을 수 있음.
        print(data)




Sen = Sensing()
rospy.spin() # node가 꺼질때까지 코드를 킬거야