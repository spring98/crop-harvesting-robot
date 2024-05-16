#!/usr/bin/env python2

import rospy
# from std_msgs.msg import String
# from std_msgs.msg import Time
# from nav_msgs.msg import Odometry


class Decision():
    def __init__(self):
        rospy.init_node('Control', anonymous=True)
        rospy.Subscriber('/timer', Time, self.Time_call)
        rospy.Subscriber('/D2C', Odometry, self.Decision_call)
        
        self.pub = rospy.Publisher('/C2D', String, queue_size=1)
        

    def Time_call(self, time):
        return 0

    def Decision_call(self, data):
        self.x_loc = data.pose.pose.position.x
        self.y_loc = data.pose.pose.position.y
        self.z_loc = data.pose.pose.position.z

        print(self.x_loc, " ",  self.y_loc, " ", self.z_loc)




Dec = Decision()
rospy.spin()