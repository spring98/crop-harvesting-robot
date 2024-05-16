#!/usr/bin/env python2

from std_msgs.msg import Time
import rospy

# rospy.init_node('time', anonymous = True)
rospy.init_node('listener', anonymous=True)

pub = rospy.Publisher('/timer', Time, queue_size = 1)
rate = rospy.Rate(10)
test = Time()

while not rospy.is_shutdown():
    test.data = rospy.Time.now()
    pub.publish(test)
    rate.sleep()
    
