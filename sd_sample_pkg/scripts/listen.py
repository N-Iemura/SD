#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def callback(message):
    if message.data == "Left":
        rospy.loginfo("Mode_%s", message.data) # ターミナルへの表示
    if message.data == "Right":
        print("R")

rospy.init_node('listener')
sub = rospy.Subscriber('/cmd_LR', String, callback)
rospy.spin()