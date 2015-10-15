#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo("Listener heard %s from the messenger.", data.data)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('chatter2', String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
