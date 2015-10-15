#!/usr/bin/env python

import rospy
from std_msgs.msg import String

pub = rospy.Publisher('chatter2', String, queue_size=10)

def messengerCallback(data):
    global pub
    messenger_str = "Forwarding %s" % data.data
    pub.publish(messenger_str)

def messenger():
    rospy.init_node('messenger', anonymous=True)
    rospy.Subscriber('chatter1', String, messengerCallback)
    rospy.spin()

if __name__ == '__main__':
    messenger()
