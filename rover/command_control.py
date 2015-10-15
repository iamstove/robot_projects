#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from kobuki_msgs.msg import BumperEvent


pub = rospy.Publisher('keyboard_command', String, queue_size=10)
pub2 = rospy.Publisher('impact', String, queue_size=10)

def bumperCallback(data):
    global pub2
    print ("IMPACT ISSUE NEW COMMAND")
    while pub2.get_num_connections() == 0: #if we don't have someone to listen to, don't say anything
        pass
    
    pub2.publish("We crashed")

def send_commands():
    global pub
    rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, bumperCallback)
    rospy.init_node('command_control', anonymous=True)
    while pub.get_num_connections() == 0:
        #print "passing"
        pass

    while True:
        print("Please enter a command: ") #replace this with the input splitting script
    	in_com = raw_input()
    	pub.publish(in_com)

if __name__ == '__main__':
    try:
        send_commands()
    except rospy.ROSInterruptException:
        pass
