#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

curr_velocity = Twist()
sleep_time = (1/500.0)
pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size = 10)


def command(data):
    global curr_velocity
    #rospy.loginfo("Twist recieved" + str(data))
    curr_velocity = data
    

def twist_init():
    global curr_velocity
    curr_velocity.linear.x = 0.0
    curr_velocity.linear.y = 0.0
    curr_velocity.linear.z = 0.0
    curr_velocity.angular.x = 0.0
    curr_velocity.angular.y = 0.0
    curr_velocity.angular.z = 0.0

def constant():
    global curr_velocity
    global sleep_time
    rospy.init_node('constant_command', anonymous=True)
    twist_init()
    rospy.Subscriber('kobuki_command', Twist, command)

    while True:
        #rospy.loginfo("Twist sent" + str(curr_velocity))
        pub.publish(curr_velocity)
        rospy.sleep(sleep_time)
    

if __name__ == '__main__':
    try:
        constant()
    except rospy.ROSInterruptException:
        pass
