#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent

curr_velocity = Twist()
sleep_time = (1/200.0)
pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size = 10)

def command(data):
	global curr_velocity
	#rospy.loginfo("Twist recieved" + str(data))
	curr_velocity = data

def bumperCallback(data):
	global not_bumping
	not_bumping = False # We have crashed.
	

def node_init():
	global curr_velocity
	global not_bumping
	not_bumping = True
	curr_velocity.linear.x = 0.0
	curr_velocity.linear.y = 0.0
	curr_velocity.linear.z = 0.0
	curr_velocity.angular.x = 0.0
	curr_velocity.angular.y = 0.0
	curr_velocity.angular.z = 0.0

def constant():
	global curr_velocity
	global sleep_time
	global not_bumping
	
	rospy.init_node('constant_command', anonymous=True)
	node_init()
	rospy.Subscriber('kobuki_command', Twist, command)
	rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, bumperCallback)

	while not_bumping:
		#rospy.loginfo("Twist sent" + str(curr_velocity))
		pub.publish(curr_velocity)
		rospy.sleep(sleep_time)
	

if __name__ == '__main__':
	try:
		constant()
	except rospy.ROSInterruptException:
		pass
