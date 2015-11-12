#!/usr/bin/env python


import rospy
from cmvision.msg import Blobs, Blob
from geometry_msgs.msg import Twist

#we need not use constant command, assuming that the poling of the images is fast enough
pub = rospy.Publisher('kobuki_command', Twist, queue_size=10)
pub2 = rospy.Publisher('pid_input', (String, float, float, float), queue_size=10)
K_P = 1.5
K_D = 1.25
channel = 'turning'

def blobsCallback(data): #this is called whenever a blob message is posted, blob messages are posted even if blobs are not detected
	x = 0
	y = 0
	area = 0
	# bloblist = []
	if data.blob_count > 0: #we have a blob, track it
		for box in data.blobs:
			area += box.area
			y += box.y * box.area
			x += box.x * box.area
		x = x / area
		y = y / area
		
		blobloc = (320.0 - x)/320.0
		pub2.publish('turning', blobloc, K_P, K_D)
	else: #stay still
		pub2.publish('turning', 0, K_P, K_D)

def turnCallback(data):
	if not(data[0] == channel):
		return
	global curr_velocity
	curr_velocity.angular.z = data[1]
	pub.publish(curr_velocity)

def twist_init():
	global curr_velocity
	curr_velocity = Twist()
	curr_velocity.linear.x, curr_velocity.linear.y, curr_velocity.linear.z = 0, 0, 0
	curr_velocity.angular.x, curr_velocity.angular.y, curr_velocity.angular.z = 0, 0, 0

def detect_blob():
	
	twist_init()
	rospy.init_node('blob_tracker', anonymous = True)
	rospy.Subscriber('/blobs', Blobs, blobsCallback)
	rospy.Subscriber('pid_command', (String, float), turnCallback)
	rospy.spin()

if __name__ == '__main__':
	detect_blob()
