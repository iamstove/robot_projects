#!/usr/bin/env python

#use newline with this
import rospy
from cmvision.msg import Blobs, Blob
from geometry_msgs.msg import Twist

#we need not use constant command, assuming that the poling of the images is fast enough
pub = rospy.Publisher('kobuki_command', Twist, queue_size=10)

K_P = 1.5
K_D = 1.25

def blobsCallback(data): #this is called whenever a blob message is posted, blob messages are posted even if blobs are not detected
	global curr_velocity
	global pastloc
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
		'''index = len(bloblist)
		bloblist.append(box)
		while index > 0 and bloblist[(index-1)/2].area < bloblist[index].area:
		bloblist[(index-1)/2], bloblist[index] = bloblist[index], bloblist[(index-1)/2]
		index = (index-1)/2'''
		
		blobloc = (320.0 - x)/320.0
		#print(blobloc)
		curr_velocity.angular.z = K_P * blobloc + K_D * (blobloc - pastloc)
		#print(curr_velocity.angular.z)
		curr_velocity.linear.x = .2
		pastloc = blobloc
		#print("going!!")
		pub.publish(curr_velocity)
	else: #stay still
		#print("Staying!!")
		twist_init()
		pub.publish(curr_velocity)

def twist_init():
	global curr_velocity
	global pastloc
	pastloc = 0
	curr_velocity = Twist()
	curr_velocity.linear.x, curr_velocity.linear.y, curr_velocity.linear.z = 0, 0, 0
	curr_velocity.angular.x, curr_velocity.angular.y, curr_velocity.angular.z = 0, 0, 0

def detect_blob():
	
	twist_init()
	rospy.init_node('blob_tracker', anonymous = True)
	rospy.Subscriber('/blobs', Blobs, blobsCallback)
	rospy.spin()

if __name__ == '__main__':
	detect_blob()
