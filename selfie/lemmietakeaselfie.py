#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from struct import unpack
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from std_msgs.msg import String
import sys
import math

isDepthReady = False
isBlobReady = True
keepMove = True
mid_height = 240
pub = rospy.Publisher('kobuki_command', Twist, queue_size=10)

def twist_init():
	global curr_velocity
	curr_velocity = Twist()
	curr_velocity.linear.x = 0.0
	curr_velocity.linear.y = 0.0
	curr_velocity.linear.z = 0.0
	curr_velocity.angular.x = 0.0
	curr_velocity.angular.y = 0.0
	curr_velocity.angular.z = 0.0

def depthCallback(data):
	"""updates depthData"""
	global depthData, isDepthReady
	depthData = data
	isDepthReady = True

def blobsCallback(data):
	"""updates the blobs"""
	global blobData, isBlobReady
	blobData = data
	isBlobReady = True

def main():

	twist_init()
	rospy.init_node('selife_stalker', anonymous = True) # Initialize this node
	rospy.Subscriber('/blobs', Blobs, blobsCallback)
	rospy.Subscriber("/camera/depth/image", Image, depthCallback, queue_size=10)
	while not isDepthReady and not rospy.is_shutdown() and not isBlobReady:
		pass

	while not rospy.is_shutdown():
		#check blobs
		#if follow blob
			#move forward, keeping the blob centered, adjusting speed with distance
			#check for obsticles
		#if picture blob
			#take picture




if __name__ == '__main__':
	main()
