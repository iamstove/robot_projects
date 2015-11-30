#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from struct import unpack
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from std_msgs.msg import String
import sys
import math
import cv2
from cv_bridge import CvBridge, CvBridgeError
from cmvision.msg import Blobs, Blob
import copy

isDepthReady = False
isBlobReady = True
colorImage = Image()
isColorImageReady = False
keepMove = True
blobsData = Blobs()
depthImage = Image()
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

def updateColorImage(data):
	global colorImage, isColorImageReady
	colorImage = data
	isColorImageReady = True

def selfie(image):
	path = "./pictures"
	for (path, dirs, files) in os.walk(path):
		pass
	filenum = int(files[-1][-6:-4]) + 1 #last file number + 1
	if filenum < 10:
		filename = "pict0" + filenum + ".jpg"
	elif filenum < 100:
		filename = "pict" + filenum + ".jpg"
	else: #100 or greater
		print("pictures full")
	os.system('spd-say \"3\"')
	rospy.sleep(1)
	os.system('spd-say \"2\"')
	rospy.sleep(1)
	os.system('spd-say \"1\"')
	rospy.sleep(1)
	os.system('spd-say \"Smile\"')
	rospy.sleep(.25)
	cv2.imwrite(filename, color_image, [cv2.IMWRITE_JPEG_QUALITY, 100])

def main():
	twist_init()
	rospy.init_node('selife_stalker', anonymous = True) # Initialize this node
	rospy.Subscriber('/blobs', Blobs, blobsCallback)
	rospy.Subscriber("/camera/depth/image", Image, depthCallback, queue_size=10)
	bridge = CvBridge()

	while not isDepthReady and not rospy.is_shutdown() and not isBlobReady and not isColorImageReady:
		pass

	while not rospy.is_shutdown():
		try:
			color_image = bridge.imgmsg_to_cv2(colorImage, "bgr8")
		except CvBridgeError, e:
			print e
		#check blobs
		#if follow blob
			#move forward, keeping the blob centered, adjusting speed with distance
			#check for obsticles
		#if picture blob
			#take picture
			selfie(color_image)




if __name__ == '__main__':
	main()
