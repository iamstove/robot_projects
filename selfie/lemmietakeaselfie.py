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
color_namelist = ['Orange', 'Pink', 'Green', 'Blue'] # We'll use this for indexing different colors.

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

def parseBlobs(data)
	global curr_blobweights
	global has_new_blobinfo
	global fd
	global x, y
	x = [-1, -1, -1, -1] # Orange, Pink, Green, Blue
	y = [-1, -1, -1, -1] # could be generalized but is ok for now
	area = [-1, -1, -1, -1]
	if data.blob_count > 0: # We have a blob / some blobs, track them
		for box in data.blobs:
			if box.name in color_namelist:
				color_index = color_namelist.index(box.name)
				if area[color_index] == -1:
					area[color_index] = box.area
				else:
					area[color_index] += box.area
				y[color_index] += box.y * box.area
				x[color_index] += box.x * box.area
			else:
				fd.write("Unidentified or irrelevant color: " + box.name + "\n")

		for color_index in range(len(color_namelist)): # Divide by the total weight to find the center position
			if area[color_index] == -1:
				x[color_index] = -1
				y[color_index] = -1
			else:
				x[color_index] /= area[color_index]
				y[color_index] /= area[color_index]

		curr_blobweights = [x, y] # Boom.
	else:
		curr_blobweights = [area, area]
	has_new_blobinfo = True

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
		blobsCopy = copy.deepcopy(blobsData) #copy of blobs so it doesn't change
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
