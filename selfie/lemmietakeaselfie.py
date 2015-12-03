#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from struct import unpack
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from std_msgs.msg import String
import sys
import math
import time
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

def data_init():
	global pastData, nowFollowBlob, nowTakeSelfie
	nowFollowBlob = False
	nowTakeSelfie = False
	pastData = []

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
	global blobData, isBlobReady, blobTime
	blobData = data
	blobTime = time.clock()
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

		curr_blobweights = [x, y, area] # Boom.
	else:
		curr_blobweights = [area, area, area]
	has_new_blobinfo = True

def handleBlobs(): #this still needs to check to see if the picture card exists, if it does then we just call the selfie funtion
	global curr_blobweights, nowFollowBlob
	if not curr_blobweights[0][2] == -1 and not curr_blobweights[0][3] == -1:
		# check if the blue and green are nearby relative to their areas
		xdif = curr_blobweights[0][3] - curr_blobweights[0][2]
		ydif = curr_blobweights[1][3] - curr_blobweights[1][2]
		distsq = xdif*xdif + ydif*ydif
		if distsq < curr_blobweights[2][3] * 4:
			# We have someone to follow
			followPoint = (curr_blobweights[0][3] - xdif/2.0, curr_blobweights[1][3] - ydif/2.0)
			nowFollowBlob = True
	if not curr_blobweights[0][0] == -1 and not curr_blobweights[0][1] == -1: # we think the camera card exists, orange and pink are there (pink inside orange)
		xdif = curr_blobweights[0][0] - curr_blobweights[0][1]
		ydif = curr_blobweights[1][0] - curr_blobweights[1][1]
		dist = sqrt(xdif**2 + ydif**2)
		if dist < curr_blobweights[2][1]: #the center of the two is inside of the pink area
			#we have a camera card?
			nowTakeSelfie = True


def handleDistance(): 	# If it exists, this function collects Kinect's distance at followPoint and handles
			# the twist response and verbal response to seeing different environmental factors,
			# recording followPoint->Kinect data in pastData and looking at the kinect depth field
			# to figure out if we should panic and stop
			# XXX: Move code from randmov for this, maybe? probably? ST: yes, it will handle the too close too far problem, it won't need the turn away stuff. Otherwise, if anything is less than 1m away, just stop

def selfie(image):
	path = "./pictures"
	for (path, dirs, files) in os.walk(path):
		pass
	filenum = str(int(files[-1][-9:-4]) + 1) #last file number + 1
	filename = "pict" + filenum.zfill(5) + ".jpg"
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
	global curr_velocity, blobData, blobTime, isBlobReady, isColorImageReady, blobCopy, timeCopy
	global nowFollowBlob, nowTakeSelfie
	twist_init()
	data_init()
	rospy.init_node('selfie_stalker', anonymous = True) # Initialize this node
	rospy.Subscriber('/blobs', Blobs, blobsCallback)
	rospy.Subscriber("/camera/depth/image", Image, depthCallback, queue_size=10)
	rospy.Subscriber("/camera/rgb/image_color", Image, updateColorImage, queue_size=10)
	bridge = CvBridge()

	while not isDepthReady and not rospy.is_shutdown() and not isBlobReady and not isColorImageReady:
		pass

	while not rospy.is_shutdown():
		try:
			color_image = bridge.imgmsg_to_cv2(colorImage, "bgr8")
		except CvBridgeError, e:
			print e

		#check blobs
		while not isBlobReady: # XXX: Fast loop. Slow it down?
			pass
		blobCopy = copy.deepcopy(blobData) #copy of blobs so it doesn't change
		timeCopy = blobTime
		parseBlobs(blobCopy)
		handleBlobs()
		isBlobReady = False # Finished processing this batch

		if nowFollowBlob and not nowTakeSelfie:
			pub.publish(curr_velocity)
			#move forward, keeping the blob centered, adjusting speed with distance
			#check for obstacles
			nowFollowBlob = False
		#if picture blob
			#take picture
			selfie(color_image)




if __name__ == '__main__':
	main()
