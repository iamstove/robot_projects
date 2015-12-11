#!/usr/bin/env python

import rospy
from graphics import *
from sensor_msgs.msg import Image
from struct import unpack
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion
import sys
import math
import random
import time
import cv2
from cv_bridge import CvBridge, CvBridgeError
from cmvision.msg import Blobs, Blob
import copy
from nav_msgs.msg import Odometry

colorImage = Image()
blobsData = Blobs()
depthImage = Image()
pub = rospy.Publisher('kobuki_command', Twist, queue_size=10)
color_namelist = ['Green', 'Blue', 'Orange', 'Pink'] # We'll use this for indexing different colors.




def data_init():
	global pastData, nowFollowBlob, nowTakeSelfie, depthAverage, dirAverage, isDepthReady, isBlobReady, isColorImageReady
	global followPoint
	global spacePoints
	global del_x, del_r, isOdomReady
	del_x = (0, 0)
	del_r = 0
	spacePoints = []
	isOdomReady = False
	nowFollowBlob = False
	nowTakeSelfie = False
	isDepthReady = False
	isBlobReady = False
	isColorImageReady = False
	depthAverage = 0.0
	dirAverage = 0.0
	followPoint = (320, 240)
	pastData = [(time.clock(), 0.0, followPoint)]

def odomCallback(data):
	global del_x, del_r, isOdomReady
	q = [data.pose.pose.orientation.x,
	data.pose.pose.orientation.y,
	data.pose.pose.orientation.z,
	data.pose.pose.orientation.w]

	roll, pitch, yaw = euler_from_quaternion(q)

	del_x = (data.pose.pose.position.x, data.pose.pose.position.y)
	del_r = yaw

	isOdomReady = True

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

def scanMid():
	'''scans across the center of an image, and returns a list of depths'''
	if isDepthReady:
		depthCopy = depthData
		midstep = 10
		horzArr = []
		for pixel in range(0, 640, midstep): #build an array of values across the center of the screen (midstep px apart)
			offset = (240 * depthData.step) + (pixel * 4)
			(val,) = unpack('f', depthCopy.data[offset] + depthCopy.data[offset+1] + depthCopy.data[offset+2] + depthCopy.data[offset+3])
			horzArr.append((pixel,val))

		handleMiddle(horzArr)
	else:
		pass


def scanFront():
	'''Returns: 	The number of nans in front'''

	global depthData
	# Assert that we have depthData and it's new / current

	gridStep = 10

	nans = 0

	# Ranges should be (0, 640), (400, 480)
	for x in range(0, 640, gridStep):
		for y in range(400, 480, gridStep):
			# Check the depth of (x, y)
			offset = (y * depthData.step) + (x * 4)
			(val,) = unpack('f', depthData.data[offset] + depthData.data[offset+1] + depthData.data[offset+2] + depthData.data[offset+3])
			if math.isnan(val):
				nans = nans + 1
	return nans

def parseBlobs(data):
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

def handleMiddle(middleList):
	global spacePoints
	mapList = []
	for point in middleList:
		if not math.isnan(point[1]):
			theta = math.atan((320.0 - point[0]) / 320.0 * math.tan(0.5))
			thetap = theta + del_r
			dvec_x = (math.cos(thetap) * point[1], math.sin(thetap) * point[1])
			vec_x = (dvec_x[0] + del_x[0], dvec_x[1] + del_x[1])
			mapList.append(vec_x)
			mapPoints(mapList)
	spacePoints += mapList

def mapPoints(mapList):
	global win
	for point in mapList:
		win.plot(int(point[0]*100), int(point[1]*100))

def handleBlobs(): #this still needs to check to see if the picture card exists, if it does then we just call the selfie funtion
	global curr_blobweights, nowFollowBlob, followPoint
	if not curr_blobweights[0][2] == -1 and not curr_blobweights[0][3] == -1:
		# check if the blue and green are nearby relative to their areas
		xdif = curr_blobweights[0][3] - curr_blobweights[0][2]
		ydif = curr_blobweights[1][3] - curr_blobweights[1][2]
		distsq = xdif*xdif + ydif*ydif
		if distsq < curr_blobweights[2][3] * 256:
			# We have someone to follow
			followPoint = (curr_blobweights[0][3] - xdif/2.0, curr_blobweights[1][3] - ydif/2.0)
			nowFollowBlob = True
	if not curr_blobweights[0][0] == -1 and not curr_blobweights[0][1] == -1: # we think the camera card exists, orange and pink are there (pink inside orange)
		xdif = curr_blobweights[0][0] - curr_blobweights[0][1]
		ydif = curr_blobweights[1][0] - curr_blobweights[1][1]
		distsq = xdif*xdif + ydif*ydif
		if distsq < curr_blobweights[2][1]: #the center of the two is inside of the pink area
			#we have a camera card?
			nowTakeSelfie = True


def handleMovement(): 	# If it exists, this function collects Kinect's distance at followPoint and handles
			# the twist response and verbal response to seeing different environmental factors,
			# recording followPoint->Kinect data in pastData and looking at the kinect depth field
			# to figure out if we should panic and stop
			# XXX: Move code from randmov for this, maybe? probably? ST: yes, it will handle the too close too far problem, it won't need the turn away stuff. Otherwise, if anything is less than 1m away, just stop
	global depthData, followPoint, nowFollowBlob, timeCopy, blobCopy, depthAverage, dirAverage, curr_velocity
	global hasObstacle, waitUp, woahThere, pastData

	hasObstacle = scanFront() > 512

	numPointsAveraged = 30
	dee = 1.3

	if nowFollowBlob and not hasObstacle:
		# Get the depth of the followPoint
		(x, y) = (int(followPoint[0]), int(followPoint[1]))
		offset = (y * depthData.step) + (x * 4)
		(followDepth,) = unpack('f', depthData.data[offset] + depthData.data[offset+1] + depthData.data[offset+2] + depthData.data[offset+3])

		# Append timeCopy and followDepth, modified:

		# If we are nan now,
		if math.isnan(followDepth):
			if pastData[-1][1] > 5.0:
				pastData.append((timeCopy, 7.0, followPoint))
				waitUp = True
			elif pastData[-1][1] < 0.5:
				pastData.append((timeCopy, 0.0, followPoint))
				woahThere = True
			else:
				pastData.append((timeCopy, dee, followPoint)) #arbitrary
		else:
			pastData.append((timeCopy, followDepth, followPoint))
		# Append to the pastData array (timeCopy, depthFollowPoint): set depthFollowPoint to far or close if it's nan

		# set pastTurn and compare to currTurn
	else: # else we have no followPoint or have encountered an obstacle, so append a nothing.
		pastData.append((timeCopy, dee, (320, 240)))

	if len(pastData) <= numPointsAveraged + 1:
		depthAverage += pastData[-1][1] / float(numPointsAveraged)
		dirAverage += pastData[-1][2][0] / (640.0 * numPointsAveraged)
	else: # we have numPointsAveraged points.  Also remove the first when you add the last.
		depthAverage += (pastData[-1][1] - pastData[-1 - numPointsAveraged][1]) / float(numPointsAveraged)
		dirAverage += (pastData[-1][2][0] - pastData[-1 - numPointsAveraged][2][0]) / (640.0 * numPointsAveraged)

	curr_velocity.linear.x = (depthAverage - dee)/2
	curr_velocity.angular.z = -(dirAverage - 0.5)*2

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
	global nowFollowBlob, nowTakeSelfie, followPoint, depthAverage, dirAverage
	global win
	count = 0
	win = GraphWin('title', 500, 500)
	twist_init()
	data_init()
	rospy.init_node('depth_stalker', anonymous = True) # Initialize this node
	rospy.Subscriber('/odom', Odometry, odomCallback)
	rospy.Subscriber('/blobs', Blobs, blobsCallback)
	rospy.Subscriber("/camera/depth/image", Image, depthCallback, queue_size=10)
	rospy.Subscriber("/camera/rgb/image_color", Image, updateColorImage, queue_size=10)
	bridge = CvBridge()

	while (not isDepthReady or not isBlobReady or not isColorImageReady or not isOdomReady) and not rospy.is_shutdown():
		pass

	while not rospy.is_shutdown():
		try:
			color_image = bridge.imgmsg_to_cv2(colorImage, "bgr8")
		except CvBridgeError, e:
			print e
		
		#if count % 100 == 0:
		#	scanMid()
		
		#check blobs
		while not isBlobReady: # XXX: Fast loop. Slow it down?
			pass
		blobCopy = copy.deepcopy(blobData) #copy of blobs so it doesn't change
		timeCopy = blobTime
		parseBlobs(blobCopy)
		handleBlobs()
		isBlobReady = False # Finished processing this batch
		handleMovement()
		count = count + 1
		if count % 2 == 0:
			sys.stderr.write("nFB:" + str(nowFollowBlob)[0] +
					 " fP:" + str(followPoint) +
					 " c_vX:" + str(curr_velocity.linear.x) +
					 " c_vZ:" + str(curr_velocity.angular.z) +
					 " hOb:" + str(hasObstacle) +
					 " dpAv:" + str(depthAverage) +
					 " drAv:" + str(dirAverage) +
					 "\n")
		if nowFollowBlob and not nowTakeSelfie:
			pub.publish(curr_velocity)
			#move forward, keeping the blob centered, adjusting speed with distance
			#check for obstacles
			nowFollowBlob = False
		elif nowTakeSelfie:
			#take picture
			twist_init()
			pub.publish(curr_velocity)
			selfie(color_image)




if __name__ == '__main__':
	main()
