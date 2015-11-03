#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from struct import unpack
import sys

depthData = Image();
isDepthReady = False;

def depthCallback(data):
	global depthImage, isDepthImageReady
	depthImage = data
	isDepthImageReady = True

def main():
	global depthData, isDepthReady
	rospy.init_node('depth_example', anonymous=True)
	rospy.Subscriber("/camera/depth/image", Image, depthCallback, queue_size=10)

	while not isDepthReady and not rospy.is_shutdown():
		pass

	while not rospy.is_shutdown():
		step = depthData.step
		depthValue = []
		tot = 0
		i = 0
		mid_height = 240
		for pixel in range(0, 640, 20):
			sys.stderr.write(str(i) + "\n")
			offset = (mid_height * step) + (pixel * 4)
			(depthValue[i],) = unpack('f', depthData.data[offset] + depthData.data[offset+1] + depthData.data[offset+2] + depthData.data[offset+3])
			tot += depthValue[i]
			sys.stderr.write("Distance: " + str(depthValue[i]) + "\n")
			i += 1

		tot /= 32
		sys.stderr.write("Distance: " + str(depthValue) + "\n")
		sys.stderr.write("Avg: " + str(tot) + "\n")

		for value in depthValue:
			if math.isnan(value):
				#stop and scan up
				continue
			else: #we have a real number, we want to see if its less than 1
				if value < 1:
					#stop and turn until we don't see it anymore
					continue
				else:
					#keep moving
					continue



if __name__ == '__main__':
	main()
