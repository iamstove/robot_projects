#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from struct import unpack
import sys
import math

depthData = Image();
isDepthReady = False;
keepMove = True

def depthCallback(data):
	global depthData, isDepthReady
	depthData = data
	isDepthReady = True

def scanup(column):
	colarr = []
	for pixel in range(480,240,-15): #spotcheck up every 15px, compile into array, (this will be 16 in length)
		offset = (pixel * depthData.step) + (column * 4)
		(val,) = unpack('f', depthData.data[offset] + depthData.data[offset+1] + depthData.data[offset+2] + depthData.data[offset+3])
		colarr.append(val)

	for d in range(16):
		if d+1 = len(colarr): #check if our step will put us out of bounds
			break
		else:
			if colarr[d] > 5 and math.isnan(colarr[d+1]): #this would be the case where our nan is far away
				 continue
			else if colarr[d] < 1 and math.isnan(colarr[d+1]): #the case when we should stop and turn
				continue



def main():
	global depthData, isDepthReady
	rospy.init_node('depth_example', anonymous=True)
	rospy.Subscriber("/camera/depth/image", Image, depthCallback, queue_size=10)
	#sys.stderr.write("hello1\n")


	while not isDepthReady and not rospy.is_shutdown():
		pass

	while not rospy.is_shutdown() and keepMove:
		step = depthData.step
		sys.stderr.write("step: " +str(step)+ "\n")
		depthValue = []
		tot = 0
		for pixel in range(0, 640, 20): #build an array of values across the center of the screen (20px width)
			#sys.stderr.write(str(i) + "\n")
			offset = (mid_height * step) + (pixel * 4)
			#sys.stderr.write(str(offset)+"\n")
			(val,) = unpack('f', depthData.data[offset] + depthData.data[offset+1] + depthData.data[offset+2] + depthData.data[offset+3])
			depthValue.append(val)
			tot += val
			#sys.stderr.write("Distance: " + str(depthValue[i]) + "\n")

		tot /= 32
		#sys.stderr.write("Distance: " + str(depthValue) + "\n")
		#ssys.stderr.write("Avg: " + str(tot) + "\n")

		i = 0 #contains which column we're on
		for value in depthValue:
			if math.isnan(value): #check our spotchecks for nans
				scanup(i)
				continue
			else: #we have a real number, we want to see if its less than 1
				if value < 1:
					#stop and turn until we don't see it anymore
					continue
				else:
					#keep moving
					continue
			i += 20



if __name__ == '__main__':
	main()
