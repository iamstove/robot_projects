#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from struct import unpack
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from std_msgs.msg import String
import sys
import math

depthData = Image();
isDepthReady = False;
keepMove = True
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

def scanup(column):
	"""returns false for too far and returns true for too close"""
	colarr = []
	for pixel in range(480,240,-1): #spotcheck up every 15px, compile into array, (this will be 16 in length)
		offset = (pixel * depthData.step) + (column * 4)
		(val,) = unpack('f', depthData.data[offset] + depthData.data[offset+1] + depthData.data[offset+2] + depthData.data[offset+3])
		colarr.append(val)

	for d in range(240):
		if (d+1) = len(colarr): #check if our step will put us out of bounds
			break
		else:
			if colarr[d] > 5 and math.isnan(colarr[d+1]): #this would be the case where our nan is far away
				return False
			else if colarr[d] < 1.5 and math.isnan(colarr[d+1]): #the case when we should stop and turn
				return True

def turn_away(loc):
	"""Turns until we no longer see anything"""
	global curr_velocity
	twist_init() #make sure everything is zero, so we don't turn and move
	if loc < 10:
		curr_velocity.angular.z = .25
	else:
		curr_velocity.angular.z = -.25
	object_found = True
	while object_found:
		truth_arr = []
		#pub.publish(curr_velocity) #hand off velocity to constant command
		print(str(curr_velocity))
		horzArr = []
		for pixel in range(0, 640, 20): #build an array of values across the center of the screen (20px width)
			#sys.stderr.write(str(i) + "\n")
			offset = (mid_height * step) + (pixel * 4)
			#sys.stderr.write(str(offset)+"\n")
			(val,) = unpack('f', depthData.data[offset] + depthData.data[offset+1] + depthData.data[offset+2] + depthData.data[offset+3])
			horzArr.append(val)

		i = 0
		for val in horzArr:
			if math.isnan(val):
				truth_arr.append(scanup(i))
			else:
				if val <= 1:
					truth_arr.append(True)
				else:
					truth_arr.append(False)
			i += 20

		object_found = truth_test(truth_arr)

	curr_velocity.angular.z = 0
	#pub.publish(curr_velocity) #stop turning
	print(str(curr_velocity))
	return False #now we know it's no longer turning


def truth_test(t_arr):
	"""takes an array of booleans and if one of them is true, that means the object is still in sight, else it rturns false and we can stop turning and start moving again"""
	for item in t_arr:
		if item = True:
			return True
		else: continue
	return False

def main():
	twist_init()
	global depthData, isDepthReady, curr_velocity
	rospy.init_node('depth_example', anonymous=True)
	rospy.Subscriber("/camera/depth/image", Image, depthCallback, queue_size=10)
	#sys.stderr.write("hello1\n")


	while not isDepthReady and not rospy.is_shutdown():
		pass

	while not rospy.is_shutdown() and keepMove:
		step = depthData.step
		#sys.stderr.write("step: " +str(step)+ "\n")
		horzArr = []
		#tot = 0
		for pixel in range(0, 640, 20): #build an array of values across the center of the screen (20px width)
			#sys.stderr.write(str(i) + "\n")
			offset = (mid_height * step) + (pixel * 4)
			#sys.stderr.write(str(offset)+"\n")
			(val,) = unpack('f', depthData.data[offset] + depthData.data[offset+1] + depthData.data[offset+2] + depthData.data[offset+3])
			horzArr.append(val)
			#tot += val
			#sys.stderr.write("Distance: " + str(depthValue[i]) + "\n")

		#tot /= 32
		#sys.stderr.write("Distance: " + str(depthValue) + "\n")
		#sys.stderr.write("Avg: " + str(tot) + "\n")

		i = 0 #contains which column we're on
		for value in horzArr:
			if math.isnan(value): #check our spotchecks for nans
				scanup(i)
			else: #we have a real number, we want to see if its less than 1
				if value < 1:
					#stop and turn until we don't see it anymore
					still_turning = True
					still_turning = turn_away(i)
					if still_turning = True: #wait until it's no longer turning
						print "Massssssive error"
				else:
					#keep moving
					curr_velocity.linear.x = .25
					#pub.publish(curr_velocity)
					print(str(curr_velocity))

			i += 20



if __name__ == '__main__':
	main()
