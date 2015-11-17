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

depthData = Image()
dists = []
isDepthReady = False
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

def appendValue():
	global isCheckReady
	global depthData
	
	offset = (240 * depthData.step) + (320 * 4)
	currtime = time.clock()
	(val,) = unpack('f', depthData.data[240 * depthData.step] + depthData.data[offset+1] + depthData.data[offset+2] + depthData.data[offset+3])
	dists.append((currtime, val))
	
	isCheckReady = (len(dists) > 10)

def main():
	global depthData, isDepthReady, isCheckReady, curr_velocity
	isCheckReady = False
	value = 0
	twist_init()
	rospy.init_node('depth_example', anonymous=True)
	rospy.Subscriber("/camera/depth/image", Image, depthCallback, queue_size=10)
	
	while not isDepthReady and not rospy.is_shutdown():
		pass

	while not isCheckReady and not rospy.is_shutdown():
		appendValue()
		isDepthReady = False	

	average = 0
	
	for i in range(1, 11):
		if not math.isnan(dists[-i][1]):
			average += dists[-i][1]

	while not rospy.is_shutdown():
		while not isDepthReady:
			#rospy.sleep(0.003)
			pass

		appendValue()
		isDepthReady = False	
	
		if not math.isnan(dists[-1][1]): 
			value = value * (math.exp((-dists[-1][0] + dists[-2][0]) * 10))
			value += dists[-1][1] * (dists[-1][0] - dists[-2][0]) * 10
			if not math.isnan(dists[-10][1]):
				average += dists[-1][1] - dists[-10][1]
			else:
				average += dists[-1][1]
		else:
			if not math.isnan(dists[-10][1]):
				average -= dists[-10][1]
			value = value * math.exp(- dists[-1][0] + dists[-2][0])
		
		valprint = int(value * 9.0)
		avgprint = int(average)
		output = ''
		if valprint > avgprint:
			output = ('#' * avgprint) + ('*' * (valprint - avgprint))
		elif valprint == avgprint:	
			output = ('#' * avgprint)
		else:
			output = ('*' * valprint) + (' ' * (avgprint - valprint - 1)) + '#'
		 
	
		# print("Val: ", value, " Sumval: ", average / 10.0)
		
		print(output)


if __name__ == '__main__':
	main()
