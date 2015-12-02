#!/usr/bin/env python


import rospy
import time
from std_msgs.msg import String

#we need not use constant command, assuming that the poling of the images is fast enough
pub = rospy.Publisher('pid_command', String, queue_size=10)

pids = {} # should contain ([(time, value)], K_P and K_Ds)s.

K_P_DEFAULT = 1.
K_D_DEFAULT = 1.

def pidCallback(data): 	# data is a CSV string 
			# data should be of the form "channel,value[,k_p,k_d]"
	currtime = time.clock()
	dsplit = data.data.split(",")
	data = [dsplit[0]] + map(float, dsplit[1:])
	channel = data[0]
	value = 0
	if not(channel in pids):
		# add it to the pid controller list
		if len(data) < 4:
			# No k_p or k_d values, so we use defaults
			pids[channel] = ([(currtime, data[1])], K_P_DEFAULT, K_D_DEFAULT)
		else:
			pids[channel] = ([(currtime, data[1])], float(data[2]), float(data[3]))
		value = pids[channel][1] * data[1]
		
	else:
		# Else we're updating and adding to the list, then reporting back.  
		# Multiple implementations come to mind for reporting back a float value as
		## pid control gives it; we implement it as such:
		## Reports are triggered immediately after information is received.
		## The process involved here simply uses the current value and it compared to the previous one.
		## other implementations include adding the convolution between the function given and
		## e^-kt, which would place more importance on recent values but still weight all values, and would be
		## a fast implementation.  Alternatively, reporting back on a consistent basis as requested by 
		## the tracking requester, based on a taylor expansion that uses more terms the further back the last
		## packet of information was received.
		
		numpoints = len(pids[channel][0])
		curr = data[1]
		(past, pastime) = pids[channel][0][numpoints-1]
		(k_p, k_d) = pids[channel][1], pids[channel][2]
		pids[channel][0].append((currtime, curr))
		
		# if currtime == pastime: # Well ok then
			# scream bloody mary
			
		value = k_p * curr + k_d * (curr - past) / (currtime - pastime)
	pub.publish(channel + "," + str(value))
	
def pid_operate():
	rospy.init_node('pid_tracker', anonymous = True)
	rospy.Subscriber('pid_input', String, pidCallback)
	rospy.spin()

if __name__ == '__main__':
	pid_operate()
