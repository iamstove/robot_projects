#!/usr/bin/env python

#Imports#
import rospy
from geometry_msgs.msg import Twist
from cmvision.msg import Blobs, Blob
import time #itself!  WAUUUGHAHAHAHAHHAAAAA

#Globals#
pub = rospy.Publisher('kobuki_command', Twist, queue_size=10) # Command publisher

K_P = 1.5 # K_p in the PID equation
K_D = 1.25 # K_d in the PID equation

color_namelist = ['Greenline', 'Redball,' 'Orangegoal'] # We'll use this for indexing different colors.

def blobsCallback(data): # This is called whenever a blobs message is posted; this happens constnatly even if blobs are not detected
	global curr_blobweights 
	global has_new_blobinfo
	x = [0, 0, 0] # Greenline, Redball, Orangegoal
	y = [0, 0, 0]
	area = [-1, -1, -1]
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
				# Unidentified or irrelevant color.  Ignore it / do nothing.

		for color_index in range(color_name.length()): # Divide by the total weight to find the center position
			x[color_index] /= area[color_index]
			y[color_index] /= area[color_index]
			
		curr_blobweights = area # Boom.
		has_new_blobinfo = True


		#print(blobloc)
		curr_velocity.angular.z = K_P * blobloc + K_D * (blobloc - pastloc)
		#print(curr_velocity.angular.z)
		curr_velocity.linear.x = .2
		pastloc = blobloc
		#print("going!!")
		pub.publish(curr_velocity)
	else: #stay still
		#print("Staying!!")
		twist_init()
		pub.publish(curr_velocity)

def init_all():
	twist_init()
	global has_new_blobinfo
	
	has_new_blobinfo = False
	
def twist_init():
	global curr_velocity
	global pastloc
	pastloc = 0
	curr_velocity = Twist()
	curr_velocity.linear.x, curr_velocity.linear.y, curr_velocity.linear.z = 0, 0, 0
	curr_velocity.angular.x, curr_velocity.angular.y, curr_velocity.angular.z = 0, 0, 0
\
def follow_the_line():
	# We first want to declare our dependency upon the global blob area.
	global curr_blobweights # Note well that this is effectively all blobsCallback changes when it runs.
	# We will simulate nodelike behavior by looping until a flag has_new_blobinfo is set.
	global has_new_blobinfo
	### This is similar to nodes in that it's basically waiting for a publish trigger to come down
	### and signal this 'node' to get to work.
	
	# We will have a trigger that will mean we're finished with the line; this trigger requires we have not seen
	### a green blob for 10 new sets of blobs.  This trigger activates with changes in not_done_with_line
	not_done_with_line = True

	# The loop will incorporate a wait that waits 1/10 the time we waited to get the blob the last time.
	### This is so the program doesn't consume huge CPU.
	last_time_waited = 0 # We don't have to wait at all to try the first time.
	
	while not_done_with_line:
		while not(has_new_blobinfo):
			rospy.sleep(last_time_waited / 10) # Sleep for a bit.  Maybe?
		# Now we have_new_blobinfo, so let's process.
		if curr_blobweights[0] 
			curr_velocity.angular.z = K_P * blobloc + K_D * (blobloc - pastloc)
			#print(curr_velocity.angular.z)
			curr_velocity.linear.x = .2
			pastloc = blobloc
			#print("going!!")
			pub.publish(curr_velocity)
		else: #stay still
			#print("Staying!!")
			twist_init()
			pub.publish(curr_velocity)
		
		
	
	# When we do have new_blobinfo we will 
	

def play_game():
	init_all() # Initialize everything (includes initialization of the twist for current motion)
	rospy.init_node('soccer_player', anonymous = True) # Initialize this node
	rospy.Subscriber('/blobs', Blobs, blobsCallback)
	
	follow_the_line()
	
	rospy.spin()

if __name__ == '__main__':
	play_game()
