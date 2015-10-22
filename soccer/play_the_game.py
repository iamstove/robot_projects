#!/usr/bin/env python

#Imports#
import sys
import rospy
from geometry_msgs.msg import Twist
import math
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from cmvision.msg import Blobs, Blob
from std_msgs.msg import Empty, String
import time #itself!  WAUUUGHAHAHAHAHHAAAAA

#Globals#
pub = rospy.Publisher('kobuki_command', Twist, queue_size = 10) # Command publisher
pub2 = rospy.Publisher('keyboard_command', String, queue_size = 10) #publish to speed control, makes scanning turns easier
pub3 = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)

K_P = 1.5 # K_p in the PID equation
K_D = 1.25 # K_d in the PID equation
SLEEP_TIME = 0.1
color_namelist = ['Greenline', 'Redball', 'Orangegoal']
x = []
y = []

color_namelist = ['Greenline', 'Redball', 'Orangegoal'] # We'll use this for indexing different colors.

def odomCallback(data): #still think we might want to publish to speed control to move, but we'll still need odom for finding the angles
	global del_x
	global del_r
	# Convert quaternion to degree
	q = [data.pose.pose.orientation.x,
		 data.pose.pose.orientation.y,
		 data.pose.pose.orientation.z,
		 data.pose.pose.orientation.w]
	# roll, pitch, and yaw are in radian
	# degree = yaw * 180 / math.pi
	del_x = [data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z]
	del_r = euler_from_quaternion(q)

def resetter():
	global del_x
	del_x = [0, 0, 0]
	global del_r
	del_r = [0, 0, 0]
	while pub3.get_num_connections() == 0:
		pass
	pub3.publish(Empty())
	rospy.sleep(.5)

def blobsCallback(data): # This is called whenever a blobs message is posted; this happens constnatly even if blobs are not detected
	global curr_blobweights
	global has_new_blobinfo
	global fd
	global x, y
	x = [0, 0, 0] # Greenline, Redball, Orangegoal
	y = [0, 0, 0] # could be generalized but is ok for now
	area = [-1, -1, -1]
	if data.blob_count > 0: # We have a blob / some blobs, track them
		for box in data.blobs:
			if box.name in color_namelist:
				color_index = color_namelist.index(box.name)
				if color_index == color_namelist.index('Orangegoal') and box.area > 5000: #we only consider goal boxes that are BIG
					#sys.stderr.write(str(color_index)+" - " +str(box.area) + " \n")
					if area[color_index] == -1:
						area[color_index] = box.area
					else:
						area[color_index] += box.area
					y[color_index] += box.y * box.area
					x[color_index] += box.x * box.area
				else: #we consider other boxes of all sizes
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

def follow_the_line():
	# We first want to declare our dependency upon the global blob area.
	global curr_blobweights # Note well that this is effectively all blobsCallback changes when it runs.
	# We will simulate nodelike behavior by looping until a flag has_new_blobinfo is set.
	global has_new_blobinfo
	### This is similar to nodes in that it's basically waiting for a publish trigger to come down
	### and signal this 'node' to get to work.

	global pastloc

	# We will have a trigger that will mean we're finished with the line; this trigger requires we have not seen
	### a green blob for 10 new sets of blobs.	This trigger activates with changes in not_done_with_line
	not_done_with_line = True

	hope = 0 # The amount of hope we have at this point in time that we're on the line

	# The loop will incorporate a wait that waits 1/10 the time we waited to get the blob the last time.
	### This is so the program doesn't consume huge CPU.
	time_waited = 0 # We don't have to wait at all to try the first time.
	time_last = time.clock() # Start the time so we know when we started later.

	while not_done_with_line:
		while not(has_new_blobinfo):
			rospy.sleep(max(time_waited / 10, SLEEP_TIME / 10)) # Sleep for a bit.  Maybe?
		time_waited = time.clock() - time_last # Get the difference now
		time_last = time.clock() # Reset the time
		has_new_blobinfo = False
		# Now we have_new_blobinfo, so let's process.

		blobloc = curr_blobweights[0][0] # 0 is x, and then 0 is, in the color list, the index for Greenline
		if blobloc != -1: # If it hasn't been sentinelized
			blobloc = (320.0 - blobloc) / 320.0
			curr_velocity.angular.z = K_P * blobloc + K_D * (blobloc - pastloc)
			#print(curr_velocity.angular.z)
			curr_velocity.linear.x = 0.2
			pastloc = blobloc
			#print("going!!")
			pub.publish(curr_velocity)
			#hope = 20  We're hopeful that we'll continue to see the line

		else: # decide whether to stay still or keep up hope
			hope -= 1
			curr_velocity.linear.x -= .01
			#sys.stderr.write(str(hope)+"\n")
			if hope < 0:
				not_done_with_line = False # We're not NOT done with it ...
				#print("Staying!!")
				twist_init()
				pub.publish(curr_velocity)
			# else:
				# We've still got hope!!!

	# Now we're done with the line, so we need to look for the ball

'''def play_ball():
	# We first want to declare our dependency upon the global blob area.
	global curr_blobweights # Note well that this is effectively all blobsCallback changes when it runs.
	# We will simulate nodelike behavior by looping until a flag has_new_blobinfo is set.
	global has_new_blobinfo

	global del_x
	global del_r

	### NEEDS: TURN -PI/2 RAD ###
	sys.stderr.write("Startng Moving\n")
	#move_and_wait("L", 0.5, 90)
    pub2.publish("L .5 90")
	sys.stderr.write("Finished moving\n")
	### THEN, WE SEARCH ###

	not_done_with_search = True # We begin our search now, in fact

	hope = 0 # The amount of hope we have at this point in time that we're seeing the ball

	time_waited = 0 # We don't have to wait at all to try the first time.
	time_last = time.clock() # Start the time so we know when we started later.

	itemsFound = []
	pos1Dict = {}
	while not_done_with_search:
		itemsFound.append(scan(itemsFound)) # Will turn until it spots something, then control will continue
		lock_on(itemsFound[0]) # Will out of bounds error if no item is found in the end
		pos1Dict[itemsFound[0]] = del_r[2]
		itemsFound.append(scan(itemsFound))
		lock_on(itemFound[1])
		pos1Dict[itemsFound[1]] = del_r[2]


def lock_on(item_found):
	global curr_blobweights
	global has_new_blobinfo
	global del_x
	global del_r

	if "ball" == item_found:
		itemindex = 1
	elif "goal" == item_found:
		itemindex = 2
	else:
		return # ruh roh

	count = 0
	hope = 0

	while count < 1000:
		while not(has_new_blobinfo):
			rospy.sleep(SLEEP_TIME / 10) # Sleep for a bit.  Maybe?
		# Now we have_new_blobinfo, so let's process.

		blobloc = curr_blobweights[0][itemindex]

		if blobloc != -1: # If it hasn't been sentinelized
			blobloc = (320.0 - blobloc) / 320.0
			curr_velocity.angular.z = K_P * blobloc
			hope += 1.0/(1000.0*blobloc**2)
			pub.publish(curr_velocity)
			if hope > 100:
				return True
		else: # decide whether to stay still or keep up hope
			hope -= 1
			if hope < 0:
				twist_init()
				pub.publish(curr_velocity)
		count += 1
	return False

def scan(itemsFound):
	global curr_blobweights
	global has_new_blobinfo

	count = 0

	while count < 1000:
		while not(has_new_blobinfo):
			rospy.sleep(SLEEP_TIME / 10) # Sleep for a bit.  Maybe?
		# Now we have_new_blobinfo, so let's process.

		ballloc = curr_blobweights[0][1] # 0 is x, and then 1 is, in the color list, the index for Redball
		goalloc = curr_blobweights[0][2] # yada yada 2 is Orangegoal

		if ("ball" in itemsFound or ballloc == -1) and ("goal" in itemsFound or goalloc == -1): # If it HAS been sentinelized or we don't care about it either way,
			curr_velocity.angular.z = -0.1
			pub.publish(curr_velocity)
		else: # We do see something, so ...
			if not("ball" in itemsFound) and ballloc != -1:
				found = "ball"
				return found
			elif not("goal" in itemsFound) and goalloc != -1:
				found = "goal"
				return found
		count += 1
	return None'''

def turn_and_find():
	global x, move_complete
	angles = {}
	maxgoal = -1
	sys.stderr.write("Startng Moving\n")
	move_and_wait("L", 0.5, 90)
	sys.stderr.write("Resetting and moving again\n")
	pub2.publish("R .2 180")
	sys.stderr.write("Looking for things\n")
	middle = 320;
	while not(move_complete):
		if (x[1] < middle + 5) and (x[1] > middle - 5):
			#if not angles.has_key('b1'):
			#	sys.stderr.write("ball: "+str(del_r[2])+"\n")
			angles['b1'] = del_r[2]
		if (x[2] < middle - 5) and (x[2] > middle + 5):
			#if not angles.has_key('g1'):
			#	sys.stderr.write("goal: "+str(del_r[2])+"\n")
			#if maxgoal < area[2]: #we only want the center of the biggest goal we find
			angles['g1'] = del_r[2]
	move_complete = False
	angle1 = angles['b1'] *180.0 / math.pi
	angle2 = angles['g1']*180.0/math.pi
	sys.stderr.write("angles (b1,b2): " + str(angle1)+ ", "+str(angle2)+'\n')
	resetter()
	if angles['b1'] < -math.pi/2:
		move_and_wait("F", .5, .5)
	else:
		move_and_wait("B", .5, .5)

	move_and_wait("L", .5 ,180)
	pub2.publish("R .2 180")
	sys.stderr.write("Looking for things again\n")
	maxgoal = -1 #we'll have a new biggest
	while not(move_complete):
		if (x[1] < middle + 5) and (x[1] > middle - 5):
			#if not angles.has_key('b2'):
			#	sys.stderr.write("ball: "+str(del_r[2])+"\n")
			angles['b2'] = del_r[2]
		if (x[2] < middle + 5) and (x[2] > middle - 5):
			#if not angles.has_key('g2'):
				#	sys.stderr.write("goal: "+str(del_r[2])+"\n")
			#if maxgoal < area[2]: #we only want the center of the biggest goal we find
			angles['g2'] = del_r[2]
	move_complete = False
	angle3 = angles['b2'] * 180.0 / math.pi
	angle4 = angles['g2'] * 180.0 / math.pi
	resetter()
	sys.stderr.write("angles (b2, g2): " + str(angle3)+ ", "+str(angle4)+'\n')
	(final_dist,final_angle)=triangles(angles)
	if final_dist > 0:
		final_dist = math.fabs(final_dist)
		move_and_wait("F", .25, final_dist)
	else:
		final_dist = math.fabs(final_dist)
		move_and_wait("B", .25, final_dist)
	move_and_wait("L", .25, final_angle)
	move_and_wait("F", .75, 1)

def move_and_wait(direction, speed, distance):
	global move_complete, SLEEP_TIME
	move_complete = False
	pub2.publish(direction + " " + repr(speed) + " " + str(distance))
	while not(move_complete):
		rospy.sleep(SLEEP_TIME)
	move_complete = False
	resetter()

def triangles(dict):
	'''this function takes a dictionary of angles in radians and returns a tuple of distance and angle IN DEGREES this allows
	for direct input in to the moving functions'''
	x = (.5 * math.tan(dict['g1']))/(math.tan(dict['g2'])-math.tan(dict['g1']))
	height_g = (math.tan(dict['g2'])*x)

	y = (.5 * math.tan(dict['b1']))/(math.tan(dict['b2'])-math.tan(dict['b1']))
	height_b = math.tan(dict['b2'])*y

	hr = height_g-height_b
	lam = math.atan(hr/(y-x))
	w = height_b/math.tan(lam)
	dist = y + w
	lam = math.degrees(lam)
	sys.stderr.write("(Dist, lam): "+str(dist)+" "+str(lam)+"\n")
	return (dist, lam)

def moveCallback(message):
	global move_complete
	move_complete = True

def play_game():
	global fd
	fd = open("roserrlog.txt", "w")
	init_all() # Initialize everything (includes initialization of the twist for current motion)
	rospy.init_node('soccer_player', anonymous = True) # Initialize this node
	rospy.Subscriber('/blobs', Blobs, blobsCallback)
	rospy.Subscriber('/odom', Odometry, odomCallback)
	rospy.Subscriber('subcontrol', String, moveCallback)
	follow_the_line()
	resetter()
	sys.stderr.write("End of the line\nPlaying ball\n")
	turn_and_find()

	fd.close()

if __name__ == '__main__':
	play_game()
