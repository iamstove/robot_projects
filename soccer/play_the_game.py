#!/usr/bin/env python

#Imports#
import rospy
from geometry_msgs.msg import Twist
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
    while pub3.get_num_connections() == 0:
        pass
    pub3.publish(Empty())

def blobsCallback(data): # This is called whenever a blobs message is posted; this happens constnatly even if blobs are not detected
	global curr_blobweights
	global has_new_blobinfo
	x = [0, 0, 0] # Greenline, Redball, Orangegoal
	y = [0, 0, 0] # could be generalized but is ok for now
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
			# else
				# Unidentified or irrelevant color.  Ignore it / do nothing.

		for color_index in range(len(color_namelist)): # Divide by the total weight to find the center position
			if area[color_index] == -1:
				x[color_index] = -1
				y[color_index] = -1
			else:
				x[color_index] /= area[color_index]
				y[color_index] /= area[color_index]

		curr_blobweights = [x, y] # Boom.
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
	### a green blob for 10 new sets of blobs.  This trigger activates with changes in not_done_with_line
	not_done_with_line = True

	hope = 0 # The amount of hope we have at this point in time that we're on the line

	# The loop will incorporate a wait that waits 1/10 the time we waited to get the blob the last time.
	### This is so the program doesn't consume huge CPU.
	time_waited = 0 # We don't have to wait at all to try the first time.
	time_last = time.clock() # Start the time so we know when we started later.

	while not_done_with_line:
		while not(has_new_blobinfo):
			rospy.sleep(time_waited / 10) # Sleep for a bit.  Maybe?
		time_waited = time.clock() - time_last # Get the difference now
		time_last = time.clock() # Reset the time
		# Now we have_new_blobinfo, so let's process.

		blobloc = curr_blobweights[0][0] # 0 is x, and then 0 is, in the color list, the index for Greenline
		if blobloc != -1: # If it hasn't been sentinelized
			curr_velocity.angular.z = K_P * blobloc + K_D * (blobloc - pastloc)
			#print(curr_velocity.angular.z)
			curr_velocity.linear.x = .2
			pastloc = blobloc
			#print("going!!")
			pub.publish(curr_velocity)
			hope = 10 # We're hopeful that we'll continue to see the line
		else: # decide whether to stay still or keep up hope
			hope -= 1
			if hope < -20:
				not_done_with_line = False # We're not NOT done with it ...
			elif hope < 0:
				#print("Staying!!")
				twist_init()
				pub.publish(curr_velocity)
			# else:
				# We've still got hope!!!

	# Now we're done with the line, so we need to look for the ball

def play_ball():
	SEARCH_NONE, SEARCH_BALL, SEARCH_GOAL = 0,0,0 # Supposed to be a state thing, complete this line

	# We first want to declare our dependency upon the global blob area.
	global curr_blobweights # Note well that this is effectively all blobsCallback changes when it runs.
	# We will simulate nodelike behavior by looping until a flag has_new_blobinfo is set.
	global has_new_blobinfo

	### NEEDS: TURN -PI/2 RAD ###

	### THEN, WE SEARCH ###

	not_done_with_search = True # We begin our search now, in fact

	hope = 0 # The amount of hope we have at this point in time that we're seeing the ball

	time_waited = 0 # We don't have to wait at all to try the first time.
	time_last = time.clock() # Start the time so we know when we started later.

	while not_done_with_search:
		while not(has_new_blobinfo):
			rospy.sleep(time_waited / 10) # Sleep for a bit.  Maybe?
		time_waited = time.clock() - time_last # Get the difference now
		time_last = time.clock() # Reset the time
		# Now we have_new_blobinfo, so let's process.

		ballloc = curr_blobweights[0][1] # 0 is x, and then 1 is, in the color list, the index for Redball
		goalloc = curr_blobweights[0][2] # yada yada 2 is Orangegoal

		if ballloc == -1 and goalloc == -1: # If it HAS been sentinelized
			curr_velocity.angular.z = 0.2
			pub.publish(curr_velocity)
		elif ballloc != -1: # Else if we do see the ball
			### TURN TO THE BALL.  SET A STATE VALUE SO THAT WE FOLLOW THE BALL ON SUCCESSIVE LOOPS ###
			hope -= 1
			if hope < -20:
				not_done_with_line = False # We're not NOT done with it ...
			elif hope < 0:
				#print("Staying!!")
				twist_init()
				pub.publish(curr_velocity)
			# else:
				# We've still got hope!!!

	# Now we're done with the line, so we need to look for the ball

def stball():
    pub2.publish("L .25 90, R .2 180") #turn the robot



def play_game():
	init_all() # Initialize everything (includes initialization of the twist for current motion)
	rospy.init_node('soccer_player', anonymous = True) # Initialize this node
	rospy.Subscriber('/blobs', Blobs, blobsCallback)
	rospy.Subscriber('/odom', Odometry, odomCallback)

	follow_the_line()
	resetter()
	# play_ball()

	rospy.spin()

if __name__ == '__main__':
	play_game()
