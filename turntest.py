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
SLEEP_TIME = .05
color_namelist = ['Greenline', 'Redball', 'Orangegoal']
x = []
y = []

def resetter():
	global del_x
	del_x = [0, 0, 0]
	global del_r
	del_r = [0, 0, 0]
	while pub3.get_num_connections() == 0:
		pass
	pub3.publish(Empty())
	rospy.sleep(.5)

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
				if color_index == color_namelist.index('Orangegoal') and box.area > 1000: #we only consider goal boxes that are BIG
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
			if maxgoal < area[2]: #we only want the center of the biggest goal we find
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
			if maxgoal < area[2]: #we only want the center of the biggest goal we find
				angles['g2'] = del_r[2]
	move_complete = False
	angle3 = angles['b2'] * 180.0 / math.pi
	angle4 = angles['g2'] * 180.0 / math.pi
	resetter()
	sys.stderr.write("angles (b2, g2): " + str(angle3)+ ", "+str(angle4)+'\n')
	(final_dist,final_angle)=triangles(angles)
	move_and_wait("F", .25, final_dist)
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
	lam = math.atan(hr/dict['b1'])
	w = height_b/math.tan(lam)
	dist = y + w
	lam = math.degrees(lam)
	sys.stderr.write("(Dist, lam): "str(dist)+" "+str(lam)+"\n")
	return (dist, lam)

def moveCallback(message):
	global move_complete
	move_complete = True

def play_game():
	global fd
	fd = open("roserrlog.txt", "w")
	init_all() # Initialize everything (includes initialization of the twist for current motion)
	rospy.init_node('turntest', anonymous = True) # Initialize this node
	rospy.Subscriber('/blobs', Blobs, blobsCallback)
	rospy.Subscriber('/odom', Odometry, odomCallback)
	rospy.Subscriber('subcontrol', String, moveCallback)
	resetter()
	turn_and_find()
	sys.stderr.write("End of the line\nPlaying ball\n")

	fd.close()

if __name__ == '__main__':
	play_game()
