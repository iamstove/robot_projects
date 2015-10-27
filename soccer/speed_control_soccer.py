#!/usr/bin/env python

import rospy
import math
import sys
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from std_msgs.msg import String


pub = rospy.Publisher('kobuki_command', Twist, queue_size=10)
pub2 = rospy.Publisher('subcontrol', String, queue_size=10)
reverse = {'F': 'B', 'B': 'F', 'R': 'L', 'L': 'R'}

def twist_init():
	global curr_velocity
	curr_velocity = Twist()
	curr_velocity.linear.x = 0.0
	curr_velocity.linear.y = 0.0
	curr_velocity.linear.z = 0.0
	curr_velocity.angular.x = 0.0
	curr_velocity.angular.y = 0.0
	curr_velocity.angular.z = 0.0

def parse_command(data):
	command_list = data.data.split(',')
	sys.stderr.write('List of commands: ' + str(command_list)+'\n')
	#for each command in command list, split it up and send it to the accelerator program
	for c in command_list:
		#remove the space on the left
		c = c.lstrip().rstrip()
		#split c into a triple that contains the following things
		command_type, max_speed, distance = c.split(' ')
		if distance < 0: 
			# Fek off m8; you're giving me weird reversed commands now
			speed_change(reverse[command_type[0:1].upper()], float(max_speed), -float(distance))
		else:	
			speed_change(command_type[0].upper(), float(max_speed), float(distance))
		resetter()
		rospy.sleep(0.25)

	resetter()



def odomCallback(data):
	global del_x, del_r
	global odom_reset
	global q_0, r_0, past_r
	global turns
	
	# Handle reset
	if odom_reset:
		q_0 = [data.pose.pose.orientation.x,
			data.pose.pose.orientation.y,
			data.pose.pose.orientation.z,
			data.pose.pose.orientation.w]
		r_0 = [data.pose.pose.position.x,
			data.pose.pose.position.y,
			data.pose.pose.position.z]
		turns = 0
		odom_reset = False
		

	# Convert quaternion to degree
	q = [data.pose.pose.orientation.x,
		 data.pose.pose.orientation.y,
		 data.pose.pose.orientation.z,
		 data.pose.pose.orientation.w]

	# Pick up our current position
	r = [data.pose.pose.position.x,
		data.pose.pose.position.y,
		data.pose.pose.position.z]

	# Set the euler angles from our current quaternions
	euler  = euler_from_quaternion(q)
	euler0 = euler_from_quaternion(q_0)

	if past_r[2] > 0 and r[2] < 0:
		if past_r[2] > math.pi / 3: # so not close to zero
			turns += 1
	elif past_r[2] < 0 and r[2] > 0:
		if past_r[2] < -math.pi / 3:
			turns -= 1

	del_r = euler[2] - euler_0[2] + turns * 2 * math.pi
	past_r = r
	del_x, del_y, del_z = map(lambda x, y: x - y, r, r_0)

def speed_change(command_type, max_speed, distance):
	global curr_velocity
	global not_bumping
	global del_x
	global del_r

	lin_min = 0.03
	rot_min = 0.2
	lin_max = 1
	rot_max = 2
	progr = 0
	spd_min = 0	# Don't know which speed this is yet.
	acc_max = 0	# max accel, changed based on type of command
	past_del_r = 0	# the previous delta-r was zero

	not_bumping = True
	maxim = max_speed # The maximum speed, aka k
	speed = 0
	del_final = distance	# the final distance
	sleep_time = 0.02	# this value may need to change
	
	resetter()

	# if we're working with a rotation instruction, we're going to need to convert
	# from deg to radians; the command is in deg.
	# We will also want to set the minimum speed to the appropriate one.
	if command_type == 'R' or command_type == 'L':	# If we're working with a rotational command
		del_final = del_final * math.pi / 180.0 ##then we know we heard degrees, so make rads
		#sys.stderr.write("Del final in rad: " + str(del_final)+"\n")
		spd_min = rot_min			##and set the min speed to the rotational min
		acc_max = rot_max
	else:				# Otherwise, we're working with a linear command
		spd_min = lin_min	##so we should be using the linear minimum speed
		acc_max = lin_max
	# if we're working with a "negative" command (that which causes our values picked
	# up from odom to be negative in lin.x or ang.z, we want `to reverse
	# where the final delta should be.
	if command_type == 'R' or command_type == 'B':	# If the command is a 'negative directioned' one
		del_final = -del_final			##Make the final destination negative

	while not_bumping:	# While we haven't collided with anything (in the front at least) ...
		# change the twist curr_velocity to be a twist representative of the current motion required
		if command_type == 'F' or command_type == 'B':	# If we're moving forwards or backwards
			progr = math.fabs(del_x / del_final)		##then our level of progress depends on del_x
		elif command_type == 'R' or command_type == 'L':# Else if we're going right or left
			progr = del_r / del_final ##current rotation * number of turns * 2 * pi / final
		else:
			sys.stderr.write(str(command_type) + " was submitted; invalid command type character.\n")
			break
		speed = min(math.sqrt(max(spd_min*spd_min, acc_max*math.fabs(del_final - del_final*math.fabs(1 - 2.0*progr)))), maxim)
		if command_type == 'F':
			curr_velocity.linear.x = speed
		elif command_type == 'B':
			curr_velocity.linear.x = -speed
		elif command_type == 'R':
			curr_velocity.angular.z = -speed
		elif command_type == 'L':
			curr_velocity.angular.z = speed

		if progr >= 1.0:	# If we're at or over 100% of the way there,
			sys.stderr.write("command completed\n")
			break
	
		#publish the changed speed to the constant command's topic
		pub.publish(curr_velocity)
		#have a nap
		rospy.sleep(sleep_time)

	curr_velocity.linear.x = 0
	curr_velocity.angular.z = 0
	pub.publish(curr_velocity)
	pub2.publish("Done")

def resetter():
	global odom_reset
	odom_reset = True
	while odom_reset:
		pass

def bump_respond(data):
	global not_bumping
	not_bumping = False

def command_listen():
	rospy.init_node('speed_change', anonymous=True)
	twist_init()
	resetter()
	rospy.Subscriber('keyboard_command', String, parse_command)
	rospy.Subscriber('impact', String, bump_respond)
	rospy.Subscriber('/odom', Odometry, odomCallback)
	rospy.spin()

if __name__ == '__main__':
	try:
		command_listen()
	except rospy.ROSInterruptException:
		pass
