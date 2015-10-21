#!/usr/bin/env python

import rospy
import math
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from std_msgs.msg import String


pub = rospy.Publisher('kobuki_command', Twist, queue_size=10)
pub2 = rospy.Publisher('subcontrol', String, queue_size=10)

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
	#print ('List of commands: ' + str(command_list))
	#for each command in command list, split it up and send it to the accelerator program
	for c in command_list:
		#print ("Command: " + str(c))
		#remove the space on the left
		c = c.lstrip().rstrip()
		#split c into a triple that contains the following things
		command_type, max_speed, distance = c.split(' ')
		speed_change(command_type[0].upper(), float(max_speed), float(distance))
		resetter()
		rospy.sleep(0.25)
		
		#the split command in this case should always have 3 parts
		#print ('Split command:' + str((command_type, max_speed, distance)))
		

def odomCallback(data):
	global del_x
	global del_r 
	global odom_reset
	global q_0
	global r_0
	
	if True == odom_reset:
		q_0 = [data.pose.pose.orientation.x,
	           data.pose.pose.orientation.y,
	           data.pose.pose.orientation.z,
	           data.pose.pose.orientation.w]
		r_0 = [data.pose.pose.position.x,
			   data.pose.pose.position.y,
			   data.pose.pose.position.z]
		odom_reset = false
 
	# Convert quaternion to degree
	q = [data.pose.pose.orientation.x,
		 data.pose.pose.orientation.y,
		 data.pose.pose.orientation.z,
		 data.pose.pose.orientation.w]
	
	r = [data.pose.pose.position.x,
		 data.pose.pose.position.y,
		 data.pose.pose.position.z]
	
	euler  = euler_from_quaternion(q)
	euler0 = euler_from_quaternion(q_0)
	
	roll, pitch, yaw = map(lambda x, y: x - y, euler, euler0)
	# roll, pitch, and yaw are in radian
	# degree = yaw * 180 / math.pi
	del_r = yaw

	del_x, del_y, del_z = map(lambda x, y: x - y, r, r_0)
	


# method begins
def speed_change(command_type, max_speed, distance):
	global curr_velocity
	global not_bumping
	global del_x
	global del_r
	
	lin_min = 0.03
	rot_min = 0.3
	lin_max = 1
	rot_max = 2
	progr = 0
	spd_min = 0	# Don't know which speed this is yet.
	acc_max = 0	# max accel, changed based on type of command
	turns = 0	# The number of turns counterclockwise; starts at zero
	past_del_r = 0	# the previous delta-r was zero
	
	not_bumping = True
	maxim = max_speed # The maximum speed, aka k
	speed = 0
	del_final = distance	# the final distance
	del_x = 0	# current distance
	del_r = 0	# current rotation
	sleep_time = 0.02	#this value may need to change
	resetter()

	rospy.loginfo("Command: " + command_type + " " + str(max_speed) + " " + str(distance))
	
	# if we're working with a rotation instruction, we're going to need to convert
	# from deg to radians; the command is in deg.
	# We will also want to set the minimum speed to the appropriate one.
	if command_type == 'R' or command_type == 'L':	# If we're working with a rotational command
		spd_min = rot_min			##and set the min speed to the rotational min
		acc_max = rot_max
	else:				# Otherwise, we're working with a linear command
		spd_min = lin_min	##so we should be using the linear minimum speed
		acc_max = lin_max
	# if we're working with a "negative" command (that which causes our values picked
	# up from odom to be negative in lin.x or ang.z, we want to reverse
	# where the final delta should be.
	if command_type == 'B' or command_type == 'R':	# If the command is a 'negative directioned' one
		del_final = -del_final			##Make the final destination negative
	
	while not_bumping:	# While we haven't collided with anything (in the front at least) ...	
		# change the twist curr_velocity to be a twist representative of the current motion required
		if command_type == 'F' or command_type == 'B':	# If we're moving forwards or backwards
			progr = del_x / del_final		##then our level of progress depends on del_x
		elif command_type == 'R' or command_type == 'L':# Else if we're going right or left
			if past_del_r > 0 and del_r < 0:
				if past_del_r > math.pi / 3: # so not close to zero
					turns += 1
			elif past_del_r < 0 and del_r > 0:
				if past_del_r < -math.pi / 3:
					turns -= 1
			del_final = del_final * math.pi / 180.0
			past_del_r = del_r
			progr = (del_r + turns * 2 * math.pi) / del_final ##then our progress depends on del_r
		else:
			rospy.loginfo(str(command_type) + " was submitted; invalid command type character.") 
			break
				
		#speed = math.sqrt(max(spd_min*spd_min, (1.0 - math.fabs(1.0 - 2.0*progr)) * maxim * maxim))
		speed = min(math.sqrt(max(spd_min*spd_min, acc_max*math.fabs(del_final - del_final*math.fabs(1 - 2.0*progr)))), maxim)
		print(speed)	
		if command_type == 'F':
			curr_velocity.linear.x = speed
		elif command_type == 'B':
			curr_velocity.linear.x = -speed
		elif command_type == 'R':
			curr_velocity.angular.z = -speed
		elif command_type == 'L':
			curr_velocity.angular.z = speed
		
		if progr >= 1:	# If we're at or over 100% of the way there,
			rospy.loginfo("command completed")
			break
		#publish the changed speed to the constant command's topic
		pub.publish(curr_velocity)
		pub2.publish("Done")
		#have a nap
		rospy.sleep(sleep_time)

	curr_velocity.linear.x = 0
	curr_velocity.angular.z = 0
	pub.publish(curr_velocity)

def resetter():
	global odom_reset
	odom_reset = True
	
	
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
