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

def resetter():
	global del_x
	del_x = [0, 0, 0]
	global del_r
	del_r = [0, 0, 0]
	while pub3.get_num_connections() == 0:
		pass
	pub3.publish(Empty())

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
	sys.stderr.write("Startng Moving\n")
	move_and_wait("L", 0.5, 90)
    #pub2.publish("L .5 90")
	sys.stderr.write("Finished moving\n")


def move_and_wait(direction, speed, distance):
	global move_complete, SLEEP_TIME
	move_complete = False
	pub2.publish(direction + " " + repr(speed) + " " + str(distance))
	while not(move_complete):
		rospy.sleep(SLEEP_TIME)

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
	play_ball()

	fd.close()

if __name__ == '__main__':
	play_game()
