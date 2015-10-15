#!/usr/bin/env python


import rospy
from cmvision.msg import Blobs, Blob
from geometry_msgs.msg import Twist

#we need not use constant command, assuming that the poling of the images is fast enough
pub = rospy.Publisher('kobuki_command', Twist, queue_size=10)

K_P = 1.5
K_D = 0.5

def blobsCallback(data): #this is called whenever a blob message is posted, blob messages are posted even if blobs are not detected
	global curr_velocity
	global pastloc
	global nocount
	x = 0
	y = 0
	area = 0
	bloblist = []
	if data.blob_count > 0: #we have a blob, track it
		for box in data.blobs:
			index = len(bloblist)
			bloblist.append(box)
			while index > 0 and bloblist[(index-1)/2].area < bloblist[index].area:
				bloblist[(index-1)/2], bloblist[index] = bloblist[index], bloblist[(index-1)/2]
				index = (index-1)/2	
		bigblob = bloblist[0]
		#nearlist = findBlobsInArea(bigblob, bloblist)
		#area = area + box.area
		#x = x + (box.x * box.area)
		#y = y + (box.y * box.area)
		#print(bigblob.area)
		#blobloc = bigblob.x - 320 #this is how we know which way to move the robot, negative to the left, pos to the right
		if bigblob.x > 320.0:
			blobloc = (500.0 - bigblob.x)/640.0
		else:
			blobloc = (bigblob.x - 500.0)/640.0
		curr_velocity.angular.z = K_P * blobloc # + K_D * (blobloc - pastloc)
		#print(blobloc)
		#curr_velocity.angular.z = K_P * blobloc + K_D * (blobloc - pastloc)
		curr_velocity.linear.x = .1 # + K_D * (blobloc - pastloc)
		pastloc = blobloc
		pub.publish(curr_velocity)
		nocount = 10
	else: #stay still
		print("Nah lmao")
		nocount -= 1
		if nocount < 5:
			twist_init()
			pub.publish(curr_velocity)
		
def nearBlobOverlap(blob, bloblist):
	feather = 5
	nearlist = []
	#for box in bloblist:
	#	if box.left - blob.right < feather and box.left - blob.right > feather:
			

def twist_init():
	global curr_velocity
	global pastloc
	global nocount
	nocount = 0
	pastloc = 0
	curr_velocity = Twist()
	curr_velocity.linear.x, curr_velocity.linear.y, curr_velocity.linear.z = 0, 0, 0
	curr_velocity.angular.x, curr_velocity.angular.y, curr_velocity.angular.z = 0, 0, 0

def detect_blob():
	
	twist_init()
	rospy.init_node('ball_tracker', anonymous = True)
	rospy.Subscriber('/blobs', Blobs, blobsCallback)
	rospy.spin()

if __name__ == '__main__':
	detect_blob()
