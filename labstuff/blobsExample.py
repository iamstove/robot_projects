#!/usr/bin/env python


import rospy
from cmvision.msg import Blobs, Blob
from geometry_msgs.msg import Twist

pub = rospy.Publisher('kobuki_command', Twist, queue_size=10) #/mobile_base/commands/velocity

def blobsCallback(data):
    x = 0
    y = 0
    area = 0
    if data.blob_count > 0:
        for box in data.blobs:
            area = area + box.area
            x = x + (box.x * box.area)
            y = y + (box.y * box.area)
        x = x / area
        y = y / area
    print "(%i,%i)" % (x,y)

def blobsCallback2(data):
	global curr_velocity

	x = 0
	y = 0
	area = 0
	bloblist = []
	if data.blob_count > 0:
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
		direction = 0 # 
		print(bigblob.area)
	
def nearBlobOverlap(blob, bloblist):
	feather = 5
	nearlist = []
	#for box in bloblist:
	#	if box.left - blob.right < feather and box.left - blob.right > feather:
			

def twist_init():
	global curr_velocity
	curr_velocity = Twist()
	curr_velocity.linear.x, curr_velocity.linear.y, curr_velocity.linear.z = 0, 0, 0
	curr_velocity.angular.x, curr_velocity.angular.y, curr_velocity.angular.z = 0, 0, 0

def detect_blob():
    rospy.init_node('blob_tracker', anonymous = True)
    rospy.Subscriber('/blobs', Blobs, blobsCallback2)
    rospy.spin()

if __name__ == '__main__':
    detect_blob()
