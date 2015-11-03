#!/usr/bin/env python
import roslib
#roslib.load_manifest('rosopencv')
import sys
import rospy
import cv2
import math
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

depthImage = Image()
isDepthImageReady = False;
colorImage = Image()
isColorImageReady = False;


def updateDepthImage(data):
    global depthImage, isDepthImageReady, prev_depth
	prev_depth = depthValue
    depthImage = data
    isDepthImageReady = True

def updateColorImage(data):
    global colorImage, isColorImageReady
    colorImage = data
    isColorImageReady = True

def main():
    global depthImage, isDepthImageReady, colorImage, isColorImageReady, depthValue
    rospy.init_node('image_converter', anonymous=True)
    rospy.Subscriber("/camera/depth/image", Image, updateDepthImage, queue_size=10)
    rospy.Subscriber("/camera/rgb/image_color", Image, updateColorImage, queue_size=10)
    bridge = CvBridge()
    cv2.namedWindow("Color Image")
    cv2.setMouseCallback("Color Image", mouseClick)

    while not isDepthImageReady or not isColorImageReady:
        pass

    while not rospy.is_shutdown():
        try:
            depth = bridge.imgmsg_to_cv2(depthImage, desired_encoding="passthrough")
        except CvBridgeError, e:
            print e
            print "depthImage"

        try:
            color_image = bridge.imgmsg_to_cv2(colorImage, "bgr8")
        except CvBridgeError, e:
            print e
            print "colorImage"

		depthValue = []
		mid_height = 240
        for pixel in range(0, 640, 20):
        	depthValue = depth.item(mid_height,pixel,0)

		print(depthValue)

		for value in depthValue:
			if math.isnan(value):
				#stop and scan up
				continue
			else: #we have a real number, we want to see if its less than 1
				if value < 1:
					#stop and turn until we don't see it anymore
					continue
				else:
					#keep moving
					continue


        #print "Depth at (%i,%i) is %f." % (xLocation,yLocation,depthValue)

        #depthStr = "%.2f" % depthValue
        #locationStr = "(%i,%i)" % (xLocation, yLocation)

        #cv2.rectangle(color_image, (xLocation-10,yLocation-10), (xLocation+10,yLocation+10), (0,255,0), 2)
        #cv2.putText(color_image, locationStr, (xLocation+15, yLocation-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 1)
        #cv2.putText(color_image, depthStr, (xLocation+15,yLocation+10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 1)
    	cv2.imshow("Color Image", color_image)
        cv2.waitKey(1)

    cv2.destroyAllWindows()



if __name__ == '__main__':
    main()
