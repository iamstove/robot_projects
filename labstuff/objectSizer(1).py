#!/usr/bin/env python
import rospy
import cv2
import math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Change this to your desired image topic
defaultImageTopic = "/camera/rgb/image_color"

# Increase/decrease this value (integer from 1 to ...) to change the thickness of the box
boxThickness = 2

colorImage = Image()
isColorImageReady = False

startX = 0
startY = 0
endX = 0
endY = 0
isBoxReady = False
isDrawing = False

def mouseClick(event, x, y, flags, param):
    global startX, startY, endX, endY, isBoxReady, isDrawing
    if event == cv2.EVENT_LBUTTONDOWN and not isDrawing:
        startX = x
        startY = y
        isDrawing = True
        isBoxReady = False

    if event == cv2.EVENT_MOUSEMOVE and isDrawing:
        endX = x
        endY = y
        isBoxReady = True

    if event == cv2.EVENT_LBUTTONUP and isDrawing:
        endX,endY = x,y
        isDrawing = False
        if startX < endX:
            left,right = startX, endX
        else:
            left,right = endX, startX

        if startY < endY:
            top,bottom = startY, endY
        else:
            top,bottom = endY, startY

        width = math.fabs(startX - endX)
        height = math.fabs(startY - endY)
        widthHeightRatio = float(width)/float(height)
        
        print ""
        print "left : %3i right : %3i" % (left, right)
        print "top  : %3i bottom: %3i" % (top, bottom)
        print "width: %3i height: %3i" % (width, height)
        print "width/height radio %f" % widthHeightRatio

def updateColorImage(data):
    global colorImage, isColorImageReady
    colorImage = data
    isColorImageReady = True

def main():
    global colorImage, isColorImageReady, startX, startY, endX, endY
    
    rospy.init_node('object_sizer', anonymous=True)
    rospy.Subscriber(defaultImageTopic, Image, updateColorImage, queue_size=10)
    bridge = CvBridge()
    cv2.namedWindow("Color Image")
    cv2.setMouseCallback("Color Image", mouseClick)

    while not isColorImageReady and not rospy.is_shutdown():
        pass

    while not rospy.is_shutdown():
        try:
            color_image = bridge.imgmsg_to_cv2(colorImage, "bgr8")
        except CvBridgeError, e:
            print e
            print "colorImage"
        
        if isBoxReady:
            cv2.rectangle(color_image, (startX,startY), (endX,endY), (0,255,0), boxThickness)

        cv2.imshow("Color Image", color_image)
        cv2.waitKey(1)

    cv2.destroyAllWindows()
 
if __name__ == '__main__':
    main()
