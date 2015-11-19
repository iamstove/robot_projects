#!/usr/bin/env python
import rospy
import cv2
import math
from sensor_msgs.msg import Image
from struct import unpack
from cv_bridge import CvBridge, CvBridgeError
from cmvision.msg import Blobs, Blob
import copy

# Change this to your desired image topic
defaultImageTopic = "/camera/rgb/image_color"

colorImage = Image()
isColorImageReady = False
blobsData = Blobs()
isBlobsReady = False
depthImage = Image()
isDepthImageReady = False


def updateColorImage(data):
    global colorImage, isColorImageReady
    colorImage = data
    isColorImageReady = True

def updateBlobs(data):
    global blobsData, isBlobsReady
    blobsData = data
    isBlobsReady = True

def updateDepthImage(data):
    global depthImage, isDepthImageReady
    depthImage = data
    isDepthImageReady = True

def getDepth(x,y):
    global depthImage
    step = depthImage.step
    offset = (y * step) + (x * 4)
    (dist,) = unpack('f', depthImage.data[offset] + depthImage.data[offset+1] + depthImage.data[offset+2] + depthImage.data[offset+3])
    return dist

def filterByMinimumArea(blob, minimumArea):
    if blob.area < minimumArea:
        return False
    else:
        return True

def main():
    global colorImage, isColorImageReady, blobsData, isBlobsReady, depthImage, isDepthImageReady
    
    rospy.init_node('object_sizer', anonymous=True)
    rospy.Subscriber(defaultImageTopic, Image, updateColorImage, queue_size=10)
    rospy.Subscriber('/blobs', Blobs, updateBlobs)
    rospy.Subscriber('/camera/depth/image', Image, updateDepthImage, queue_size=10)
    bridge = CvBridge()
    cv2.namedWindow("Color Image")

    while (not isColorImageReady or not isBlobsReady or not isDepthImageReady) and not rospy.is_shutdown():
        pass

    while not rospy.is_shutdown():
        try:
            color_image = bridge.imgmsg_to_cv2(colorImage, "bgr8")
        except CvBridgeError, e:
            print e

        # Create a deep copy of blobsData
        blobsCopy = copy.deepcopy(blobsData)

        if blobsCopy.blob_count > 0:
            for box in blobsCopy.blobs:
                if filterByMinimumArea(box, 100):
                    startX = box.left
                    endX = box.right
                    startY = box.top
                    endY = box.bottom
                    cv2.rectangle(color_image, (startX,startY), (endX,endY), (0,255,0), 2)

        cv2.imshow("Color Image", color_image)
        cv2.waitKey(1)

    cv2.destroyAllWindows()
 
if __name__ == '__main__':
    main()
