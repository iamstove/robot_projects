#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from struct import unpack

depthData = Image();
isDepthReady = False;

def depthCallback(data):
    global depthImage, isDepthImageReady
    depthImage = data
    isDepthImageReady = True

def main():
    global depthData, isDepthReady
    rospy.init_node('depth_example', anonymous=True)
    rospy.Subscriber("/camera/depth/image", Image, depthCallback, queue_size=10)

    while not isDepthReady and not rospy.is_shutdown():
        pass

    while not rospy.is_shutdown():
        step = depthData.step
        depthValue = []
        tot = 0
        mid_height = 240
        for pixel in range(0, 640, 20):
            offset = (mid_height * step) + (pixel * 4)
        	(depthValue,) = unpack('f', depthData.data[offset] + depthData.data[offset+1] + depthData.data[offset+2] + depthData.data[offset+3])
            tot += depthValue

        tot /= 32
        print "Distance: %f" % dist
        print "Avg: %f" % tot

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



if __name__ == '__main__':
    main()
