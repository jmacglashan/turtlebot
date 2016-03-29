#!/usr/bin/python

import rospy
import cv2
from sensor_msgs.msg import Image
from turtle_env.msg import CameraFeatures
from turtle_env.msg import CameraScaleFeatures
from turtle_env.msg import CameraChannels
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import math
import time
import sys

class ImageScale:

    def __init__(self, width=160, height=120):
        self.width = width
        self.height = height
        self.nPrints = 1
        self.bridge = CvBridge()
        rospy.init_node('camera_scaled', anonymous=True)
        rospy.Subscriber('camera/rgb/image_color', Image, self.cameraCallback, queue_size=1)
        self.image_pub = rospy.Publisher("turtle_env/camera_scaled", Image)

        rospy.spin()

    def cameraCallback(self, data):
        if self.nPrints > 0:
            print 'getting callback'
            self.nPrints -= 1

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'rgb8')
        except CvBridge, e:
            print e

        cv_image = cv2.resize(cv_image, (self.width, self.height))
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "rgb8"))
        except CvBridgeError, e:
            print e


if __name__ == '__main__':
    if len(sys.argv) != 2:
        ImageScale()
    else:
        ImageScale(width=int(sys.argv[1]), height=int(sys.argv[2]))


