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

class ImageScale:

    def __init__(self):
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
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridge, e:
            print e

        cv_image = cv2.resize(cv_image, (160, 120))
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cdi, "rgb8"))
        except CvBridgeError, e:
            print e


if __name__ == '__main__':
    ImageScale()


