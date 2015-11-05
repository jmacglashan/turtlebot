#!/usr/bin/python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import math

class CameraFeatures:

	def __init__(self):
		self.nPrints = 1
		self.bridge = CvBridge()
		rospy.init_node('camera_features', anonymous=True)
		rospy.Subscriber('camera/rgb/image_color', Image, self.cameraCallback, queue_size=1)
		
		rospy.spin()
		

	def cameraCallback(self, data):
		if self.nPrints > 0:
			print 'getting callback'
			try:
				cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
			except CvBridge, e:
				print e

			conv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2YCR_CB)

			di = self.distImage(conv, (-1, 128, 128))
			cv2.imwrite('bdist.jpg', di)

			#cv2.imwrite('extract.jpg', cv_image)
			#cv2.imwrite('extractconv.jpg', conv)
			
			self.nPrints -= 1
			print 'finished writing'

	def distImage(self, img, targetCol):
		height, width, channels = img.shape
		distImg = np.zeros((height,width,1), np.uint8)
		for i in xrange(width):
				for j in xrange(height):
					distImg[j,i,0] = self.colDist(img[j,i], targetCol)
		return distImg

	def colDist(self, col, targetCol):
			sum = 0
			for i in xrange(length(col)):
				if targetCol[i] >= 0:
					d = targetCol[i] - col[i]
					sum += d*d
			dist = int(math.sqrt(sum))
			return dist



if __name__ == '__main__':
	CameraFeatures()
