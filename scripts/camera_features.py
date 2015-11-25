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
		self.image_pub = rospy.Publisher("turtle_env/camera_features/distance_image",Image)

		
		rospy.spin()
		

	def cameraCallback(self, data):
		if self.nPrints > 0:
			print 'getting callback'
			self.nPrints -= 1

		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
		except CvBridge, e:
			print e

		cv_image = cv2.resize(cv_image, (120,160))

		#print 'converring color space'
		conv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2YCR_CB)
		#print 'getting dist image'

		#cv2.imwrite('ycrcb.jpg', conv)

		#y, cr, cb = cv2.split(conv)
		#cv2.imwrite('y.jpg', y)
		#cv2.imwrite('cr.jpg', cr)
		#cv2.imwrite('cb.jpg', cb)

		#107 187 81	
		targetCol = rospy.get_param('turtle_env/camera_features/dist_color', [107, 255, 200])
		mask = rospy.get_param('turtle_env/camera_features/dist_mask', [1, 0, 0])

		#di = self.distImage(conv, (0, 255, 64), (1,0,0))
		di = self.distImage(conv, targetCol, mask)

		
		try:
			cdi = cv2.cvtColor(di, cv2.COLOR_GRAY2RGB)
			self.image_pub.publish(self.bridge.cv2_to_imgmsg(cdi, "rgb8"))
		except CvBridgeError, e:
			print e




		#print 'writing dist image'
		#cv2.imwrite('bdist.jpg', di)
		#cv2.imwrite('extract.jpg', cv_image)

		#cv2.imwrite('extract.jpg', cv_image)
		#cv2.imwrite('extractconv.jpg', conv)
		
		
		#print 'finished writing'

	def distImage(self, img, targetCol, mask=(0, 0, 0)):
		
		#img = cv2.GaussianBlur(img, (5,5), 2)

		targetCol = [0 if mask[i] == 1 else targetCol[i] for i in xrange(len(targetCol))]

		if mask[0] == 1:
			img[:,:,0] = 0
		if mask[1] == 1:
			img[:,:,1] = 0
		if mask[2] == 1:
			img[:,:,2] = 0

		height, width, channels = img.shape
		tImg = np.zeros((height,width,3), np.int)
		tImg[:,:] = targetCol
		sImg = img.astype(np.int) - tImg
		sqImg = np.square(sImg)
		y, cr, cb = cv2.split(sqImg)
		ssqImg = y + cr + cb
		sqrtImg = np.sqrt(ssqImg)
		normed = 1. - (sqrtImg / 255)
		sqNormed = np.square(normed)
		dImg = np.clip(255 * sqNormed, 0, 255).astype(np.uint8)
		

		return dImg




	



if __name__ == '__main__':
	CameraFeatures()
