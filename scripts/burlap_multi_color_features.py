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


class CameraFeaturesDriver:


	def __init__(self):
		self.nPrints = 1
		self.bridge = CvBridge()
		rospy.init_node('camera_channel_features', anonymous=True)
		rospy.Subscriber('camera/rgb/image_color', Image, self.cameraCallback, queue_size=1)
		self.features_pub = rospy.Publisher("turtle_env/camera_features/channel_features",CameraChannels)
	
		rospy.spin()
		

	def cameraCallback(self, data):
		if self.nPrints > 0:
			print 'getting callback'
			self.nPrints -= 1

		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
		except CvBridge, e:
			print e

		#first rescale
		cv_image = cv2.resize(cv_image, (160,120))
		#then color swap
		conv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2YCR_CB)

		#get parameters
		targetCols = rospy.get_param('turtle_env/camera_features/tColors', [[107, 255, 200], [200, 192, 70]])
		masks = rospy.get_param('turtle_env/camera_features/tMasks', [[1, 0, 0], [1, 0, 0]])
		thresholds = rospy.get_param('turtle_env/camera_features/thresholds', [80, 200])
		response_nears = rospy.get_param('turtle_env/camera_features/response_nears', [[0.1, 0.15], [0.65, 0.8]])
		response_mids = rospy.get_param('turtle_env/camera_features/response_mids', [[0.01, 0.03], [0.1, 0.2]])
		response_fars = rospy.get_param('turtle_env/camera_features/response_fars', [[0.0015, 0.003], [0.0015, 0.003]])
		pads = rospy.get_param('turtle_env/camera_features/window_pads', [0, 0])


		#set max channels to the min specification for all parameters
		mxChannels = len(targetCols)
		mxChannels = min(mxChannels, masks)
		mxChannels = min(mxChannels, thresholds)
		mxChannels = min(mxChannels, response_nears)
		mxChannels = min(mxChannels, response_mids)
		mxChannels = min(mxChannels, response_fars)


		camera_channel_features = CameraChannels()
		camera_channel_features.channels = []
		for i in xrange(mxChannels):
			responseParams = (response_nears[i], response_mids[i], response_fars[i])
			di = self.distImage(conv, targetCols[i], masks[i])

			scales = self.computeWindowRespon=se(di, 8, thresholds[i], responseParams, pads[i])

			nCF = CameraFeatures()
			nCF.features = scales[0].flatten()
			mCF = CameraFeatures()
			mCF.features = scales[1].flatten()
			fCF = CameraFeatures()
			fCF.features = scales[2].flatten()

			sf = CameraScaleFeatures()
			sf.scales = [nCF, mCF, fCF]

			camera_channel_features.channels.append(sf)

		self.features_pub.publish(camera_channel_features)



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


	def computeWindowResponse(self, img, factor, threshold, responseParams, pad=0):

		scales = []
		for i in xrange(len(responseParams)):
			scales.append(np.zeros((factor,factor,1), np.int))

		ret, img = cv2.threshold(img, threshold, 255, cv2.THRESH_TOZERO)
		for r in xrange(factor):
			for c in xrange(factor):
				wimg = self.extractWindow(r, c, img, factor, pad)
				numOn = np.mean(wimg) / 255. #fraction of on
				for i in xrange(len(scales)):
					response, cap = responseParams[i]
					fVal = ((numOn - response) / (cap - response)) * 255
					scales[i][r,c,0] = fVal

		for i in xrange(len(scales)):
			scales[i] = np.clip(scales[i], 0, 255).astype(np.uint8)

		return scales




	def extractWindow(self, r, c, img, factor, pad=0):
		height, width = img.shape
		wh = height / factor
		ww = width / factor

		spy = wh*r - pad
		spx = ww*c - pad

		epy = spy + wh + pad
		epx = spx + ww + pad

		spy = max(0, spy)
		spx = max(0, spx)

		epy = min(height, epy)
		epx = min(width, epx)

		window = img[spy:epy, spx:epx]

		return window



	



if __name__ == '__main__':
	CameraFeaturesDriver()