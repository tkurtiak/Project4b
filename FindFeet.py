#!/usr/bin/env python2
import cv2
import numpy as np
from matplotlib import pyplot as plt
#from mpl_toolkits.mplot3d import axes3d, Axes3D 
#from scipy.stats import multivariate_normal
#from scipy.cluster.vq import kmeans, whiten, kmeans2
import imutils
#Image Select
#import Tkinter, tkFileDialog
from time import time
import os
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
import rospy
import copy

from sklearn import linear_model, datasets

from nav_msgs.msg import Odometry # We need this message type to read position and attitude from Bebop nav_msgs/Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from std_msgs.msg import Empty

import FixedGMM


bridge = CvBridge()

# Initialize GMM Feet params
GMMin = 'WoodenFeetGMM_3.npz'
thresh = .15#1e-4#1e-5
npzfile = np.load(GMMin)
k = npzfile['arr_0']
mean = npzfile['arr_1']
cov = npzfile['arr_2']
pi = npzfile['arr_3']

def run():
    rospy.init_node('feature_detection', anonymous=True)
    rospy.Subscriber("/image_raw", Image, detectFeet)
    rospy.spin()

def detectFeet(data):
	# Set global parameters for GMM inputs
	global thresh,k,mean,cov,pi

	global img,mask,kernel,erosion,betterMask, cnts
	# Pass image through CV bridge
	img = bridge.imgmsg_to_cv2(data,"passthrough")

	imgshape = img.shape

	
	
	img = img[img.shape[0]/2:img.shape[0],:,:]

	# Execute GMM
	mask = FixedGMM.GMM(img,thresh,k,mean,cov,pi)

	# Apply erode and dilate
	kernel = np.ones((5,5),np.uint8)
	erosion = cv2.erode(mask,kernel,iterations = 1)
	betterMask = cv2.dilate(erosion,kernel,iterations=3)
	contour_img=0*np.ones(np.shape(betterMask))
	# Now, select only the two biggest features
	# Using contours
	cnts = cv2.findContours(betterMask, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
	cnts = imutils.grab_contours(cnts)

	# Find top two contours
	i = 0
	A = np.zeros((len(cnts),2))
	for c in cnts:
		A[i,:] = np.array([cv2.contourArea(c),i])
		i = i+1
	sortA = np.sort(A,axis = 0)

	if len(cnts)>=2:
		max_contour = cnts[sortA[0,1].astype(int)]
		second_contour = cnts[sortA[1,1].astype(int)]
		contour_img=cv2.drawContours(contour_img, [max_contour], -1, (255, 255, 255), -1)
		contour_img=cv2.drawContours(contour_img, [second_contour], -1, (255, 255, 255), -1)
		numFeet = 2
	elif len(cnts) ==1:
		max_contour = cnts[sortA[0,1].astype(int)]
		
		contour_img=cv2.drawContours(contour_img, [max_contour], -1, (255, 255, 255), -1)
		numFeet = 1
	else:
		contour_img=0*np.ones(np.shape(betterMask))
		numFeet = 0


	# topA = 0
	# secondA = 0
	# max_contour = 0
	# second_contour = 0
	# if len(cnts) >=2:
	# 	#max_contour=cnts[0]
	# 	#second_contour=cnts[1]
	# 	for c in cnts:
	# 		A = cv2.contourArea(c)
	# 		if A>topA:
	# 			topA=A
	# 			max_contour=c
	# 		if A>secondA and A<topA:
	# 			secondA=A
	# 			second_contour=c
	# elif len(cnts) == 1:
	# 	max_contour = cnts
	# 	second_contour = 0
	# else:
	# 	max_contour = 0
	# 	second_contour = 0

	# contour_img=0*np.ones(np.shape(betterMask))
	# if len(cnts) >=1:
	# 	contour_img=cv2.drawContours(contour_img, [max_contour], -1, (255, 255, 255), -1)
	# if len(cnts) >=2:
	# 	contour_img=cv2.drawContours(contour_img, [second_contour], -1, (255, 255, 255), -1)


	plt.figure(1)
 	plt.imshow(img)
 	
 	plt.figure(2)
 	#plt.imshow(betterMask)
 	plt.imshow(contour_img)
 	plt.pause(0.05)
	# cv2.imshow('Raw',img)
	# cv2.imshow('mask',betterMask)
	# cv2.waitKey(10)
	# cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass