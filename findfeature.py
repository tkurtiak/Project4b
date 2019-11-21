#!/usr/bin/env python
# from __future__ import division
import cv2
import numpy as np
import message_filters

from matplotlib import pyplot as plt
import imutils
from time import time, sleep
# import os
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from cv_bridge import CvBridge, CvBridgeError
import rospy
import copy
import stereoDepth as SD
# from sklearn import linear_model, datasets

from nav_msgs.msg import Odometry # We need this message type to read position and attitude from Bebop nav_msgs/Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from std_msgs.msg import Empty
import PlaneRANSAC as PR
from itertools import compress
import tf
from optic_flow_example.msg import OpticFlowMsg
import cPickle

import faulthandler

def featuredetect(img):

    numFeatures = 500

    surf = cv2.xfeatures2d.SURF_create(numFeatures)
    kp, des = surf.detectAndCompute(img,None)
    # Initiate STAR detector
    # orb = cv2.ORB_create(nfeatures=10000)

    # # compute the descriptors with ORB
    # kp, des = orb.detectAndCompute(img, None)

    # draw only keypoints location,not size and orientation
    img2 = cv2.drawKeypoints(img,kp,None,color=(0,255,0), flags=0)


# PLOT!

    # plt.cla()
    # plt.imshow(img2)
    # plt.pause(0.005)
    # # plt.show()

    return kp, des

def PoseEstimate_image(image):
    # global prev_time
    # secs = leftImg.header.stamp.secs
    # nsecs = leftImg.header.stamp.nsecs
    # current_time = float(secs) + float(nsecs)*1e-9

    # delta_t = current_time - prev_time
    # prev_time = current_time
    # curr_time = rospy.get_rostime()

    # left_image = bridge.imgmsg_to_cv2(leftImg, desired_encoding="mono8")
    # right_image = bridge.imgmsg_to_cv2(rightImg, desired_encoding="mono8")
    r = cv2.selectROI(image)

    image = image[int(r[1]):int(r[1]+r[3]), int(r[0]):int(r[0]+r[2])]
    
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    kp1, des1 = featuredetect(gray)

    image_feat = cv2.drawKeypoints(image,kp1,None)

    featarr = np.array(des1)
    keyarr = np.array(kp1)

    # fullarr = np.array(featarr,keyarr)
    keypoints = []
    descriptors = np.zeros(np.shape(featarr))
    num = 0

    for point in kp1:

        temp1 = (point.pt, point.size, point.angle, point.response, point.octave, point.class_id)
        keypoints.append(temp1)
        descriptors[num] = featarr[num]
        num = num + 1

    # for point in des1:
    #     temp2 = (point.)
    #     descriptors.append(temp2)
    # Dumping keypoints and descriptors
    f1 = open("./keypoints.txt", "w+")
    # f2 = open("./descriptors.txt", "w+")

    f1.write(cPickle.dumps(keypoints))
    np.savetxt('descriptors.txt', descriptors, fmt='%f')
    f1.close()
    # f2.close()
    # hist = cv2.calcHist([image],[0],None,[256],[0,256])
    # hist,bins = np.histogram(image.ravel(),256,[0,256])

    # hist = plt.hist(image.ravel(),256,[0,256]); 

    plt.cla()
    plt.imshow(image_feat, 'gray')
    plt.pause(0.005)
    plt.show()
    # prev_time = curr_time


if __name__ == '__main__':
    try:
        #Images 300 and 12
        image = cv2.imread("./newbag/frame0057.jpg")
        PoseEstimate_image(image)
        # rundetection()
    except rospy.ROSInterruptException:
        pass