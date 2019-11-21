#!/usr/bin/env python
# from __future__ import division
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
# import stereoDepth as SD
# from sklearn import linear_model, datasets

from nav_msgs.msg import Odometry # We need this message type to read position and attitude from Bebop nav_msgs/Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from std_msgs.msg import Empty
# import PlaneRANSAC as PR
from itertools import compress
import tf
from optic_flow_example.msg import OpticFlowMsg
import cPickle
import sys
# from pykalman import UnscentedKalmanFilter

# from robust_kalman import RobustKalman
# from robust_kalman.utils import HuberScore, VariablesHistory, WindowStatisticsEstimator
from sklearn.cluster import KMeans
# from sklearn.linear_model import RANSACRegressor

ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'

if ros_path in sys.path:

    sys.path.remove(ros_path)

import cv2

sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
keypoints = cPickle.loads(open("./keypoints.txt").read())
kp = []

for point in keypoints:
    temp = cv2.KeyPoint(x=point[0][0],y=point[0][1],_size=point[1], _angle=point[2], _response=point[3], _octave=point[4], _class_id=point[5]) 
    kp.append(temp)

des_img_des = np.loadtxt('descriptors.txt', dtype = float)
a = des_img_des
# a1 = np.loadtxt('descriptors1.txt', dtype = float)

rolavnum = 4
it = 0

# Rolling average
xarr = np.zeros(rolavnum)
yarr = np.zeros(rolavnum)

flow_x = 0
flow_y = 0
# Camera focal length [pixel]
f = 202
# Stereo base distance [mm]
B = 30
prev_time = 0
x_prev = 0
y_prev = 0

prev_image = None
last_time = 0
lk_params = dict( winSize  = (15,15),
                               maxLevel = 2,
                               criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
bridge = CvBridge()
points_to_track = []

center_pub = rospy.Publisher("/wall_center_point",Point)
contours_pub = rospy.Publisher('/mask', Image, queue_size=1)

def totuple(a):
    try:
        return tuple(totuple(i) for i in a)
    except TypeError:
        return a

def defpoints(image, spacing):
    points_to_track = []
    for x in range(0,image.shape[0],spacing):
        for y in range(0,image.shape[1],spacing):
            new_point = [y, x]
            points_to_track.append(new_point)
    points_to_track = np.array(points_to_track, dtype=np.float32) # note: float32 required for opencv optic flow calculations
    points_to_track = points_to_track.reshape(points_to_track.shape[0], 1, points_to_track.shape[1]) # for some reason this needs to be shape (npoints, 1, 2)
    return points_to_track

def writeOdom(data):
    global global_pos
    global global_vel
    global_pos=data.pose.pose
    global_vel=data.twist.twist

def rundetection():
    rospy.init_node('feature_detection', anonymous=True)
    right_sub = message_filters.Subscriber("/image_raw_throttled", Image, queue_size=10)#,heyo1)#,queue_size=4)
    left_sub = message_filters.Subscriber("/image_raw_throttled", Image, queue_size=10)#,heyo2)#,queue_size=4)

    rospy.Subscriber('/bebop/odom', Odometry, writeOdom)

    ts = message_filters.TimeSynchronizer([left_sub,right_sub],10)
    # ts.registerCallback(OpticalFlow)
    ts.registerCallback(PoseEstimate)
    rospy.spin()

def featuredetect(img):
    numFeatures = 500

    surf = cv2.xfeatures2d.SURF_create(numFeatures)
    kp, des = surf.detectAndCompute(img,None)

    # draw only keypoints location,not size and orientation
    img2 = cv2.drawKeypoints(img,kp,None,color=(0,255,0), flags=0)

    return kp, des

def featurecompare(des1, des2):

    matcher = cv2.DescriptorMatcher_create(cv2.DescriptorMatcher_FLANNBASED)
    matches = matcher.knnMatch(np.asarray(des1,np.float32),np.asarray(des2,np.float32), 2) #2

    return matches

def plotter(image, points, points1, points2, cc, col, col1, col2):

    color_img = image
    if cc == 0:
        color_img = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    color = col # bgr colorspace
    color1 = col1
    color2 = col2
    linewidth = 3
    for x,y in points:
        cv2.circle(color_img, (int(x),int(y)), 5 , color, thickness = linewidth) # draw a red line from the point with vector = [vx, vy]        
    for x,y in points1:
        cv2.circle(color_img, (int(x),int(y)), 5 , color1, thickness = linewidth) # draw a red line from the point with vector = [vx, vy]        
    # for x,y in points2:
    cv2.circle(color_img, (int(points2[0]),int(points2[1])), 5 , color2, thickness = linewidth) # draw a red line from the point with vector = [vx, vy]        
    return color_img
    # plt.cla()
    # # plt.plot(color_img)
    # plt.imshow(color_img)
    # # plt.show()
    # plt.pause(0.05)
    # cv2.imshow('tracked image',color_img)
    # cv2.waitKey(1)

def plotavg(image, point, cc):

    color_img = image
    if cc == 0:
        color_img = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    color = [0,255,0] # bgr colorspace
    linewidth = 3
    # for x,y in points:
    cv2.circle(color_img, (int(point[0]),int(point[1])), 5 , color, thickness = linewidth) # draw a red line from the point with vector = [vx, vy]        
    
    plt.cla()
    # plt.plot(color_img)
    plt.imshow(color_img)
    # plt.show()
    plt.pause(0.05)

def find_squares(img):

    gray_new = img
    # gray_new = cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
    # edges are preserved while doing median blur while removing noise 
    gra = cv2.medianBlur(gray_new,5) 
    #image normalization
    gray = np.zeros(gra.shape)
    gray = cv2.normalize(gra, gray, 0, 255, cv2.NORM_MINMAX)

    # adaptive threshold 
    thresh = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
            cv2.THRESH_BINARY,17,2)
    # erode out the noise
    thresh = cv2.erode(thresh,np.ones((3,3), np.uint8),iterations=1)

    im, cnts, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

    drawing = np.zeros((thresh.shape[0], thresh.shape[1], 3), np.uint8)
 
    # draw contours 
    for i in range(len(cnts)):
        color_contours = (255, 255, 255) 
        # draw contours in a black image
        cv2.drawContours(drawing, cnts, i, color_contours, 1, 8, hierarchy)
    
    # do dilation after finding the 
    drawing1 = cv2.dilate(drawing, np.ones((3,3), np.uint8), iterations=9)
    
    
    img_not = np.zeros((drawing1.shape[0], drawing1.shape[1], 3), np.uint8)
    img_not = cv2.bitwise_not(drawing1)
    mask = cv2.cvtColor(img_not, cv2.COLOR_BGR2GRAY)
    im1, cnts1, hierarchy1 = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    cnt_area = []
    cnt_num = []
    for c in cnts1:
        cnt_area.append(cv2.contourArea(c))

    cnt_num = np.argsort(cnt_area)
    cnt_area.sort()
    large_cnts = np.zeros(np.shape(mask))
    cnts_oi=[]
    for i in range(5): # in the 5 largest contours, check if cnt_area > 5000
        if cnt_area[len(cnt_area)-1-i] > 5000:
            fresh_im = np.zeros(np.shape(mask))
            cv2.drawContours(fresh_im, cnts1, cnt_num[len(cnt_num)-1-i], (255, 255, 255), -1)
            im_temp = 255*np.ones(mask.shape) - fresh_im

            cv2.drawContours(large_cnts, cnts1, cnt_num[len(cnt_num)-1-i], (255, 255, 255), -1)
            cnts_oi.append(cnts1[cnt_num[len(cnt_num)-1-i]])

    # dilate large conoturs 
    large_cnts = cv2.dilate(large_cnts, np.ones((5,5), np.uint8), iterations=1)

    new_gray = cv2.bitwise_and(gray_new, gray_new, mask = np.uint8(large_cnts))
    # cv2.imshow('mas',new_gray)

    return new_gray, cnts_oi

class Queue:

    #Constructor creates a list
    def __init__(self):
        self.queue = list()

    #Adding elements to queue
    def enqueue(self,data):
        #Checking to avoid duplicate entry (not mandatory)
        if data not in self.queue:
            self.queue.insert(0,data)
            return True
        return False

    #Removing the last element from the queue
    def dequeue(self):
        if len(self.queue)>0:
            return self.queue.pop()
        return ("Queue Empty!")

    #Getting the size of the queue
    def size(self):
        return len(self.queue)

    #printing the elements of the queue
    def printQueue(self):
        return self.queue

points_max_cx = Queue()
points_max_cy = Queue()


def PoseEstimate(leftImg,rightImg):
    global it
    left_image = bridge.imgmsg_to_cv2(leftImg, desired_encoding="mono8")

    img = left_image

    large_cnts = 0

    img, cnts_oi = find_squares(img)
    b1 = 0
    kp1, des1 = featuredetect(left_image)
    cur_img_des = des1
    b = cur_img_des

    matches = featurecompare(cur_img_des, des_img_des)

    points = np.zeros((len(matches),2))
    delta = np.zeros((len(matches),2))
    dist = np.zeros((len(matches)))

    matchMask = np.zeros((len(matches),2))        
    # ratio test as per Lowe's paper
    # source: https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_feature2d/py_matcher/py_matcher.html
    if len(matches)!=0:
        for i in range(0,len(matches)-1):
            points[i] = kp1[matches[i][0].queryIdx].pt#features[m.queryIdx]]
            if matches[i][0].distance < 0.8*matches[i][1].distance:    
                matchMask[i]=[1,0]       
    
    matchMaskbool = matchMask.astype('bool')
    points = points[matchMaskbool[:,0]]

    # Finding points inside contours
    # y = 0
    points_new = []#np.zeros((len(matches),2))
    for x in range(len(points)):
        for l in range(len(cnts_oi)):
        # print(points[x].tolist())
            point = (int(points[x][0]), int(points[x][1]))
            dist = cv2.pointPolygonTest(cnts_oi[l],point,False)
            if dist >= 0:
                points_new.append(points[x])
                # y = y+1

    # Finding average points

    # classifications, centers = kmeans(points_new)
    clusters_num = 3
    if len(points_new)<clusters_num:
        clusters_num = len(points_new)

    if clusters_num:
        print(clusters_num)
        estimator = KMeans(n_clusters=clusters_num)
        estimator.fit(points_new)
        # Ck's are the different clusters with corresponding point indices
        c1 = np.where(estimator.labels_ == 0)[0]
        c2 = np.where(estimator.labels_ == 1)[0]
        c3 = np.where(estimator.labels_ == 2)[0]
        max_len = len(c1)
        max_c = 0
        max_con = c1
 
        if len(c2) > max_len:
            max_len = len(c2)
            max_c = 1
            max_con = c2
        if len(c3) > max_len:
            max_len = len(c3)
            max_c = 2
            max_con = c3

        points_max_c = []
        # print(points_new[max_con[0]][:])
        for i in range(max_len):
            points_max_c.append(points_new[max_con[i]][:])

        max_cx = estimator.cluster_centers_[max_c][0]
        max_cy = estimator.cluster_centers_[max_c][1]

        if it<rolavnum: 
            points_max_cx.enqueue(max_cx)
            points_max_cy.enqueue(max_cy)

        else:
            points_max_cx.dequeue()
            points_max_cy.dequeue()

            points_max_cx.enqueue(max_cx)
            points_max_cy.enqueue(max_cy)


        it = it + 1
        
        x = 0
        y = 0
        temo1 = points_max_cx.printQueue()
        temo2 = points_max_cy.printQueue()
        for i in range (points_max_cy.size()):
            x = x + temo1[i]
            y = y + temo2[i]

        # rolling avg centroid
        x = int(x/points_max_cy.size())
        y = int(y/points_max_cy.size())

        outt = Point()
        outt.x=x
        outt.y=y
        center_pub.publish(outt)

        # plotavg(img,(x,y),0)

        centroid = [x,y]

        # # plotter(img,np.array(points_new),0, (255, 0, 0))
        pub_cv = plotter(img, points_new, points_max_c, centroid, 0, (255, 0, 0), (0, 0, 255), (0, 255, 0))

        contours_pub.publish(bridge.cv2_to_imgmsg(pub_cv, "rgb8")) 

if __name__ == '__main__':
    try:
        rundetection()
    except rospy.ROSInterruptException:
        pass
