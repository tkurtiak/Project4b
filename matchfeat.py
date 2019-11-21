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
import sys
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'

if ros_path in sys.path:

    sys.path.remove(ros_path)

import cv2

sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
# import sys
# # insert at 1, 0 is the script path (or '' in REPL)
# # sys.path.insert(1, '/home/vdorbala/git/PyFeatureTrack/')
# sys.path.insert(1, '/home/vdorbala/git/KLT/')

# from klt import lucasKannadeTracker

import faulthandler

flow_x = 0
flow_y = 0
# Camera focal length [pixel]
f = 202
# Stereo base distance [mm]
B = 30
prev_time = 0

prev_image = None
last_time = 0
lk_params = dict( winSize  = (15,15),
                               maxLevel = 2,
                               criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
bridge = CvBridge()
points_to_track = []

odom_pub = rospy.Publisher("/our_odometry",Odometry,queue_size = 10)

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

# def OpticalFlow(leftImg,rightImg):
#     global prev_image, last_time, points_to_track
    
#     curr_image = bridge.imgmsg_to_cv2(leftImg, desired_encoding="mono8")

#     secs = leftImg.header.stamp.secs
#     nsecs = leftImg.header.stamp.nsecs
#     curr_time = float(secs) + float(nsecs)*1e-9

#     if prev_image is None:
#         prev_image = curr_image
#         last_time = curr_time
#         points_to_track = defpoints(curr_image, 100)
#         return

#     dt = curr_time - last_time

#     new_pos, status, error = cv2.calcOpticalFlowPyrLK(prev_image, curr_image, points_to_track, None, **lk_params)

#     flow = new_pos - points_to_track

#     plotter(curr_image, points_to_track, flow)

#     prev_image = curr_image
#     last_time = curr_time
flow_x_old = 0

def make_odometry(depth, delta_t):
    global flow_x_old
    #rosrun topic_tools throttle messages our_odometry 5.0 our_odometry

    global flow_x, flow_y

    flow_x = flow_x*5
    flow_y = flow_y*5
    depth = depth*1.001
    # print(flow_x, flow_y, depth)

    if abs(flow_y)>abs(flow_x_old):
        flow_x_old=flow_y

    if abs(flow_x)<1.8 or abs(flow_y)<1.8 or abs(depth)<2.5:

    # rospy.init_node('from_flow', anonymous=True)
    # ts = message_filters.TimeSynchronizer(flowsub,10)
        odom_broadcast = tf.TransformBroadcaster()

        odom_quat = tf.transformations.quaternion_from_euler(0,0,0)

        odom_broadcast.sendTransform((flow_x, flow_y, depth),(odom_quat),rospy.get_rostime(),'base_link', "odom")

        odom = Odometry()

        odom.pose.pose = Pose(Point(flow_x, flow_y, depth), Quaternion(*odom_quat))

        odom.header.stamp = rospy.get_rostime()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom_pub.publish(odom)


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

def featurecompare(des1, des2):
    # FLANN parameters
    # FLANN_INDEX_KDTREE = 1
    # index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    # search_params = dict(checks=50)   # or pass empty dictionary

    matcher = cv2.DescriptorMatcher_create(cv2.DescriptorMatcher_FLANNBASED)
    matches = matcher.knnMatch(np.asarray(des1,np.float32),np.asarray(des2,np.float32), 2) #2

    # ratio_thresh = 0.7
    # good_matches = []
    # for m,n in matches:
    #     if m.distance < ratio_thresh * n.distance:
    #         good_matches.append(m)

    # flann = cv2.FlannBasedMatcher(index_params,search_params)
    # matches = flann.knnMatch(des1,des2,k=2)
    return matches

def plotter(image, points, cc):

    color_img = image
    if cc == 0:
        color_img = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    color = [0,255,0] # bgr colorspace
    linewidth = 3
    for x,y in points:
        cv2.circle(color_img, (int(x),int(y)), 5 , color, thickness = linewidth) # draw a red line from the point with vector = [vx, vy]        
    
    plt.cla()
    # plt.plot(color_img)
    plt.imshow(color_img)
    # plt.show()
    plt.pause(0.05)
    # cv2.imshow('tracked image',color_img)
    # cv2.waitKey(1)

def angle_cos(p0, p1, p2):
    d1, d2 = (p0-p1).astype('float'), (p2-p1).astype('float')
    return abs( np.dot(d1, d2) / np.sqrt( np.dot(d1, d1)*np.dot(d2, d2) ) )

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

def PoseEstimate(leftImg,rightImg):
    global prev_time
    secs = leftImg.header.stamp.secs
    nsecs = leftImg.header.stamp.nsecs
    current_time = float(secs) + float(nsecs)*1e-9

    delta_t = current_time - prev_time
    prev_time = current_time
    # curr_time = rospy.get_rostime()

    left_image = bridge.imgmsg_to_cv2(leftImg, desired_encoding="mono8")

    img = left_image

    large_cnts = 0

    img, cnts_oi = find_squares(img)
    b1 = 0
    kp1, des1 = featuredetect(left_image)
    cur_img_des = des1
    b = cur_img_des

    keypoints = cPickle.loads(open("./keypoints.txt").read())
    kp = []

    for point in keypoints:
        temp = cv2.KeyPoint(x=point[0][0],y=point[0][1],_size=point[1], _angle=point[2], _response=point[3], _octave=point[4], _class_id=point[5]) 
        kp.append(temp)

    des_img_des = np.loadtxt('descriptors.txt', dtype = float)
    a = des_img_des
    # a1 = np.loadtxt('descriptors1.txt', dtype = float)

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
            # delta[i] = np.subtract(features[matches[i][0].queryIdx].pt,lastfeatures[matches[i][0].trainIdx].pt)   
            # dist[i] = np.sqrt(delta[i,0]**2+delta[i,1]**2)#matches[i][0].distance
            if matches[i][0].distance < 0.8*matches[i][1].distance:    
                matchMask[i]=[1,0]  
                # In addition to "Lowe's" method for bad feature match removel
                # Only accept features matches which have a delta less than some threshold
                # if dist[i]<20:    
                #     matchMask[i]=[1,0]     
    
    matchMaskbool = matchMask.astype('bool')
    ## Filter out bad feature matches
    #print dist
    # If distance is too high between matches, get rid of the match
    #matchMask = np.array(~(dist>20))
    #print matchMask
    #print matchMask.shape
    #global points, points2, delta2, matchMask
    # remove bad matches
    points = points[matchMaskbool[:,0]]
    delta = delta[matchMaskbool[:,0]]
    dist = dist[matchMaskbool[:,0]]
    #tracker = cv2.Tracker_KCF_create()
    #lastpoints = cv2.KeyPoint_convert(lastfeatures).astype(int)
    #features = KLT.calc_klt(lastimg, thisimg, lastpoints, win_size=(21, 21), max_iter=10, min_disp=0.01)
    
    #print "SIFT compare: ",len(features)
    y = 0
    points_new = np.zeros((len(matches),2))
    for x in range(len(points)):
        for l in range(len(cnts_oi)):
        # print(points[x].tolist())
            point = (int(points[x][0]), int(points[x][1]))
            dist = cv2.pointPolygonTest(cnts_oi[l],point,False)
            if dist >= 0:
                points_new[y] = points[x]
                y = y+1

    plotter(img,points_new,0)

if __name__ == '__main__':
    try:
        rundetection()
    except rospy.ROSInterruptException:
        pass
