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

def draw_optic_flow_field(gray_image, points, flow):
    '''
    gray_image: opencv gray image, e.g. shape = (width, height)
    points: points at which optic flow is tracked, e.g. shape = (npoints, 1, 2)
    flow: optic flow field, should be same shape as points
    '''
    color_img = cv2.cvtColor(gray_image, cv2.COLOR_GRAY2BGR)
    color_red = [0,0,255] # bgr colorspace
    linewidth = 1
    for i, point in enumerate(points):
        x = point[0,0]
        y = point[0,1]
        vx = flow[i][0,0]
        vy = flow[i][0,1]
        cv2.line(color_img, (x,y), (x+vx, y+vy), color_red, linewidth) # draw a red line from the point with vector = [vx, vy]        
    
    cv2.imshow('optic_flow_field',color_img)
    cv2.waitKey(1)

def plotter(image, points, flow):

    color_img = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    color_red = [0,255,0] # bgr colorspace
    linewidth = 5
    for i, point in enumerate(points):
        x = point[0,0]
        y = point[0,1]
        vx = flow[i][0,0]
        vy = flow[i][0,1]
        cv2.line(color_img, (x,y), (x+vx, y+vy), color_red, linewidth) # draw a red line from the point with vector = [vx, vy]        
    
    plt.cla()
    # plt.plot(color_img)
    plt.imshow(color_img)
    # plt.show()
    plt.pause(0.05)
    # cv2.imshow('tracked image',color_img)
    # cv2.waitKey(1)

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

def opticflow_callback(data):
    global flow_x, flow_y

    vxarr = data.vx
    vyarr = data.vy

    flow_x = np.mean(vxarr[0])
    flow_y = np.mean(vyarr[0]) 

def rundetection():
    rospy.init_node('feature_detection', anonymous=True)
    right_sub = message_filters.Subscriber("/image_raw", Image, queue_size=10)#,heyo1)#,queue_size=4)
    left_sub = message_filters.Subscriber("/image_raw", Image, queue_size=10)#,heyo2)#,queue_size=4)

    rospy.Subscriber('/bebop/odom', Odometry, writeOdom)
    rospy.Subscriber("/optic_flow", OpticFlowMsg, opticflow_callback)

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

def PoseEstimate(leftImg,rightImg):
    global prev_time
    secs = leftImg.header.stamp.secs
    nsecs = leftImg.header.stamp.nsecs
    current_time = float(secs) + float(nsecs)*1e-9

    delta_t = current_time - prev_time
    prev_time = current_time
    # curr_time = rospy.get_rostime()

    left_image = bridge.imgmsg_to_cv2(leftImg, desired_encoding="mono8")
    right_image = bridge.imgmsg_to_cv2(rightImg, desired_encoding="mono8")

    img1 = left_image
    img2 = right_image

    kp1, des1 = featuredetect(left_image)

    image_feat = cv2.drawKeypoints(img1,kp1,None);

    plt.cla()
    plt.imshow(image_feat, 'gray')
    plt.pause(0.005)
    # plt.show()
    # prev_time = curr_time


if __name__ == '__main__':
    try:
        rundetection()
    except rospy.ROSInterruptException:
        pass
