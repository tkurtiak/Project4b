#!/usr/bin/env python

'''
Simple "Square Detector" program.
Loads several images sequentially and tries to find squares in each image.
'''

# Python 2/3 compatibility
from __future__ import print_function
import sys
import os
PY3 = sys.version_info[0] == 3
dirpath = os.getcwd()

if PY3:
    xrange = range

import numpy as np
import cv2 as cv
import cv2
import imutils


def angle_cos(p0, p1, p2):
    d1, d2 = (p0-p1).astype('float'), (p2-p1).astype('float')
    return abs( np.dot(d1, d2) / np.sqrt( np.dot(d1, d1)*np.dot(d2, d2) ) )

def find_squares(img):
    # img = cv.GaussianBlur(img, (5, 5), 0)
    gray_new = cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
    gray = cv.medianBlur(gray_new,5) 
    edges = cv2.Canny(img,100,200)
  
    # ret,thresh = cv2.threshold(gray,0,120,cv.THRESH_BINARY)
    thresh = cv.adaptiveThreshold(gray,255,cv.ADAPTIVE_THRESH_GAUSSIAN_C,\
            cv.THRESH_BINARY,17,2)
  
    # # erode out the noise
    thresh = cv2.erode(thresh,np.ones((3,3), np.uint8),iterations=1)

    im2, cnts, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    # cnts1 = imutils.grab_contours(cnts)
    # img = cv2.drawContours(img, cnts, -1, (0,255,0), 3)

    hull = []
 
    # calculate points for each contour
    for i in range(len(cnts)):
    # creating convex hull object for each contour
        hull.append(cv2.convexHull(cnts[i], False))
    # create an empty black image
    drawing = np.zeros((thresh.shape[0], thresh.shape[1], 3), np.uint8)
 
    # draw contours and hull points
    for i in range(len(cnts)):
        color_contours = (255, 255, 255) # green - color for contours
        color = (255, 0, 0) # blue - color for convex hull
        # draw ith contour
        cv2.drawContours(drawing, cnts, i, color_contours, 1, 8, hierarchy)
        # draw ith convex hull object
        # cv2.drawContours(drawing, hull, i, color, 1, 8)

    drawing1 = cv2.dilate(drawing,np.ones((3,3), np.uint8),iterations=9)
    img_not = np.zeros((drawing1.shape[0], drawing1.shape[1], 3), np.uint8)
    img_not = cv2.bitwise_not(drawing1)
    # img_not.convertTo(img_not, CV_8U)
    mask=cv.cvtColor(img_not, cv2.COLOR_BGR2GRAY)
    im1, cnts1, hierarchy1 = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
     
    cnt_area = []
    cnt_num = []
    for c in cnts1:
        cnt_area.append(cv2.contourArea(c))

    cnt_num = np.argsort(cnt_area)
    cnt_area.sort()
    coi = []
    large_cnts = 0*np.ones(np.shape(mask))

    for i in range(5): # in the 5 largest contours, check if cnt_area > 5000
        if cnt_area[len(cnt_area)-1-i] > 5000:
            cv2.drawContours(large_cnts, cnts1, cnt_num[len(cnt_num)-1-i], (255, 255, 255), -1)
    # print(large_cnts.shape)
    

    gray_rgb=cv2.cvtColor(gray_new, cv2.COLOR_GRAY2RGB)
    # print(gray_rgb.shape)

    # #apply the mask to the original thickened mask
    new_large = cv2.dilate(large_cnts,np.ones((9,9), np.uint8),iterations=4)

    new_gray = cv2.bitwise_and(gray_rgb, gray_rgb, mask = np.uint8(new_large))

    hull1 = []
 
    # calculate points for each contour
    for i in range(len(cnts1)):
    # creating convex hull object for each contour
        hull1.append(cv2.convexHull(cnts1[i], False))
    # create an empty black image
    drawing2 = np.zeros((thresh.shape[0], thresh.shape[1], 3), np.uint8)
 
    # draw contours and hull points
    for i in range(len(cnts1)):
        color_contours = (0, 255, 0) # green - color for contours
        color = (255, 0, 0) # blue - color for convex hull
        # draw ith contour
        cv2.drawContours(drawing2, cnts1, i, color_contours, 1, 8, hierarchy1)
        # draw ith convex hull object
        cv2.drawContours(drawing2, hull1, i, color, 1, 8)
    
    cv2.imshow('thresh',thresh)

    cv2.imshow('contours',drawing)
    cv2.imshow('contours_eroded',new_gray)

    cv2.waitKey(0)

def main():
    for subdir, dirs, files in os.walk(dirpath + '/wall_down'):
        files.sort()
        for file in files:
            filepath = subdir + os.sep + file
            if filepath.endswith(".jpg") or filepath.endswith(".pgm") or filepath.endswith(".png") or filepath.endswith(".ppm"):

                # print(file)
                img = cv.imread(filepath)
                find_squares(img)    

    print('Done')


if __name__ == '__main__':
    print(__doc__)
    main()
    cv.destroyAllWindows()
