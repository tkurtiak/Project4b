#!/usr/bin/env python

from __future__ import print_function
import sys
import os
PY3 = sys.version_info[0] == 3
dirpath = os.getcwd()

if PY3:
    xrange = range

import numpy as np
import cv2
import imutils

def find_squares(img):

    gray_new = cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
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

    for i in range(5): # in the 5 largest contours, check if cnt_area > 5000
        if cnt_area[len(cnt_area)-1-i] > 5000:
            fresh_im = np.zeros(np.shape(mask))
            cv2.drawContours(fresh_im, cnts1, cnt_num[len(cnt_num)-1-i], (255, 255, 255), -1)
            im_temp = 255*np.ones(mask.shape) - fresh_im

            cv2.drawContours(large_cnts, cnts1, cnt_num[len(cnt_num)-1-i], (255, 255, 255), -1)

    # dilate large conoturs 
    large_cnts = cv2.dilate(large_cnts, np.ones((5,5), np.uint8), iterations=1)

    new_gray = cv2.bitwise_and(gray_new, gray_new, mask = np.uint8(large_cnts))
    cv2.imshow('mas',new_gray)
    
    cv2.waitKey(0)
     
def main():
    for subdir, dirs, files in os.walk(dirpath + '/new_wall_down1'):
        files.sort()
        for file in files:
            filepath = subdir + os.sep + file
            if filepath.endswith(".jpg") or filepath.endswith(".pgm") or filepath.endswith(".png") or filepath.endswith(".ppm"):

                # print(file)
                img = cv2.imread(filepath)
  
                find_squares(img)
                
    print('Done')

if __name__ == '__main__':
    print(__doc__)
    main()
    cv2.destroyAllWindows()
