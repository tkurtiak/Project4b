#!/usr/bin/env python

import cv2 
import numpy as np
import imutils
# from matplotlib import pyplot as plt
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
import rospy
# from multiprocessing import Pool,Process, Queue


bridge = CvBridge()
img_pub = rospy.Publisher("/center_point_img",Image,queue_size=1)
bridge_pub = rospy.Publisher("/bridge_details",Point,queue_size=1)


def poor_mans_differential(img, cannyval):
	edges = cv2.Canny(img,cannyval,cannyval*2.7,apertureSize = 3)
	# edges = cv2.dilate(edges,np.ones((3,3),np.uint8),iterations = 1)
	return edges

def double_threshold(img,r1,r2,threshinner,threshouter):

	inner_only= img*0

	outer_only=inner_only+1#np.ones(img.shape)

	inner_only[int(img.shape[0]/2-r1):int(img.shape[0]/2+r1) , int(img.shape[1]/2-r2):int(img.shape[1]/2+r2)]=1
	outer_only[int(img.shape[0]/2-r1):int(img.shape[0]/2+r1) , int(img.shape[1]/2-r2):int(img.shape[1]/2+r2)]=0

	inner_pic= cv2.bitwise_and(img,img, mask=inner_only)
	outer_pic= cv2.bitwise_and(img,img, mask=outer_only)

	outer_mask= cv2.inRange(outer_pic, threshouter[0],threshouter[1])#100, 200)
	inner_mask= cv2.inRange(inner_pic, threshinner[0],threshinner[1])#130, 230)

	mask=cv2.bitwise_or(outer_mask,inner_mask)

	return mask
	# mask = cv2.inRange(img_filt, 0, 13)

def find_bridge(image_raw):


	print '--------------'

	cannyval=int(np.amin(image_raw.shape)*.12)#.35)#.31)
	print cannyval
	# adaptive_filtersize= int(np.amin(image_raw.shape)*.01)
	# print adaptive_filtersize

	
	img_filt=poor_mans_differential(image_raw,cannyval)
	mask = cv2.inRange(img_filt, 0, 13)
	# mask2 = cv2.inRange(image_raw, 130, 230)
	# mask = cv2.erode(mask,np.ones((3,3),np.uint8),iterations = 1)
	# mask = cv2.dilate(mask,np.ones((3,3),np.uint8),iterations = 2)
	# 



	mask2=double_threshold(image_raw,60,60,[235,260],[170,260])
	mask= cv2.bitwise_and(mask,mask2)


	# mask = cv2.erode(mask,np.ones((3,3),np.uint8),iterations = 1)
	# mask = cv2.dilate(mask,np.ones((3,3),np.uint8),iterations = 1)
	# 

	

	# edges = cv2.Canny(mask,cannyval,cannyval*3,apertureSize = 3) 

	# cv2.imshow('canny',edges)

	cnts = cv2.findContours(mask, cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
	cnts = imutils.grab_contours(cnts)

	maxA=0
	countours_exist=False
	got_bridge=False
	cx0=0
	cy0=0
	bridge_angle=0
	first_hole_angle=None

	for c in cnts:
		
		# A = cv2.contourArea(c)
		A = cv2.arcLength(c,False)
		
		if A>maxA:
			countours_exist=True
			maxA=A
			max_contour=c

	if countours_exist:
			checked=np.array([[-1,-1]])
			

			p_min= .15* cv2.arcLength(max_contour,True)
			equi_radius = .125/.15*p_min #.5*np.sqrt(4*maxA/np.pi)
			M = cv2.moments(max_contour)
			cx0 = int(M['m10']/(M['m00']+.0000000001))
			cy0 = int(M['m01']/(M['m00']+.0000000001))

			cv2.drawContours(image_raw, [max_contour], -1, 255, 4)
			for c in cnts:
				perimeter = cv2.arcLength(c,True)
				if perimeter>p_min: #if its not very small
					M = cv2.moments(c)
					cx = int(M['m10']/(M['m00']+.0000000001))
					cy = int(M['m01']/(M['m00']+.0000000001))


					if np.equal(checked,[cx,cy]).all(1).any()==False:
						checked = np.append(checked, [[cx,cy]], axis=0)

						if np.linalg.norm(np.array([cx-cx0,cy-cy0]))< 1.2*equi_radius: #if its within the "bridge" body

							
							hull_area = cv2.contourArea(cv2.convexHull(c))

							if hull_area != 0:
								solidity = float(M['m00'])/hull_area
							else:
								solidity = 0
							# print 'solidity: ',solidity
							if solidity>.8:


								rect=cv2.minAreaRect(c)
								# print '--'
								# print rect

								angle=rect[2]
								L=rect[1][0]
								W=rect[1][1]

								cv2.drawContours(image_raw, [c], -1, 0, 2)
								if L<W: #if length is shorter than width. then the angle is actually 90deg off
									W=rect[1][0]
									L=rect[1][1]
									if rect[2]<0:
										angle=rect[2]+90
									else:
										angle=rect[2]-90

								print angle, L/W
								if np.abs((L/W)-3)<.66: #if its got the aspect ratio of the hole
									print 'got a rectangle'
									cv2.drawContours(image_raw, [c], -1, 125, 3)
									if first_hole_angle==None:
										first_hole_angle=angle
									else:
										if np.abs(angle-first_hole_angle) < 5:
											got_bridge=True

	if got_bridge or first_hole_angle is not None:

		if got_bridge==False:
			print 'NOT SUPER SURE'
		angle=(first_hole_angle+90) * 3.14/180
		bridge_angle=angle
		
		x2 = int(round(cx0 + 1000* np.cos(angle)))
		y2 = int(round(cy0 + 1000 * np.sin(angle)))
		x1 = int(round(cx0 - 1000* np.cos(angle)))
		y1 = int(round(cy0 - 1000 * np.sin(angle) ))


		cv2.line(image_raw,(x1,y1),(x2,y2),(0,255,0),2)
		cv2.circle(image_raw,(int(cx0),int(cy0)),3,(255,0,0),-1)

		# cv2.imshow('image_raw',image_raw)
	
	
	# print("--- %s seconds ---" % (time.time() - start_time))
	# cv2.waitKey(0)
	# cv2.destroyAllWindows()
	# plt.cla()
	# plt.imshow(image_raw)
	# plt.pause(.005)
	

	return got_bridge, image_raw, cx0, cy0, bridge_angle

def img_callback(data):
	img = bridge.imgmsg_to_cv2(data, "bgr8")

	img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
	print 'somethings happening'
	# interference_ratio= .14 #how far from the left side of the screen the lense shit is in frame
	
	# x_start= int(interference_ratio*img.shape[1])
	# img= img[:,x_start:]

	#scale image down for speed, reduces accuracy
	imgScale=1

	# newX,newY = img.shape[1]*imgScale, img.shape[0]*imgScale
	# img = cv2.resize(img,(int(newX),int(newY)),cv2.INTER_AREA)

	gotit, img_center,x,y, angle = find_bridge(img)


	##########################################################
	
	# img_center=image_raw
	# x=cx0
	# y=cy0
	# angle=bridge_angle

	##########################################################
	
	outt=Point()

	if gotit:
		outt.x= int(x*(1/imgScale))
		outt.y= int(y*(1/imgScale))
		outt.z= angle
	else:
		if x==0 and y==0:
			outt.x= 0
			outt.y= 0
			outt.z= 0
		else:
		#if not super sure then iffy .5 solution
			outt.x= int(x*(1/imgScale))+.5
			outt.y= int(y*(1/imgScale))+.5
			outt.z= angle

	img_center = cv2.cvtColor(img_center,cv2.COLOR_GRAY2BGR)
	img_pub.publish(bridge.cv2_to_imgmsg(img_center, "bgr8"))
	bridge_pub.publish(outt)

def find_bridge_main():
	rospy.init_node('find_bridge', anonymous=True)
	img_sub = rospy.Subscriber("/duo3d/left/image_rect", Image, img_callback,queue_size=1,buff_size=52428800)
	rospy.spin()


if __name__ == '__main__':
	try:
		
		find_bridge_main()
	except rospy.ROSInterruptException:
		pass
